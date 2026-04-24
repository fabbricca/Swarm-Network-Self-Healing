#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <fstream>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"

#include "modules/controller/controller_factory.h"
#include "platform/ns3/base_station/ns3_base_station.h"
#include "platform/ns3/drone/ns3_drone.h"
#include "platform/ns3/uwb_anchor/ns3_uwb_anchor.h"
#include "platform/ns3/uwb_channel/uwb_channel.h"
#include "platform/ns3/uwb_channel/uwb_energy_params.h"

using namespace ns3;

namespace {

void EnsureMobility(Ptr<Node> node, const Vector& pos) {
  auto mob = node->GetObject<ConstantPositionMobilityModel>();
  if (!mob) {
    mob = CreateObject<ConstantPositionMobilityModel>();
    node->AggregateObject(mob);
  }
  mob->SetPosition(pos);
}

// Multi-base partition: v1 supports up to 3 bases.  Base 1 uses id 0 (legacy
// single-base value), bases 2/3 use ids 250/251 so they never collide with
// drone or anchor ids (drones 1..20, anchors 21..38).
constexpr size_t MAX_BASES = 3;
constexpr uint8_t BASE_IDS[MAX_BASES] = {0, 250, 251};

// Parse an optional override file of the form:
//   # comments allowed
//   base=x,y,z                 (alias for base1=)
//   baseN=x,y,z                (N in 1..MAX_BASES)
//   droneN=x,y,z               (N in 1..NUM_DRONES)
//   droneN_base=M              (v1 scenario files only; silently ignored in v2)
//   anchorN=x,y,z              (N in 1..NUM_ANCHORS)
//   kill_droneN=T              (schedule mid-sim drone kill)
//   kill_baseN=T               (schedule mid-sim base kill, N in 1..MAX_BASES)
// Missing keys keep the simulator's hardcoded defaults. Unknown keys are
// treated as hard errors so typos can't silently no-op.
bool loadScenarioOverrides(
    const std::string& path,
    std::vector<std::optional<Vector>>& baseOverrides,
    std::vector<std::optional<Vector>>& droneOverrides,
    std::vector<std::optional<Vector>>& anchorOverrides,
    std::vector<std::pair<size_t, double>>& droneKills,
    std::vector<std::pair<size_t, double>>& baseKills) {
  std::ifstream in(path);
  if (!in.is_open()) {
    std::cerr << "[Sim] scenarioFile: unable to open " << path << std::endl;
    return false;
  }

  const size_t numDrones = droneOverrides.size();
  const size_t numAnchors = anchorOverrides.size();
  const size_t numBases = baseOverrides.size();

  auto parseVec = [&](const std::string& value, Vector& out, const std::string& line) -> bool {
    double coords[3] = {0.0, 0.0, 0.0};
    size_t start = 0;
    for (int i = 0; i < 3; ++i) {
      size_t end = value.find(',', start);
      if (i < 2 && end == std::string::npos) {
        std::cerr << "[Sim] scenarioFile: expected 3 comma-separated numbers on line: " << line << std::endl;
        return false;
      }
      std::string token = value.substr(start, end - start);
      try {
        coords[i] = std::stod(token);
      } catch (...) {
        std::cerr << "[Sim] scenarioFile: cannot parse number '" << token << "' on line: " << line << std::endl;
        return false;
      }
      start = (end == std::string::npos) ? value.size() : end + 1;
    }
    out = Vector(coords[0], coords[1], coords[2]);
    return true;
  };

  std::string line;
  size_t lineno = 0;
  while (std::getline(in, line)) {
    ++lineno;
    // Strip trailing CR (tolerate CRLF) and leading/trailing whitespace.
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ' || line.back() == '\t')) line.pop_back();
    size_t first = line.find_first_not_of(" \t");
    if (first == std::string::npos) continue;
    line = line.substr(first);
    if (line[0] == '#') continue;

    size_t eq = line.find('=');
    if (eq == std::string::npos) {
      std::cerr << "[Sim] scenarioFile: missing '=' on line " << lineno << ": " << line << std::endl;
      return false;
    }
    std::string key = line.substr(0, eq);
    std::string val = line.substr(eq + 1);

    // kill_droneN=T has a scalar RHS (seconds), not a vec3.
    if (key.rfind("kill_drone", 0) == 0) {
      try {
        size_t idx = static_cast<size_t>(std::stoul(key.substr(10)));
        if (idx < 1 || idx > numDrones) {
          std::cerr << "[Sim] scenarioFile: kill_drone index " << idx
                    << " out of range 1.." << numDrones << " on line " << lineno << std::endl;
          return false;
        }
        double at_s = std::stod(val);
        if (at_s < 0.0) {
          std::cerr << "[Sim] scenarioFile: kill time must be >= 0 on line "
                    << lineno << ": " << line << std::endl;
          return false;
        }
        droneKills.emplace_back(idx, at_s);
      } catch (...) {
        std::cerr << "[Sim] scenarioFile: bad kill entry '" << line
                  << "' on line " << lineno << std::endl;
        return false;
      }
      continue;
    }

    // kill_baseN=T: schedule a base-station kill.  N is 1-based over the
    // MAX_BASES slots (not the physical BASE_IDS).
    if (key.rfind("kill_base", 0) == 0) {
      try {
        size_t idx = static_cast<size_t>(std::stoul(key.substr(9)));
        if (idx < 1 || idx > MAX_BASES) {
          std::cerr << "[Sim] scenarioFile: kill_base index " << idx
                    << " out of range 1.." << MAX_BASES << " on line " << lineno << std::endl;
          return false;
        }
        double at_s = std::stod(val);
        if (at_s < 0.0) {
          std::cerr << "[Sim] scenarioFile: kill time must be >= 0 on line "
                    << lineno << ": " << line << std::endl;
          return false;
        }
        baseKills.emplace_back(idx, at_s);
      } catch (...) {
        std::cerr << "[Sim] scenarioFile: bad kill_base entry '" << line
                  << "' on line " << lineno << std::endl;
        return false;
      }
      continue;
    }

    // droneN_base=M: v1 pinning directive.  v2 drones roam freely across all
    // registered bases, so this is silently accepted (for scenario-file
    // back-compat) but has no effect.  We still validate the format so a
    // typo like "droneN_bse=1" still errors via the unknown-key path.
    {
      const size_t under = key.find('_');
      if (under != std::string::npos
          && key.rfind("drone", 0) == 0
          && key.substr(under) == "_base") {
        try {
          const size_t drone_idx = static_cast<size_t>(std::stoul(key.substr(5, under - 5)));
          if (drone_idx < 1 || drone_idx > numDrones) {
            std::cerr << "[Sim] scenarioFile: drone index " << drone_idx
                      << " out of range 1.." << numDrones << " on line " << lineno << std::endl;
            return false;
          }
          const size_t base_idx = static_cast<size_t>(std::stoul(val));
          if (base_idx < 1 || base_idx > numBases) {
            std::cerr << "[Sim] scenarioFile: drone base assignment "
                      << base_idx << " out of range 1.." << numBases
                      << " on line " << lineno << std::endl;
            return false;
          }
          // v2: drones are not pinned.  Silently accept the directive for
          // back-compat with v1 scenario files but ignore its effect.
          (void)drone_idx;
          (void)base_idx;
        } catch (...) {
          std::cerr << "[Sim] scenarioFile: bad drone_base entry '" << line
                    << "' on line " << lineno << std::endl;
          return false;
        }
        continue;
      }
    }

    Vector pos;
    if (!parseVec(val, pos, line)) return false;

    if (key == "base") {
      // Legacy single-base alias.
      baseOverrides[0] = pos;
    } else if (key.rfind("base", 0) == 0 && key.size() > 4
               && std::all_of(key.begin() + 4, key.end(), ::isdigit)) {
      try {
        size_t idx = static_cast<size_t>(std::stoul(key.substr(4)));
        if (idx < 1 || idx > numBases) {
          std::cerr << "[Sim] scenarioFile: base index " << idx
                    << " out of range 1.." << numBases << " on line " << lineno << std::endl;
          return false;
        }
        baseOverrides[idx - 1] = pos;
      } catch (...) {
        std::cerr << "[Sim] scenarioFile: bad base key '" << key << "' on line " << lineno << std::endl;
        return false;
      }
    } else if (key.rfind("drone", 0) == 0) {
      try {
        size_t idx = static_cast<size_t>(std::stoul(key.substr(5)));
        if (idx < 1 || idx > numDrones) {
          std::cerr << "[Sim] scenarioFile: drone index " << idx
                    << " out of range 1.." << numDrones << " on line " << lineno << std::endl;
          return false;
        }
        droneOverrides[idx - 1] = pos;
      } catch (...) {
        std::cerr << "[Sim] scenarioFile: bad drone key '" << key << "' on line " << lineno << std::endl;
        return false;
      }
    } else if (key.rfind("anchor", 0) == 0) {
      try {
        size_t idx = static_cast<size_t>(std::stoul(key.substr(6)));
        if (idx < 1 || idx > numAnchors) {
          std::cerr << "[Sim] scenarioFile: anchor index " << idx
                    << " out of range 1.." << numAnchors << " on line " << lineno << std::endl;
          return false;
        }
        anchorOverrides[idx - 1] = pos;
      } catch (...) {
        std::cerr << "[Sim] scenarioFile: bad anchor key '" << key << "' on line " << lineno << std::endl;
        return false;
      }
    } else {
      std::cerr << "[Sim] scenarioFile: unknown key '" << key << "' on line " << lineno << std::endl;
      return false;
    }
  }
  return true;
}

// ── Aggregation helpers for end-of-sim metrics ──
struct AggStats {
  size_t count = 0;
  double avg = 0.0;
  double p50 = 0.0;
  double p95 = 0.0;
  double max = 0.0;
  double stddev = 0.0;
};

double percentile(std::vector<double> values, double p) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  if (values.size() == 1) return values[0];
  const double rank = p * static_cast<double>(values.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(rank));
  const size_t hi = static_cast<size_t>(std::ceil(rank));
  const double frac = rank - static_cast<double>(lo);
  return values[lo] + (values[hi] - values[lo]) * frac;
}

AggStats aggregateStats(const std::vector<double>& v) {
  AggStats s;
  s.count = v.size();
  if (v.empty()) return s;
  const double sum = std::accumulate(v.begin(), v.end(), 0.0);
  s.avg = sum / static_cast<double>(v.size());
  s.p50 = percentile(v, 0.50);
  s.p95 = percentile(v, 0.95);
  s.max = *std::max_element(v.begin(), v.end());
  double sq = 0.0;
  for (double x : v) { const double d = x - s.avg; sq += d * d; }
  s.stddev = std::sqrt(sq / static_cast<double>(v.size()));
  return s;
}

std::string fmtNum(double v, int precision = 2) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << v;
  return oss.str();
}

}  // namespace

int main(int argc, char* argv[]) {
  Time::SetResolution(Time::NS);

  // Self-healing protocol — multi-hop relay chain across 20 drones:
  //
  // Sectorial layout: 6 helpers at 40 m spaced 60° around the base,
  // 14 lost drones at 75-85 m split across the 6 sectors (3-2-3-2-2-2).
  // This forces each helper to serve a distinct subset of lost drones and
  // makes cross-sector relay an opt-in edge case rather than the default.

  double maxRangeMeters = 50.0;
  double simSeconds = 300.0;
  double kAtt = 0.3;
  double kRep = 8.0;
  double dSafe = 2.0;
  double vMax = 1.0;
  double droneWeightKg = 0.029;
  double uwbNoiseStdDev = 0.0;
  std::string csvOut = "";
  std::string animOut = "/output/drone-simulation.xml";
  std::string algorithmName = "centroid";
  std::string scenarioFile = "";


  CommandLine cmd;
  cmd.AddValue("algorithm", "Formation control algorithm: centroid or weighted", algorithmName);
  cmd.AddValue("maxRangeMeters", "UWB max range cutoff (coverage)", maxRangeMeters);
  cmd.AddValue("simSeconds", "Simulation stop time", simSeconds);
  cmd.AddValue("kAtt", "Controller attractive gain", kAtt);
  cmd.AddValue("kRep", "Controller repulsive gain", kRep);
  cmd.AddValue("dSafe", "Controller safety distance", dSafe);
  cmd.AddValue("vMax", "Controller max velocity", vMax);
  cmd.AddValue("droneWeightKg", "Controller drone weight (kg)", droneWeightKg);
  cmd.AddValue("uwbNoiseStdDev", "UWB LOS noise std dev in meters", uwbNoiseStdDev);
  cmd.AddValue("csvOut", "CSV path for reposition logs (empty disables)", csvOut);
  cmd.AddValue("animOut", "NetAnim XML output path (empty disables)", animOut);
  cmd.AddValue("scenarioFile", "Optional override file for base/drone/anchor positions", scenarioFile);
  cmd.Parse(argc, argv);

  const auto parsedAlgo = parseControllerAlgorithm(algorithmName);
  if (!parsedAlgo) {
    std::cerr << "[Sim] unknown --algorithm=" << algorithmName
              << " (expected 'centroid' or 'weighted')" << std::endl;
    return 1;
  }
  const ControllerAlgorithm algorithm = *parsedAlgo;
  std::cout << "[Sim] formation-control algorithm: "
            << controllerAlgorithmName(algorithm) << std::endl;

  sim::UwbChannelConfig uwbCfg;
  uwbCfg.maxRangeMeters = maxRangeMeters;
  sim::UwbChannel::Get().Configure(uwbCfg);

  constexpr uint32_t NUM_DRONES = 20;
  constexpr uint32_t NUM_ANCHORS = 18;  // standalone UWB anchors (base is also an anchor)

  // Sectorial default positions; a scenarioFile may selectively override any subset.
  // Drones 1-6 are helpers at 40 m radius spaced 60° around the base.
  // Drones 7-20 are 14 lost drones spread across the same 6 sectors at 75-85 m.
  const Vector baseDefault(0.0, 0.0, 0.0);
  const Vector droneDefaults[NUM_DRONES] = {
    {  40.00,   0.00, 0.0},  // D1  helper  (0°, 40m)
    {  20.00,  34.64, 0.0},  // D2  helper  (60°)
    { -20.00,  34.64, 0.0},  // D3  helper  (120°)
    { -40.00,   0.00, 0.0},  // D4  helper  (180°)
    { -20.00, -34.64, 0.0},  // D5  helper  (240°)
    {  20.00, -34.64, 0.0},  // D6  helper  (300°)
    {  72.44, -19.41, 0.0},  // D7  lost    sector D1 (-15°, 75m)
    {  85.00,   0.00, 0.0},  // D8  lost    sector D1 (  0°, 85m)
    {  72.44,  19.41, 0.0},  // D9  lost    sector D1 ( 15°, 75m)
    {  53.03,  53.03, 0.0},  // D10 lost    sector D2 ( 45°, 75m)
    {  20.71,  77.27, 0.0},  // D11 lost    sector D2 ( 75°, 80m)
    { -19.41,  72.44, 0.0},  // D12 lost    sector D3 (105°, 75m)
    { -42.50,  73.61, 0.0},  // D13 lost    sector D3 (120°, 85m)
    { -53.03,  53.03, 0.0},  // D14 lost    sector D3 (135°, 75m)
    { -72.44,  19.41, 0.0},  // D15 lost    sector D4 (165°, 75m)
    { -77.27, -20.71, 0.0},  // D16 lost    sector D4 (195°, 80m)
    { -56.57, -56.57, 0.0},  // D17 lost    sector D5 (225°, 80m)
    { -19.41, -72.44, 0.0},  // D18 lost    sector D5 (255°, 75m)
    {  20.71, -77.27, 0.0},  // D19 lost    sector D6 (285°, 80m)
    {  53.03, -53.03, 0.0},  // D20 lost    sector D6 (315°, 75m)
  };
  const Vector anchorDefaults[NUM_ANCHORS] = {
    // Middle ring at 50 m, 12 anchors at 30° spacing (dense so every drone
    // sees ≥2 non-collinear anchors here regardless of angular position).
    {  50.00,   0.00, 0.0},   // A1  (  0°)
    {  43.30,  25.00, 0.0},   // A2  ( 30°)
    {  25.00,  43.30, 0.0},   // A3  ( 60°)
    {   0.00,  50.00, 0.0},   // A4  ( 90°)
    { -25.00,  43.30, 0.0},   // A5  (120°)
    { -43.30,  25.00, 0.0},   // A6  (150°)
    { -50.00,   0.00, 0.0},   // A7  (180°)
    { -43.30, -25.00, 0.0},   // A8  (210°)
    { -25.00, -43.30, 0.0},   // A9  (240°)
    {   0.00, -50.00, 0.0},   // A10 (270°)
    {  25.00, -43.30, 0.0},   // A11 (300°)
    {  43.30, -25.00, 0.0},   // A12 (330°)
    // Outer ring at 75 m, 6 anchors on helper angles (backstop for farthest
    // lost drones so they always see ≥1 outer anchor + multiple middle-ring).
    {  75.00,   0.00, 0.0},   // A13 (  0°)
    {  37.50,  64.95, 0.0},   // A14 ( 60°)
    { -37.50,  64.95, 0.0},   // A15 (120°)
    { -75.00,   0.00, 0.0},   // A16 (180°)
    { -37.50, -64.95, 0.0},   // A17 (240°)
    {  37.50, -64.95, 0.0},   // A18 (300°)
  };

  std::vector<std::optional<Vector>> baseOverrides(MAX_BASES);
  std::vector<std::optional<Vector>> droneOverrides(NUM_DRONES);
  std::vector<std::optional<Vector>> anchorOverrides(NUM_ANCHORS);
  std::vector<std::pair<size_t, double>> droneKills;   // (1-based drone index, kill time)
  std::vector<std::pair<size_t, double>> baseKills;    // (1-based base slot, kill time)
  if (!scenarioFile.empty()) {
    if (!loadScenarioOverrides(scenarioFile, baseOverrides, droneOverrides, anchorOverrides,
                                droneKills, baseKills)) {
      return 1;
    }
    std::cout << "[Sim] scenarioFile: " << scenarioFile << " applied" << std::endl;
  }

  // Single-base legacy path: if nothing configured, keep the compiled-in default.
  if (!baseOverrides[0].has_value()) {
    baseOverrides[0] = baseDefault;
  }

  // Count active bases (contiguous from slot 0; no gaps allowed).
  uint32_t numBases = 0;
  for (size_t i = 0; i < MAX_BASES; ++i) {
    if (baseOverrides[i].has_value()) {
      if (numBases != i) {
        std::cerr << "[Sim] scenarioFile: base" << (i + 1)
                  << " defined but base" << numBases + 1 << " is not — bases must be contiguous\n";
        return 1;
      }
      ++numBases;
    }
  }

  // Validate base-kill targets.
  for (const auto& [idx1, at_s] : baseKills) {
    (void)at_s;
    if (idx1 < 1 || idx1 > numBases) {
      std::cerr << "[Sim] scenarioFile: kill_base" << idx1
                << " but only " << numBases << " base(s) defined\n";
      return 1;
    }
  }

  NodeContainer nodes;
  nodes.Create(numBases + NUM_DRONES + NUM_ANCHORS);
  // Node layout:
  //   [0 .. numBases-1]:                             base stations
  //   [numBases .. numBases+NUM_DRONES-1]:           drones
  //   [numBases+NUM_DRONES .. +NUM_ANCHORS-1]:       standalone UWB anchors

  // Base stations: each at its own node, ids from BASE_IDS[].
  for (uint32_t b = 0; b < numBases; ++b) {
    EnsureMobility(nodes.Get(b), baseOverrides[b].value());
  }

  // Drones: positions from override when present, else the default.
  // Snapshot initial positions for the end-of-sim displacement table.
  std::vector<Vector> initialDronePos(NUM_DRONES);
  for (uint32_t i = 0; i < NUM_DRONES; ++i) {
    initialDronePos[i] = droneOverrides[i].value_or(droneDefaults[i]);
    EnsureMobility(nodes.Get(numBases + i), initialDronePos[i]);
  }

  // UWB anchors: positions from override when present, else default.
  for (uint32_t i = 0; i < NUM_ANCHORS; ++i) {
    EnsureMobility(nodes.Get(numBases + NUM_DRONES + i), anchorOverrides[i].value_or(anchorDefaults[i]));
  }

  std::vector<std::unique_ptr<Ns3BaseStation>> bases;
  bases.reserve(numBases);
  for (uint32_t b = 0; b < numBases; ++b) {
    const Vector bp = baseOverrides[b].value();
    bases.push_back(std::make_unique<Ns3BaseStation>(BASE_IDS[b], nodes.Get(b)));
    bases.back()->setPosition(bp.x, bp.y, bp.z);
  }

  // Create standalone UWB anchors.
  std::vector<std::unique_ptr<Ns3UwbAnchor>> anchors;
  anchors.reserve(NUM_ANCHORS);
  for (uint32_t i = 0; i < NUM_ANCHORS; ++i) {
    // Anchor IDs must stay clear of drone IDs (1..NUM_DRONES) AND the
    // high-range base IDs (BASE_IDS).  We keep the historical scheme
    // NUM_DRONES+1..NUM_DRONES+NUM_ANCHORS which never collides with drones
    // or with our chosen BASE_IDS of 0 / 250 / 251.
    uint8_t anchor_id = static_cast<uint8_t>(NUM_DRONES + 1 + i);
    anchors.push_back(std::make_unique<Ns3UwbAnchor>(anchor_id, nodes.Get(numBases + NUM_DRONES + i)));
    Vector ap = anchorOverrides[i].value_or(anchorDefaults[i]);
    anchors.back()->setPosition(ap.x, ap.y, ap.z);
  }

  std::vector<std::unique_ptr<Ns3Drone>> drones;
  drones.reserve(NUM_DRONES);

  std::shared_ptr<std::ofstream> csv;
  if (!csvOut.empty()) {
    csv = std::make_shared<std::ofstream>(csvOut);
    if (csv->good()) {
      (*csv) << "t,drone,hops,neighbors,x,y,z" << std::endl;
    } else {
      std::cerr << "[Sim] failed to open csvOut=" << csvOut << std::endl;
      csv.reset();
    }
  }

  for (uint32_t i = 0; i < NUM_DRONES; ++i) {
    // Drone IDs remain 1..NUM_DRONES (not shifted by numBases) to keep
    // existing id-based heuristics intact.
    drones.push_back(std::make_unique<Ns3Drone>(
      static_cast<uint8_t>(i + 1),
      nodes.Get(numBases + i),
      algorithm,
      static_cast<float>(kAtt),
      static_cast<float>(kRep),
      static_cast<float>(dSafe),
      static_cast<float>(vMax),
      static_cast<float>(droneWeightKg),
      uwbNoiseStdDev
    ));
    if (csv) {
      drones.back()->setRepositionLogger(csv);
    }
  }

  // v2 roaming: every drone registers every base.  Each drone tracks all
  // bases and dynamically picks the nearest reachable one per tick.
  for (uint32_t i = 0; i < NUM_DRONES; ++i) {
    for (uint32_t b = 0; b < numBases; ++b) {
      drones[i]->registerBase(bases[b]->id());
      bases[b]->registerDrone(drones[i]->id());
    }
  }

  // Each base triggers its own floods periodically.
  for (const auto& b : bases) {
    b->start();
  }

  // Start standalone UWB anchor beacon broadcasting.
  for (const auto& a : anchors) {
    a->start();
  }

  // Start drones' periodic ticks (POS_UPDATE/ACK tracking + idle motion + HELP_PROXY timeout).
  for (const auto& d : drones) {
    d->start();
  }

  // Schedule mid-simulation drone kills.
  for (const auto& [idx1, at_s] : droneKills) {
    const size_t idx0 = idx1 - 1;
    Ns3Drone* drone_ptr = drones[idx0].get();
    Simulator::Schedule(Seconds(at_s), [drone_ptr]() { drone_ptr->kill(); });
    std::cout << "[Sim] scheduled kill: drone=" << idx1 << " at t=" << at_s << "s" << std::endl;
  }

  // Schedule mid-simulation base kills.  idx1 is 1-based slot over numBases.
  for (const auto& [idx1, at_s] : baseKills) {
    const size_t idx0 = idx1 - 1;
    Ns3BaseStation* base_ptr = bases[idx0].get();
    Simulator::Schedule(Seconds(at_s), [base_ptr]() { base_ptr->kill(); });
    std::cout << "[Sim] scheduled kill: base" << idx1
              << " (id=" << static_cast<int>(base_ptr->id())
              << ") at t=" << at_s << "s" << std::endl;
  }

  std::cout << "[Sim] base coverage=" << maxRangeMeters
            << "m, bases=" << numBases
            << ", drones=" << NUM_DRONES
            << ", uwb_anchors=" << NUM_ANCHORS
            << ", stop=" << simSeconds << "s" << std::endl;

  AnimationInterface anim(animOut);
  anim.SetBackgroundImage("whiteBackground.png", -100, -100, 200, 200, true);
  uint32_t baseStationIcon = anim.AddResource("baseStation.png");
  uint32_t droneIcon = anim.AddResource("drone.png");
  uint32_t uwbAnchorIcon = anim.AddResource("uwbAnchor.png");

  for (uint32_t b = 0; b < numBases; ++b) {
    anim.UpdateNodeImage(b, baseStationIcon);
    anim.UpdateNodeSize(b, 10, 10);
  }
  for (uint32_t i = 0; i < NUM_DRONES; ++i) {
    const uint32_t nodeIdx = numBases + i;
    anim.UpdateNodeImage(nodeIdx, droneIcon);
    anim.UpdateNodeSize(nodeIdx, 10, 10);
  }
  for (uint32_t i = 0; i < NUM_ANCHORS; ++i) {
    const uint32_t nodeIdx = numBases + NUM_DRONES + i;
    anim.UpdateNodeImage(nodeIdx, uwbAnchorIcon);
    anim.UpdateNodeSize(nodeIdx, 6, 6);
  }
  
  Simulator::Stop(Seconds(simSeconds));
  Simulator::Run();

  // ── End-of-simulation performance metrics ──
  std::cout << "\n========== END-OF-SIMULATION METRICS ==========\n";

  // ── 0. Scheduled failures (only if any kills were configured) ──
  // Printed first so later blocks (Healing, Return, FinalCoverage) are read in
  // the context of "we killed drone X at time T".
  if (!droneKills.empty()) {
    std::cout << "\n── Scheduled Failures ──\n";
    std::cout << std::left
              << std::setw(8)  << "Drone"
              << std::setw(14) << "KilledAt(s)"
              << std::setw(12) << "FinalHops"
              << "LastGtPos\n";

    uint32_t executed = 0;
    for (const auto& [idx1, at_s] : droneKills) {
      (void)at_s;
      const size_t idx0 = idx1 - 1;
      const auto& d = drones[idx0];
      const double ks = d->killedAtS();
      const bool ran = (ks >= 0.0);
      if (ran) ++executed;

      const ::Vector3D p = d->currentGtPos();
      const uint8_t h = d->hopsFromBase();
      std::string hops_str = (h == 0xFF) ? "?" : std::to_string(static_cast<int>(h));

      std::ostringstream posstr;
      posstr << "(" << fmtNum(p.x) << "," << fmtNum(p.y) << "," << fmtNum(p.z) << ")";

      std::cout << std::left
                << std::setw(8)  << static_cast<int>(d->id())
                << std::setw(14) << (ran ? fmtNum(ks) : std::string("-"))
                << std::setw(12) << hops_str
                << posstr.str() << "\n";
    }
    std::cout << "Failures: scheduled=" << droneKills.size()
              << "  executed=" << executed << "\n";
  }

  // Scheduled base failures — separate block so drone-failure regexes in the
  // runner don't get confused with base-failure data.
  if (!baseKills.empty()) {
    std::cout << "\n── Scheduled Base Failures ──\n";
    std::cout << std::left
              << std::setw(8)  << "Base"
              << std::setw(14) << "KilledAt(s)"
              << "Pos\n";

    uint32_t bExecuted = 0;
    for (const auto& [idx1, at_s] : baseKills) {
      (void)at_s;
      const size_t idx0 = idx1 - 1;
      const auto& b = bases[idx0];
      const double ks = b->killedAtS();
      const bool ran = (ks >= 0.0);
      if (ran) ++bExecuted;

      const Vector bp = baseOverrides[idx0].value();
      std::ostringstream posstr;
      posstr << "(" << fmtNum(bp.x) << "," << fmtNum(bp.y) << "," << fmtNum(bp.z) << ")";

      std::cout << std::left
                << std::setw(8)  << static_cast<int>(b->id())
                << std::setw(14) << (ran ? fmtNum(ks) : std::string("-"))
                << posstr.str() << "\n";
    }
    std::cout << "BaseFailures: scheduled=" << baseKills.size()
              << "  executed=" << bExecuted << "\n";
  }

  // Collect final positions for all drones.
  struct DroneInfo { uint8_t id; bool lost; uint8_t hops; Vector gt; std::vector<double> tri; double err; };
  std::vector<DroneInfo> info;
  double total_error = 0.0, min_error = std::numeric_limits<double>::max(), max_error = 0.0;

  for (uint32_t i = 0; i < NUM_DRONES; ++i) {
    auto mob = nodes.Get(numBases + i)->GetObject<ConstantPositionMobilityModel>();
    Vector gt = mob->GetPosition();
    auto* pos = drones[i]->position();
    std::vector<double> tri = pos ? pos->getCoordinates() : std::vector<double>{0.0, 0.0, 0.0};
    double dx = gt.x - tri[0], dy = gt.y - tri[1], dz = gt.z - tri[2];
    double err = std::sqrt(dx*dx + dy*dy + dz*dz);
    min_error = std::min(min_error, err);
    max_error = std::max(max_error, err);
    total_error += err;
    bool lost = (drones[i]->id() >= 7);
    uint8_t hops = drones[i]->hopsFromBase();
    info.push_back({drones[i]->id(), lost, hops, gt, tri, err});
  }

  // ── 1. Drone positions, hop count & trilateration error ──
  std::cout << "\n── Drone Positions, Hops & Trilateration Error ──\n";
  std::cout << std::left
            << std::setw(8)  << "Drone"
            << std::setw(10) << "Role"
            << std::setw(7)  << "Hops"
            << std::setw(28) << "Ground truth (x,y,z)"
            << std::setw(28) << "Trilaterated (x,y,z)"
            << "Error\n";
  for (const auto& d : info) {
    std::string hops_str = (d.hops == 0xFF) ? "?" : std::to_string(static_cast<int>(d.hops));
    std::string gt_str   = "(" + std::to_string(d.gt.x) + "," + std::to_string(d.gt.y) + "," + std::to_string(d.gt.z) + ")";
    std::string tri_str  = "(" + std::to_string(d.tri[0]) + "," + std::to_string(d.tri[1]) + "," + std::to_string(d.tri[2]) + ")";
    std::cout << std::left
              << std::setw(8)  << static_cast<int>(d.id)
              << std::setw(10) << (d.lost ? "lost" : "helper")
              << std::setw(7)  << hops_str
              << std::setw(28) << gt_str
              << std::setw(28) << tri_str
              << d.err << "m\n";
  }
  std::cout << "Trilateration: min=" << min_error
            << "m  max=" << max_error
            << "m  avg=" << (total_error / NUM_DRONES) << "m\n";

  // ── 1b. Initial vs final positions (displacement table) ──
  // Captures how far each drone actually moved from its spawn, useful at a
  // glance for sanity-checking returning-mode behavior (lost drones that
  // completed return should have final ≈ some hop-1 coverage position).
  std::cout << "\n── Initial vs Final Positions ──\n";
  std::cout << std::left
            << std::setw(8)  << "Drone"
            << std::setw(10) << "Role"
            << std::setw(28) << "Initial (x,y,z)"
            << std::setw(28) << "Final (x,y,z)"
            << "Displacement (m)\n";
  for (uint32_t i = 0; i < NUM_DRONES; ++i) {
    const Vector& ip = initialDronePos[i];
    const Vector& fp = info[i].gt;
    double ddx = fp.x - ip.x, ddy = fp.y - ip.y, ddz = fp.z - ip.z;
    double disp = std::sqrt(ddx*ddx + ddy*ddy + ddz*ddz);
    std::string ip_str = "(" + std::to_string(ip.x) + "," + std::to_string(ip.y) + "," + std::to_string(ip.z) + ")";
    std::string fp_str = "(" + std::to_string(fp.x) + "," + std::to_string(fp.y) + "," + std::to_string(fp.z) + ")";
    std::cout << std::left
              << std::setw(8)  << static_cast<int>(info[i].id)
              << std::setw(10) << (info[i].lost ? "lost" : "helper")
              << std::setw(28) << ip_str
              << std::setw(28) << fp_str
              << disp << "\n";
  }

  // ── 2. Packet receive counts by type ──
  std::cout << "\n── Packet Receive Counts (by type) ──\n";
  std::cout << std::left
            << std::setw(8)  << "Drone"
            << std::setw(10) << "Role"
            << std::setw(7)  << "Hops"
            << std::setw(13) << "POS_UPDATE"
            << std::setw(10) << "POS_ACK"
            << std::setw(13) << "HELP_PROXY"
            << std::setw(8)  << "FLOOD"
            << std::setw(11) << "NEIGHBOR"
            << "UWB_BEACON\n";
  for (uint32_t i = 0; i < NUM_DRONES; ++i) {
    const auto& s = drones[i]->rxStats();
    std::string hops_str = (info[i].hops == 0xFF) ? "?" : std::to_string(static_cast<int>(info[i].hops));
    std::cout << std::left
              << std::setw(8)  << static_cast<int>(drones[i]->id())
              << std::setw(10) << (info[i].lost ? "lost" : "helper")
              << std::setw(7)  << hops_str
              << std::setw(13) << s.pos_update
              << std::setw(10) << s.pos_ack
              << std::setw(13) << s.help_proxy
              << std::setw(8)  << s.flood
              << std::setw(11) << s.neighbor
              << s.uwb_beacon << "\n";
  }

  // ── 3. Distance traveled (ground truth) ──
  // Sampled from the mobility model each physics tick, so this is independent
  // of trilateration noise.  Aggregated per role so variants can be compared
  // on how much their helpers (and lost drones) actually move.
  std::cout << "\n── Distance Traveled (ground truth) ──\n";
  std::cout << std::left
            << std::setw(8)  << "Drone"
            << std::setw(10) << "Role"
            << std::setw(7)  << "Hops"
            << "Distance (m)\n";
  double dist_helper_total = 0.0, dist_lost_total = 0.0;
  uint32_t n_helper = 0, n_lost = 0;
  for (uint32_t i = 0; i < NUM_DRONES; ++i) {
    const double d_m = drones[i]->totalDistanceTraveled();
    std::string hops_str = (info[i].hops == 0xFF) ? "?" : std::to_string(static_cast<int>(info[i].hops));
    std::cout << std::left
              << std::setw(8)  << static_cast<int>(drones[i]->id())
              << std::setw(10) << (info[i].lost ? "lost" : "helper")
              << std::setw(7)  << hops_str
              << d_m << "\n";
    if (info[i].lost) { dist_lost_total += d_m; ++n_lost; }
    else              { dist_helper_total += d_m; ++n_helper; }
  }
  const double dist_total = dist_helper_total + dist_lost_total;
  std::cout << "Distance total: helper=" << dist_helper_total << "m"
            << "  lost=" << dist_lost_total << "m"
            << "  all=" << dist_total << "m\n";
  if (n_helper > 0) {
    std::cout << "Distance avg:   helper=" << (dist_helper_total / n_helper) << "m";
    if (n_lost > 0) std::cout << "  lost=" << (dist_lost_total / n_lost) << "m";
    std::cout << "\n";
  }

  // ── 4. Energy Consumption (DWM1000 estimate) ──
  // Uses Decawave DW1000 datasheet values: 70 mA TX, 113 mA RX, 12 mA idle,
  // 3.3 V supply, 6.8 Mbps data rate, ~160 µs frame overhead.
  {
    using E = sim::Dwm1000EnergyParams;
    const auto& ch = sim::UwbChannel::Get();

    std::cout << "\n── Energy Consumption (DWM1000 estimate) ──\n";
    std::cout << std::left
              << std::setw(8)  << "Node"
              << std::setw(10) << "Role"
              << std::setw(10) << "TX"
              << std::setw(10) << "RX"
              << std::setw(12) << "TX(J)"
              << std::setw(12) << "RX(J)"
              << std::setw(12) << "Idle(J)"
              << "Total(J)\n";

    double energy_base = 0.0, energy_helper = 0.0, energy_lost = 0.0, energy_anchors = 0.0;

    auto printRow = [&](uint8_t id, const std::string& role) -> double {
      const auto& es = ch.GetEnergyStats(id);
      double tx_j = es.tx_active_s * E::TX_CURRENT_A * E::SUPPLY_VOLTAGE_V;
      double rx_j = es.rx_active_s * E::RX_CURRENT_A * E::SUPPLY_VOLTAGE_V;
      double idle_s = simSeconds - es.tx_active_s - es.rx_active_s;
      if (idle_s < 0.0) idle_s = 0.0;
      double idle_j = E::idleEnergy(idle_s);
      double total  = tx_j + rx_j + idle_j;

      std::cout << std::left
                << std::setw(8)  << static_cast<int>(id)
                << std::setw(10) << role
                << std::setw(10) << es.tx_count
                << std::setw(10) << es.rx_count
                << std::setw(12) << tx_j
                << std::setw(12) << rx_j
                << std::setw(12) << idle_j
                << total << "\n";
      return total;
    };

    // Base station (id 0).
    energy_base = printRow(0, "base");

    // Drones 1..10.
    for (uint32_t i = 0; i < NUM_DRONES; ++i) {
      double e = printRow(static_cast<uint8_t>(i + 1), info[i].lost ? "lost" : "helper");
      if (info[i].lost) energy_lost += e;
      else              energy_helper += e;
    }

    // Anchors 11..22.
    for (uint32_t i = 0; i < NUM_ANCHORS; ++i) {
      energy_anchors += printRow(static_cast<uint8_t>(NUM_DRONES + 1 + i), "anchor");
    }

    double energy_all = energy_base + energy_helper + energy_lost + energy_anchors;
    std::cout << "Energy total: base=" << energy_base << "J"
              << "  helper=" << energy_helper << "J"
              << "  lost=" << energy_lost << "J"
              << "  anchors=" << energy_anchors << "J"
              << "  all=" << energy_all << "J\n";
  }

  // ── 5. Return behavior (platoon-based lost-drone return) ──
  // Per-drone timeline of the return cascade: when the drone broadcast
  // HELP_PROXY, when it armed the return timer (first relayed ACK seen),
  // when it actually started moving back (timeout expired or next-hop flag
  // received), and when it re-entered base coverage (hops==1).
  {
    std::cout << "\n── Return Behavior ──\n";
    std::cout << std::left
              << std::setw(8)  << "Drone"
              << std::setw(10) << "Role"
              << std::setw(7)  << "Hops"
              << std::setw(12) << "HelpTx(s)"
              << std::setw(12) << "Armed(s)"
              << std::setw(12) << "Start(s)"
              << std::setw(13) << "Complete(s)"
              << "Trigger\n";

    auto fmt = [](double v) -> std::string {
      if (v < 0.0) return "-";
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(2) << v;
      return oss.str();
    };

    uint32_t n_lost_total = 0, n_armed = 0, n_started = 0, n_completed = 0;
    double sum_help_to_complete = 0.0;
    uint32_t n_sum = 0;

    for (uint32_t i = 0; i < NUM_DRONES; ++i) {
      const auto& d = drones[i];
      const std::string hops_str = (info[i].hops == 0xFF) ? "?" : std::to_string(static_cast<int>(info[i].hops));
      const double help_tx  = d->helpProxyTxTime();
      const double armed_s  = d->returnArmedTime();
      const double start_s  = d->returnStartTime();
      const double done_s   = d->returnCompleteTime();
      const bool   armed    = d->returnArmed();
      const bool   started  = start_s >= 0.0;
      const bool   done     = done_s  >= 0.0;
      std::string trigger = "-";
      if (started) trigger = d->returnTriggerWasFlag() ? "flag" : "timeout";

      std::cout << std::left
                << std::setw(8)  << static_cast<int>(d->id())
                << std::setw(10) << (info[i].lost ? "lost" : "helper")
                << std::setw(7)  << hops_str
                << std::setw(12) << fmt(help_tx)
                << std::setw(12) << fmt(armed_s)
                << std::setw(12) << fmt(start_s)
                << std::setw(13) << fmt(done_s)
                << trigger << "\n";

      if (info[i].lost) {
        ++n_lost_total;
        if (armed)   ++n_armed;
        if (started) ++n_started;
        if (done)    ++n_completed;
        if (done && help_tx >= 0.0) {
          sum_help_to_complete += (done_s - help_tx);
          ++n_sum;
        }
      }
    }

    std::cout << "Return summary: lost=" << n_lost_total
              << "  armed=" << n_armed
              << "  started=" << n_started
              << "  completed=" << n_completed << "\n";
    if (n_sum > 0) {
      std::cout << "Avg help_proxy→complete: "
                << (sum_help_to_complete / n_sum) << "s ("
                << n_sum << " drones)\n";
    } else {
      std::cout << "Avg help_proxy→complete: N/A (none completed in sim window)\n";
    }
  }

  // ── 6. Healing outcome ──
  // Healing latency = first ACK addressed to us AFTER our HELP_PROXY.  This
  // measures how long the relay chain takes to form, independent of how long
  // the lost drone takes to physically return.
  {
    std::cout << "\n── Healing Outcome ──\n";
    std::cout << std::left
              << std::setw(8)  << "Drone"
              << std::setw(10) << "Role"
              << std::setw(12) << "HelpTx(s)"
              << std::setw(13) << "FirstAck(s)"
              << std::setw(12) << "Healing(s)"
              << "Return(s)\n";

    auto fmtOpt = [](double v) -> std::string {
      if (v < 0.0) return "-";
      return fmtNum(v);
    };

    std::vector<double> healing_latencies;
    std::vector<double> return_latencies;
    uint32_t lost_count = 0, armed_count = 0, started_count = 0, completed_count = 0;

    for (uint32_t i = 0; i < NUM_DRONES; ++i) {
      const auto& d = drones[i];
      const bool killed = (d->killedAtS() >= 0.0);
      const double help_tx = d->helpProxyTxTime();
      const double first_ack = d->firstAckAfterHelpS();
      const double armed_s = d->returnArmedTime();
      const double done_s = d->returnCompleteTime();

      const double healing = (help_tx >= 0.0 && first_ack >= 0.0) ? (first_ack - help_tx) : -1.0;
      const double ret = (armed_s >= 0.0 && done_s >= 0.0) ? (done_s - armed_s) : -1.0;

      const char* role = killed ? "dead" : (info[i].lost ? "lost" : "helper");

      std::cout << std::left
                << std::setw(8)  << static_cast<int>(d->id())
                << std::setw(10) << role
                << std::setw(12) << fmtOpt(help_tx)
                << std::setw(13) << fmtOpt(first_ack)
                << std::setw(12) << fmtOpt(healing)
                << fmtOpt(ret) << "\n";

      if (info[i].lost && !killed) {
        ++lost_count;
        if (d->returnArmed()) ++armed_count;
        if (d->returnStartTime() >= 0.0) ++started_count;
        if (done_s >= 0.0) ++completed_count;
        if (healing >= 0.0) healing_latencies.push_back(healing);
        if (ret >= 0.0) return_latencies.push_back(ret);
      }
    }

    const auto heal = aggregateStats(healing_latencies);
    const auto retu = aggregateStats(return_latencies);
    const double rate = (lost_count > 0) ? (100.0 * completed_count / lost_count) : 0.0;

    std::cout << "Healing: drones=" << heal.count;
    if (heal.count > 0) {
      std::cout << "  avg=" << fmtNum(heal.avg) << "s"
                << "  p50=" << fmtNum(heal.p50) << "s"
                << "  p95=" << fmtNum(heal.p95) << "s"
                << "  max=" << fmtNum(heal.max) << "s";
    }
    std::cout << "\n";

    std::cout << "Return: drones=" << retu.count;
    if (retu.count > 0) {
      std::cout << "  avg=" << fmtNum(retu.avg) << "s"
                << "  p50=" << fmtNum(retu.p50) << "s"
                << "  p95=" << fmtNum(retu.p95) << "s"
                << "  max=" << fmtNum(retu.max) << "s";
    }
    std::cout << "\n";

    std::cout << "Recovery: lost=" << lost_count
              << "  armed=" << armed_count
              << "  started=" << started_count
              << "  completed=" << completed_count
              << "  rate=" << fmtNum(rate, 1) << "%\n";
  }

  // ── 7. Return quality ──
  // Return path efficiency = actual path / straight-line distance (>= 1.0).
  // Boundary distance = ‖returnComplete − base‖; should hug maxRangeMeters.
  // Rearm count = how many times the drone re-entered return mode after
  // losing direct coverage during station-keeping.
  {
    std::cout << "\n── Return Quality ──\n";
    std::cout << std::left
              << std::setw(8)  << "Drone"
              << std::setw(10) << "Role"
              << std::setw(10) << "PathEff"
              << std::setw(13) << "Boundary(m)"
              << "Rearms\n";

    auto fmtOpt = [](double v) -> std::string {
      if (v < 0.0) return "-";
      return fmtNum(v);
    };

    std::vector<double> path_effs;
    std::vector<double> boundaries;
    std::vector<double> rearms;

    for (uint32_t i = 0; i < NUM_DRONES; ++i) {
      const auto& d = drones[i];
      const bool killed = (d->killedAtS() >= 0.0);
      const double done_s = d->returnCompleteTime();
      double path_eff = -1.0;
      double boundary = -1.0;

      if (done_s >= 0.0) {
        const ::Vector3D start = d->returnStartPosGt();
        const ::Vector3D end = d->returnCompletePosGt();
        const double dx = end.x - start.x, dy = end.y - start.y, dz = end.z - start.z;
        const double straight = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double path = d->returnPhaseDistanceM();
        if (straight > 1e-3) {
          path_eff = path / straight;
        }
        // Boundary distance relative to THIS drone's CURRENT nearest base.
        // In v2 drones roam; the nearest-base attribution may have changed
        // since return-complete, but this is still the best post-hoc proxy
        // for "which base am I parked near".  Fall back to base0 if the
        // drone's nearest is unknown (UINT8_MAX).
        const uint8_t nb_id = d->nearestBaseId();
        size_t nb_slot = 0;
        for (uint32_t bi = 0; bi < numBases; ++bi) {
          if (bases[bi]->id() == nb_id) { nb_slot = bi; break; }
        }
        const Vector& assigned_base_pos = baseOverrides[nb_slot].value();
        const double bx = end.x - assigned_base_pos.x;
        const double by = end.y - assigned_base_pos.y;
        const double bz = end.z - assigned_base_pos.z;
        boundary = std::sqrt(bx * bx + by * by + bz * bz);
      }
      const uint32_t rearm = d->rearmCount();

      const char* role = killed ? "dead" : (info[i].lost ? "lost" : "helper");

      std::cout << std::left
                << std::setw(8)  << static_cast<int>(d->id())
                << std::setw(10) << role
                << std::setw(10) << fmtOpt(path_eff)
                << std::setw(13) << fmtOpt(boundary)
                << rearm << "\n";

      if (info[i].lost && !killed) {
        if (path_eff >= 0.0) path_effs.push_back(path_eff);
        if (boundary >= 0.0) boundaries.push_back(boundary);
        rearms.push_back(static_cast<double>(rearm));
      }
    }

    const auto pe = aggregateStats(path_effs);
    const auto bd = aggregateStats(boundaries);

    std::cout << "ReturnPath: drones=" << pe.count;
    if (pe.count > 0) {
      std::cout << "  avg=" << fmtNum(pe.avg)
                << "  p95=" << fmtNum(pe.p95)
                << "  max=" << fmtNum(pe.max);
    }
    std::cout << "\n";

    std::cout << "Boundary: drones=" << bd.count;
    if (bd.count > 0) {
      std::cout << "  avg=" << fmtNum(bd.avg) << "m"
                << "  stddev=" << fmtNum(bd.stddev) << "m"
                << "  target=" << fmtNum(maxRangeMeters) << "m";
    }
    std::cout << "\n";

    uint32_t rearm_total = 0, rearm_max = 0;
    for (double r : rearms) {
      rearm_total += static_cast<uint32_t>(r);
      if (r > rearm_max) rearm_max = static_cast<uint32_t>(r);
    }
    std::cout << "Rearm: drones=" << rearms.size()
              << "  total=" << rearm_total
              << "  max=" << rearm_max << "\n";
  }

  // ── 8. Post-return stability ──
  // Drift from the station-keeping reference position vs. final hop count.
  // A healthy run has StationDrift ≈ 0 and every drone reachable at ≤ 2 hops.
  {
    std::cout << "\n── Post-Return Stability ──\n";
    std::cout << std::left
              << std::setw(8)  << "Drone"
              << std::setw(10) << "Role"
              << std::setw(17) << "StationDrift(m)"
              << "FinalHops\n";

    auto fmtOpt = [](double v) -> std::string {
      if (v < 0.0) return "-";
      return fmtNum(v);
    };

    std::vector<double> drifts;
    uint32_t hops1 = 0, hops2 = 0, hops3plus = 0, lost_hop = 0, covered = 0, dead = 0;

    // Per-base coverage bucketing (v1 multi-base).  Each drone is pinned to a
    // base, so its FinalHops are already relative to that base.  We tally
    // independently per base so the end-of-sim report can attribute coverage.
    struct BaseBucket {
      uint32_t hops1 = 0, hops2 = 0, hops3plus = 0, lost_hop = 0, covered = 0, dead = 0;
    };
    std::vector<BaseBucket> per_base(numBases);

    for (uint32_t i = 0; i < NUM_DRONES; ++i) {
      const auto& d = drones[i];
      const bool killed = (d->killedAtS() >= 0.0);
      const double done_s = d->returnCompleteTime();
      const double drift = (done_s >= 0.0) ? d->stationKeepingDriftM() : -1.0;
      const uint8_t h = info[i].hops;
      std::string hops_str = (h == 0xFF) ? "?" : std::to_string(static_cast<int>(h));

      const char* role = killed ? "dead" : (info[i].lost ? "lost" : "helper");

      std::cout << std::left
                << std::setw(8)  << static_cast<int>(d->id())
                << std::setw(10) << role
                << std::setw(17) << fmtOpt(drift)
                << hops_str << "\n";

      if (info[i].lost && !killed && drift >= 0.0) drifts.push_back(drift);

      // Coverage bucketing: killed drones get their own bucket and don't count
      // toward the `lost` protocol-failure tally.  Denominator drops by K so a
      // mid-sim kill doesn't make the coverage rate look artificially worse.
      //
      // v2 roaming: attribute the drone to its CURRENT nearest base (the
      // one its hop count is reported against).  Lost / killed drones have
      // no current base -- attribute them to base slot 0 so the combined
      // totals add up but be aware per-base "lost" is a rough heuristic.
      size_t nb_slot = 0;
      const uint8_t nb_id = d->nearestBaseId();
      if (nb_id != 0xFF) {
        for (uint32_t bi = 0; bi < numBases; ++bi) {
          if (bases[bi]->id() == nb_id) { nb_slot = bi; break; }
        }
      }
      BaseBucket& bb = per_base[nb_slot];
      if (killed) {
        ++dead;
        ++bb.dead;
      } else if (h == 0xFF) {
        ++lost_hop;
        ++bb.lost_hop;
      } else {
        ++covered;
        ++bb.covered;
        if (h == 1)      { ++hops1; ++bb.hops1; }
        else if (h == 2) { ++hops2; ++bb.hops2; }
        else if (h >= 3) { ++hops3plus; ++bb.hops3plus; }
      }
    }

    const auto dr = aggregateStats(drifts);
    const uint32_t denom = NUM_DRONES - dead;
    const double cov_rate = (denom > 0) ? (100.0 * covered / denom) : 0.0;

    std::cout << "StationDrift: drones=" << dr.count;
    if (dr.count > 0) {
      std::cout << "  avg=" << fmtNum(dr.avg) << "m"
                << "  p95=" << fmtNum(dr.p95) << "m"
                << "  max=" << fmtNum(dr.max) << "m";
    }
    std::cout << "\n";

    // Combined coverage line (unchanged format — load-bearing for the
    // declaration runner's regex).
    std::cout << "FinalCoverage: covered=" << covered
              << "  total=" << denom
              << "  rate=" << fmtNum(cov_rate, 1) << "%"
              << "  hops1=" << hops1
              << "  hops2=" << hops2
              << "  hops3plus=" << hops3plus
              << "  lost=" << lost_hop
              << "  dead=" << dead << "\n";

    // Per-base breakdown, additive — emitted only when more than one base is
    // configured.  The single-base output stays byte-identical.
    if (numBases > 1) {
      for (uint32_t b = 0; b < numBases; ++b) {
        const BaseBucket& bb = per_base[b];
        uint32_t assigned = bb.covered + bb.lost_hop + bb.dead;
        const uint32_t base_denom = assigned - bb.dead;
        const double base_rate = (base_denom > 0) ? (100.0 * bb.covered / base_denom) : 0.0;
        std::cout << "FinalCoverage[base=" << b << "]: covered=" << bb.covered
                  << "  total=" << base_denom
                  << "  rate=" << fmtNum(base_rate, 1) << "%"
                  << "  hops1=" << bb.hops1
                  << "  hops2=" << bb.hops2
                  << "  hops3plus=" << bb.hops3plus
                  << "  lost=" << bb.lost_hop
                  << "  dead=" << bb.dead << "\n";
      }
    }
  }

  // ── 9. Per-helper midpoint convergence ──
  // Each helper drone only hears hop-2 drones within maxRangeMeters.  Its centroid
  // (and therefore its ideal midpoint) depends on which hop-2 drones it can see.
  //
  // Equilibrium-position semantics: for lost drones that completed return, we
  // use the position captured at return-complete (where they station-keep).
  // For helpers and non-completed drones, we use sim-end ground truth.  If no
  // lost drone completed, the block is tagged " (no-equilibrium)".
  auto baseMob = nodes.Get(0)->GetObject<ConstantPositionMobilityModel>();
  Vector basePos = baseMob->GetPosition();

  // Build equilibrium positions.
  struct EqPos { uint8_t id; bool lost; uint8_t hops; Vector pos; };
  std::vector<EqPos> eq;
  double max_return_complete_s = -1.0;
  uint32_t n_completed_eq = 0;
  for (uint32_t i = 0; i < NUM_DRONES; ++i) {
    const double done_s = drones[i]->returnCompleteTime();
    Vector pos = info[i].gt;
    if (done_s >= 0.0) {
      const ::Vector3D p = drones[i]->returnCompletePosGt();
      pos = Vector(p.x, p.y, p.z);
      if (done_s > max_return_complete_s) max_return_complete_s = done_s;
      ++n_completed_eq;
    }
    eq.push_back({info[i].id, info[i].lost, info[i].hops, pos});
  }
  const bool no_equilibrium = (n_completed_eq == 0);

  // Collect hop-2 drones (using equilibrium positions).
  std::vector<const EqPos*> hop2_drones;
  for (const auto& d : eq) {
    if (d.hops == 2) hop2_drones.push_back(&d);
  }

  // Global centroid (for reference).
  double gcx = 0.0, gcy = 0.0, gcz = 0.0;
  for (const auto* d : hop2_drones) { gcx += d->pos.x; gcy += d->pos.y; gcz += d->pos.z; }
  if (!hop2_drones.empty()) {
    double n = static_cast<double>(hop2_drones.size());
    gcx /= n; gcy /= n; gcz /= n;
  }
  double gmidX = (basePos.x + gcx) / 2.0;
  double gmidY = (basePos.y + gcy) / 2.0;
  double gmidZ = (basePos.z + gcz) / 2.0;

  std::cout << "\n── Per-Helper Midpoint Convergence ──";
  if (no_equilibrium) std::cout << " (no-equilibrium)";
  std::cout << "\n";
  std::cout << "Base station: (" << basePos.x << "," << basePos.y << "," << basePos.z << ")\n";
  if (!hop2_drones.empty()) {
    std::cout << "Global hop-2 centroid (" << hop2_drones.size() << " drones): ("
              << gcx << "," << gcy << "," << gcz << ")\n";
    std::cout << "Global ideal midpoint: (" << gmidX << "," << gmidY << "," << gmidZ << ")\n";
  }

  std::cout << "\n";
  for (const auto& h : eq) {
    if (h.lost) continue;

    // Find hop-2 drones within range of this helper (using equilibrium positions).
    std::vector<const EqPos*> visible;
    for (const auto* d : hop2_drones) {
      double dx = h.pos.x - d->pos.x, dy = h.pos.y - d->pos.y, dz = h.pos.z - d->pos.z;
      if (std::sqrt(dx*dx + dy*dy + dz*dz) <= maxRangeMeters) {
        visible.push_back(d);
      }
    }

    std::string hops_str = (h.hops == 0xFF) ? "?" : std::to_string(static_cast<int>(h.hops));
    std::cout << "  helper " << static_cast<int>(h.id)
              << " (hops=" << hops_str << ")"
              << "  pos=(" << h.pos.x << "," << h.pos.y << "," << h.pos.z << ")\n";

    if (visible.empty()) {
      std::cout << "    sees: no hop-2 drones in range — did not start mission\n";
      continue;
    }

    // Per-helper centroid and midpoint.
    double lcx = 0.0, lcy = 0.0, lcz = 0.0;
    std::cout << "    sees:";
    for (const auto* v : visible) {
      std::cout << " D" << static_cast<int>(v->id)
                << "(" << v->pos.x << "," << v->pos.y << ")";
      lcx += v->pos.x; lcy += v->pos.y; lcz += v->pos.z;
    }
    double vn = static_cast<double>(visible.size());
    lcx /= vn; lcy /= vn; lcz /= vn;
    std::cout << "\n";

    double lmidX = (basePos.x + lcx) / 2.0;
    double lmidY = (basePos.y + lcy) / 2.0;
    double lmidZ = (basePos.z + lcz) / 2.0;
    double dist = std::sqrt((h.pos.x-lmidX)*(h.pos.x-lmidX)
                          + (h.pos.y-lmidY)*(h.pos.y-lmidY)
                          + (h.pos.z-lmidZ)*(h.pos.z-lmidZ));

    std::cout << "    local centroid: (" << lcx << "," << lcy << "," << lcz << ")"
              << "  local midpoint: (" << lmidX << "," << lmidY << "," << lmidZ << ")"
              << "  dist=" << dist << "m\n";
  }

  std::cout << "================================================\n" << std::endl;

  Simulator::Destroy();
  return 0;
}
