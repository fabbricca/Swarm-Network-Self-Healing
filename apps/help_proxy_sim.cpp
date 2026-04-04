#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <fstream>
#include <vector>

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"

#include "platform/ns3/base_station/ns3_base_station.h"
#include "platform/ns3/drone/ns3_drone.h"
#include "platform/ns3/uwb_anchor/ns3_uwb_anchor.h"
#include "platform/ns3/uwb_channel/uwb_channel.h"

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

}  // namespace

int main(int argc, char* argv[]) {
  Time::SetResolution(Time::NS);

  // Self-healing protocol — 3-hop relay chain across 10 drones:
  //
  // Drones 1-3 (< 50m from base): in base coverage, become relay helpers (mission_active=true).
  // Drones 4-10 (>= 50m from base): outside base coverage, emit HELP_PROXY.
  //
  // Relay topology:
  //   Base → D1,D2,D3 (direct) → D4,D5 (1-hop lost) → D6,D7,D10 (2-hop lost) → D8,D9 (3-hop lost)
  //
  // Lost drones stay put and passively relay HELP_PROXY upstream via multi-hop deduped
  // broadcast relay. Only in-coverage drones (mission_active=true) reposition.

  double maxRangeMeters = 50.0;
  double simSeconds = 300.0;
  double kAtt = 1.0;
  double kRep = 8.0;
  double dSafe = 2.0;
  double vMax = 1.0;
  double droneWeightKg = 0.029;
  double uwbNoiseStdDev = 0.0;
  std::string csvOut = "";
  std::string animOut = "/output/drone-simulation.xml";


  CommandLine cmd;
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
  cmd.Parse(argc, argv);

  sim::UwbChannelConfig uwbCfg;
  uwbCfg.maxRangeMeters = maxRangeMeters;
  sim::UwbChannel::Get().Configure(uwbCfg);

  constexpr uint32_t NUM_DRONES = 10;
  constexpr uint32_t NUM_ANCHORS = 12;  // standalone UWB anchors (base is also an anchor)

  NodeContainer nodes;
  nodes.Create(1 + NUM_DRONES + NUM_ANCHORS);
  // node 0: base station (also UWB anchor)
  // node 1..10: drones
  // node 11..22: standalone UWB anchors

  // Base station
  EnsureMobility(nodes.Get(0),  Vector(  0.0,   0.0, 0.0));

  // Drones 1-3: inside base coverage (< 50m) — become relay helpers
  EnsureMobility(nodes.Get(1),  Vector( 35.0,  20.0, 0.0));
  EnsureMobility(nodes.Get(2),  Vector( 25.0, -15.0, 0.0));
  EnsureMobility(nodes.Get(3),  Vector( 40.0,  -5.0, 0.0));

  // Drones 4-10: outside base coverage — trigger HELP_PROXY, form relay chain
  EnsureMobility(nodes.Get(4),  Vector( 70.0,  25.0, 0.0));
  EnsureMobility(nodes.Get(5),  Vector( 65.0, -20.0, 0.0));
  EnsureMobility(nodes.Get(6),  Vector(105.0,  10.0, 0.0));
  EnsureMobility(nodes.Get(7),  Vector(115.0, -10.0, 0.0));
  EnsureMobility(nodes.Get(8),  Vector(140.0,  25.0, 0.0));
  EnsureMobility(nodes.Get(9),  Vector(135.0, -20.0, 0.0));
  EnsureMobility(nodes.Get(10), Vector( 85.0,   5.0, 0.0));

  // UWB anchors (nodes 11-22): 12 anchors covering the full operational area,
  // ensuring every drone can hear at least 3 anchors within 50m.
  EnsureMobility(nodes.Get(11), Vector(  0.0, -30.0, 0.0));
  EnsureMobility(nodes.Get(12), Vector(-20.0,  10.0, 0.0));
  EnsureMobility(nodes.Get(13), Vector( 40.0, -20.0, 0.0));
  EnsureMobility(nodes.Get(14), Vector( 40.0,  30.0, 0.0));
  EnsureMobility(nodes.Get(15), Vector( 65.0,   0.0, 0.0));
  EnsureMobility(nodes.Get(16), Vector( 70.0, -30.0, 0.0));
  EnsureMobility(nodes.Get(17), Vector( 80.0,  30.0, 0.0));
  EnsureMobility(nodes.Get(18), Vector(105.0, -15.0, 0.0));
  EnsureMobility(nodes.Get(19), Vector(110.0,  20.0, 0.0));
  EnsureMobility(nodes.Get(20), Vector(130.0, -25.0, 0.0));
  EnsureMobility(nodes.Get(21), Vector(140.0,  10.0, 0.0));
  EnsureMobility(nodes.Get(22), Vector(125.0,  35.0, 0.0));

  Ns3BaseStation base(0, nodes.Get(0));
  base.setPosition(0.0, 0.0, 0.0);

  // Create standalone UWB anchors.
  std::vector<std::unique_ptr<Ns3UwbAnchor>> anchors;
  anchors.reserve(NUM_ANCHORS);
  const Vector anchorPositions[] = {
    {  0.0, -30.0, 0.0},   // A1  (node 11, id 11)
    {-20.0,  10.0, 0.0},   // A2  (node 12, id 12)
    { 40.0, -20.0, 0.0},   // A3  (node 13, id 13)
    { 40.0,  30.0, 0.0},   // A4  (node 14, id 14)
    { 65.0,   0.0, 0.0},   // A5  (node 15, id 15)
    { 70.0, -30.0, 0.0},   // A6  (node 16, id 16)
    { 80.0,  30.0, 0.0},   // A7  (node 17, id 17)
    {105.0, -15.0, 0.0},   // A8  (node 18, id 18)
    {110.0,  20.0, 0.0},   // A9  (node 19, id 19)
    {130.0, -25.0, 0.0},   // A10 (node 20, id 20)
    {140.0,  10.0, 0.0},   // A11 (node 21, id 21)
    {125.0,  35.0, 0.0},   // A12 (node 22, id 22)
  };
  for (uint32_t i = 0; i < NUM_ANCHORS; ++i) {
    uint8_t anchor_id = static_cast<uint8_t>(NUM_DRONES + 1 + i);  // IDs 11..22
    anchors.push_back(std::make_unique<Ns3UwbAnchor>(anchor_id, nodes.Get(NUM_DRONES + 1 + i)));
    anchors.back()->setPosition(anchorPositions[i].x, anchorPositions[i].y, anchorPositions[i].z);
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
    drones.push_back(std::make_unique<Ns3Drone>(
      static_cast<uint8_t>(i + 1),
      nodes.Get(i + 1),
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

  // Register peers (no mission forcing here; just wiring IDs).
  for (uint32_t i = 0; i < NUM_DRONES; ++i) {
    drones[i]->setBaseStation(base.id());
    base.registerDrone(drones[i]->id());
  }

  // Base station triggers floods periodically (unicast START to initiator).
  base.start();

  // Start standalone UWB anchor beacon broadcasting.
  for (const auto& a : anchors) {
    a->start();
  }

  // Start drones' periodic ticks (POS_UPDATE/ACK tracking + idle motion + HELP_PROXY timeout).
  for (const auto& d : drones) {
    d->start();
  }

  std::cout << "[Sim] base coverage=" << maxRangeMeters
            << "m, drones=" << NUM_DRONES
            << ", uwb_anchors=" << NUM_ANCHORS
            << ", stop=" << simSeconds << "s" << std::endl;

  AnimationInterface anim(animOut);
  anim.SetBackgroundImage("whiteBackground.png", -30, -45, 200, 100, true);
  uint32_t baseStationIcon = anim.AddResource("baseStation.png");
  uint32_t droneIcon = anim.AddResource("drone.png");

  anim.UpdateNodeImage(0, baseStationIcon);
  anim.UpdateNodeSize(0, 10, 10);
  for (uint32_t i = 1; i <= NUM_DRONES; ++i) {
    anim.UpdateNodeImage(i, droneIcon);
    anim.UpdateNodeSize(i, 10, 10);
  }
  for (uint32_t i = 0; i < NUM_ANCHORS; ++i) {
    uint32_t nodeIdx = NUM_DRONES + 1 + i;
    anim.UpdateNodeSize(nodeIdx, 6, 6);
  }
  
  Simulator::Stop(Seconds(simSeconds));
  Simulator::Run();

  // ── End-of-simulation performance metrics ──
  std::cout << "\n========== END-OF-SIMULATION METRICS ==========\n";

  // Collect final positions for all drones.
  struct DroneInfo { uint8_t id; bool lost; uint8_t hops; Vector gt; std::vector<double> tri; double err; };
  std::vector<DroneInfo> info;
  double total_error = 0.0, min_error = std::numeric_limits<double>::max(), max_error = 0.0;

  for (uint32_t i = 0; i < NUM_DRONES; ++i) {
    auto mob = nodes.Get(i + 1)->GetObject<ConstantPositionMobilityModel>();
    Vector gt = mob->GetPosition();
    auto* pos = drones[i]->position();
    std::vector<double> tri = pos ? pos->getCoordinates() : std::vector<double>{0.0, 0.0, 0.0};
    double dx = gt.x - tri[0], dy = gt.y - tri[1], dz = gt.z - tri[2];
    double err = std::sqrt(dx*dx + dy*dy + dz*dz);
    min_error = std::min(min_error, err);
    max_error = std::max(max_error, err);
    total_error += err;
    bool lost = (drones[i]->id() >= 4);
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

  // ── 3. Per-helper midpoint convergence ──
  // Each helper drone only hears hop-2 drones within maxRangeMeters.  Its centroid
  // (and therefore its ideal midpoint) depends on which hop-2 drones it can see.
  auto baseMob = nodes.Get(0)->GetObject<ConstantPositionMobilityModel>();
  Vector basePos = baseMob->GetPosition();

  // Collect hop-2 drones.
  std::vector<const DroneInfo*> hop2_drones;
  for (const auto& d : info) {
    if (d.hops == 2) hop2_drones.push_back(&d);
  }

  // Global centroid (for reference).
  double gcx = 0.0, gcy = 0.0, gcz = 0.0;
  for (const auto* d : hop2_drones) { gcx += d->gt.x; gcy += d->gt.y; gcz += d->gt.z; }
  if (!hop2_drones.empty()) {
    double n = static_cast<double>(hop2_drones.size());
    gcx /= n; gcy /= n; gcz /= n;
  }
  double gmidX = (basePos.x + gcx) / 2.0;
  double gmidY = (basePos.y + gcy) / 2.0;
  double gmidZ = (basePos.z + gcz) / 2.0;

  std::cout << "\n── Per-Helper Midpoint Convergence ──\n";
  std::cout << "Base station: (" << basePos.x << "," << basePos.y << "," << basePos.z << ")\n";
  if (!hop2_drones.empty()) {
    std::cout << "Global hop-2 centroid (" << hop2_drones.size() << " drones): ("
              << gcx << "," << gcy << "," << gcz << ")\n";
    std::cout << "Global ideal midpoint: (" << gmidX << "," << gmidY << "," << gmidZ << ")\n";
  }

  std::cout << "\n";
  for (const auto& h : info) {
    if (h.lost) continue;

    // Find hop-2 drones within range of this helper.
    std::vector<const DroneInfo*> visible;
    for (const auto* d : hop2_drones) {
      double dx = h.gt.x - d->gt.x, dy = h.gt.y - d->gt.y, dz = h.gt.z - d->gt.z;
      if (std::sqrt(dx*dx + dy*dy + dz*dz) <= maxRangeMeters) {
        visible.push_back(d);
      }
    }

    std::string hops_str = (h.hops == 0xFF) ? "?" : std::to_string(static_cast<int>(h.hops));
    std::cout << "  helper " << static_cast<int>(h.id)
              << " (hops=" << hops_str << ")"
              << "  pos=(" << h.gt.x << "," << h.gt.y << "," << h.gt.z << ")\n";

    if (visible.empty()) {
      std::cout << "    sees: no hop-2 drones in range — did not start mission\n";
      continue;
    }

    // Per-helper centroid and midpoint.
    double lcx = 0.0, lcy = 0.0, lcz = 0.0;
    std::cout << "    sees:";
    for (const auto* v : visible) {
      std::cout << " D" << static_cast<int>(v->id)
                << "(" << v->gt.x << "," << v->gt.y << ")";
      lcx += v->gt.x; lcy += v->gt.y; lcz += v->gt.z;
    }
    double vn = static_cast<double>(visible.size());
    lcx /= vn; lcy /= vn; lcz /= vn;
    std::cout << "\n";

    double lmidX = (basePos.x + lcx) / 2.0;
    double lmidY = (basePos.y + lcy) / 2.0;
    double lmidZ = (basePos.z + lcz) / 2.0;
    double dist = std::sqrt((h.gt.x-lmidX)*(h.gt.x-lmidX)
                          + (h.gt.y-lmidY)*(h.gt.y-lmidY)
                          + (h.gt.z-lmidZ)*(h.gt.z-lmidZ));

    std::cout << "    local centroid: (" << lcx << "," << lcy << "," << lcz << ")"
              << "  local midpoint: (" << lmidX << "," << lmidY << "," << lmidZ << ")"
              << "  dist=" << dist << "m\n";
  }

  std::cout << "================================================\n" << std::endl;

  Simulator::Destroy();
  return 0;
}
