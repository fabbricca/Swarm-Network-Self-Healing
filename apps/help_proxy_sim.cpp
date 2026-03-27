#include <cmath>
#include <cstdint>
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

  // Requirements:
  // - 1 base station + 3 drones
  // - base station coverage range: 50m
  // - drones move by their own behavior (idle velocity) until one leaves coverage
  // - leaving coverage -> missing POS_ACK -> drone sends HELP_PROXY
  // - HELP_PROXY triggers mission_active + repositioning behavior inside drones

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

  constexpr uint32_t NUM_DRONES = 4;
  constexpr uint32_t NUM_ANCHORS = 5;  // standalone UWB anchors (base is also an anchor)

  NodeContainer nodes;
  nodes.Create(1 + NUM_DRONES + NUM_ANCHORS);
  // node 0: base station (also UWB anchor)
  // node 1..4: drones
  // node 5..9: standalone UWB anchors

  // Place drone 2 initially outside base coverage so it will timeout and emit HELP_PROXY.
  // Keep it within range of at least one other drone (drone 3) so the HELP_PROXY can be received,
  // while keeping it far enough from the base (>= 50m) that it doesn't re-enter immediately.
  EnsureMobility(nodes.Get(0), Vector(0.0, 0.0, 0.0));
  EnsureMobility(nodes.Get(1), Vector(40.0, 15.0, 0.0));
  EnsureMobility(nodes.Get(2), Vector(70.0, 10.0, 0.0));
  EnsureMobility(nodes.Get(3), Vector(30.0, 25.0, 0.0));
  EnsureMobility(nodes.Get(4), Vector(25.0, 10.0, 0.0));

  // Standalone UWB anchors placed so that every drone can hear at least 3 anchors
  // across the full operational area (including negative-Y where helpers reposition).
  EnsureMobility(nodes.Get(5), Vector(40.0, -10.0, 0.0));
  EnsureMobility(nodes.Get(6), Vector(40.0, 30.0, 0.0));
  EnsureMobility(nodes.Get(7), Vector(70.0, 10.0, 0.0));
  EnsureMobility(nodes.Get(8), Vector(0.0, -30.0, 0.0));
  EnsureMobility(nodes.Get(9), Vector(-20.0, 10.0, 0.0));

  Ns3BaseStation base(0, nodes.Get(0));
  base.setPosition(0.0, 0.0, 0.0);

  // Create standalone UWB anchors.
  std::vector<std::unique_ptr<Ns3UwbAnchor>> anchors;
  anchors.reserve(NUM_ANCHORS);
  const Vector anchorPositions[] = {
    {40.0, -10.0, 0.0},
    {40.0,  30.0, 0.0},
    {70.0,  10.0, 0.0},
    { 0.0, -30.0, 0.0},
    {-20.0, 10.0, 0.0},
  };
  for (uint32_t i = 0; i < NUM_ANCHORS; ++i) {
    uint8_t anchor_id = static_cast<uint8_t>(NUM_DRONES + 1 + i);  // IDs 5, 6, 7, 8, 9
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
  anim.SetBackgroundImage("whiteBackground.png", -10, -10, 200, 200, true);
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

  // Ground truth from NS-3 mobility model vs trilaterated position from UWB.
  double total_error = 0.0;
  double min_error = std::numeric_limits<double>::max();
  double max_error = 0.0;

  for (uint32_t i = 0; i < NUM_DRONES; ++i) {
    auto mob = nodes.Get(i + 1)->GetObject<ConstantPositionMobilityModel>();
    Vector gt = mob->GetPosition();

    auto* pos = drones[i]->position();
    std::vector<double> tri = pos ? pos->getCoordinates() : std::vector<double>{0, 0, 0};

    double dx = gt.x - tri[0];
    double dy = gt.y - tri[1];
    double dz = gt.z - tri[2];
    double err = std::sqrt(dx * dx + dy * dy + dz * dz);

    min_error = std::min(min_error, err);
    max_error = std::max(max_error, err);
    total_error += err;

    std::cout << "[Drone " << static_cast<int>(drones[i]->id()) << "] "
              << "ground_truth=(" << gt.x << "," << gt.y << "," << gt.z << ") "
              << "trilaterated=(" << tri[0] << "," << tri[1] << "," << tri[2] << ") "
              << "error=" << err << "m" << std::endl;
  }

  double avg_error = total_error / NUM_DRONES;
  std::cout << "\nTrilateration error: min=" << min_error
            << "m  max=" << max_error
            << "m  avg=" << avg_error << "m\n";

  // Midpoint convergence: check if helper drones moved toward the midpoint
  // between the base station and the lost drone (drone 2, index 1).
  auto baseMob = nodes.Get(0)->GetObject<ConstantPositionMobilityModel>();
  Vector basePos = baseMob->GetPosition();
  auto drone2Mob = nodes.Get(2)->GetObject<ConstantPositionMobilityModel>();
  Vector drone2Pos = drone2Mob->GetPosition();

  double midX = (basePos.x + drone2Pos.x) / 2.0;
  double midY = (basePos.y + drone2Pos.y) / 2.0;
  double midZ = (basePos.z + drone2Pos.z) / 2.0;

  std::cout << "\nBase station pos=(" << basePos.x << "," << basePos.y << "," << basePos.z << ")"
            << "\nLost drone (2) pos=(" << drone2Pos.x << "," << drone2Pos.y << "," << drone2Pos.z << ")"
            << "\nIdeal midpoint=(" << midX << "," << midY << "," << midZ << ")\n";

  // Distance of each helper drone (1 and 3) from the midpoint.
  for (uint32_t i = 0; i < NUM_DRONES; ++i) {
    uint8_t drone_id = drones[i]->id();
    if (drone_id == 2) continue;  // skip the lost drone itself

    auto dMob = nodes.Get(i + 1)->GetObject<ConstantPositionMobilityModel>();
    Vector dPos = dMob->GetPosition();
    double dist = std::sqrt(
      (dPos.x - midX) * (dPos.x - midX) +
      (dPos.y - midY) * (dPos.y - midY) +
      (dPos.z - midZ) * (dPos.z - midZ));

    std::cout << "[Drone " << static_cast<int>(drone_id)
              << "] final_pos=(" << dPos.x << "," << dPos.y << "," << dPos.z << ")"
              << " dist_to_midpoint=" << dist << "m" << std::endl;
  }

  std::cout << "================================================\n" << std::endl;

  Simulator::Destroy();
  return 0;
}
