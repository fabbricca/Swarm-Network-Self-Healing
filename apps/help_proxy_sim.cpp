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
  EnsureMobility(nodes.Get(5),  Vector( 75.0, -20.0, 0.0));
  EnsureMobility(nodes.Get(6),  Vector(105.0,  30.0, 0.0));
  EnsureMobility(nodes.Get(7),  Vector(115.0, -10.0, 0.0));
  EnsureMobility(nodes.Get(8),  Vector(140.0,  25.0, 0.0));
  EnsureMobility(nodes.Get(9),  Vector(135.0, -20.0, 0.0));
  EnsureMobility(nodes.Get(10), Vector( 90.0,   5.0, 0.0));

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
  // between the base station and the deepest-chain lost drone (Drone 8, node 8).
  auto baseMob = nodes.Get(0)->GetObject<ConstantPositionMobilityModel>();
  Vector basePos = baseMob->GetPosition();
  auto drone8Mob = nodes.Get(8)->GetObject<ConstantPositionMobilityModel>();
  Vector drone8Pos = drone8Mob->GetPosition();

  double midX = (basePos.x + drone8Pos.x) / 2.0;
  double midY = (basePos.y + drone8Pos.y) / 2.0;
  double midZ = (basePos.z + drone8Pos.z) / 2.0;

  std::cout << "\nBase station pos=(" << basePos.x << "," << basePos.y << "," << basePos.z << ")"
            << "\nDeepest lost drone (8) pos=(" << drone8Pos.x << "," << drone8Pos.y << "," << drone8Pos.z << ")"
            << "\nIdeal midpoint=(" << midX << "," << midY << "," << midZ << ")\n";

  // Distance of each in-coverage helper drone (IDs 1-3) from the midpoint.
  for (uint32_t i = 0; i < NUM_DRONES; ++i) {
    uint8_t drone_id = drones[i]->id();
    if (drone_id >= 4) continue;  // skip lost drones (IDs 4..10)

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
