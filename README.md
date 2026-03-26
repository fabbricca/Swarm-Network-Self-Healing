# Autonomous Network Self-Healing with Drones

A self-healing protocol for drone swarms that autonomously detects loss of connectivity with a base station and reorganizes to form multi-hop relay chains, restoring network integrity without centralized control.

## Overview

This project implements a decentralized self-healing protocol where drones in a swarm can detect communication failures and cooperatively reposition themselves to maintain connectivity with a base station. The system uses:

- **UWB (Ultra-Wideband) PHY-layer communication** for all node-to-node data exchange
- **UWB ToF (Time-of-Flight) trilateration** for GPS-free drone localization
- **ACK-based reachability detection** to identify connectivity loss
- **Flooding-based hop discovery** to establish distance metrics to the base station
- **Periodic neighbor broadcasts** for local situational awareness
- **Virtual spring-damper formation control** for coordinated repositioning

When a drone loses direct communication with the base station (detected via missing acknowledgments), it broadcasts a `HELP_PROXY` request. Neighboring drones then activate their formation controllers, repositioning to form a multi-hop relay chain that restores end-to-end connectivity.

## Architecture

### System Components

The architecture is designed around three node types:

| Component | Description |
|-----------|-------------|
| **Base Station** | Stationary node that tracks drone positions, sends acknowledgments, triggers periodic floods, and acts as a UWB anchor |
| **UWB Anchors** | Static infrastructure nodes that broadcast timestamped beacons for drone trilateration |
| **Drones** | Mobile agents running the same control stack with unicast (to base) and broadcast (swarm) communication, localized via UWB trilateration |

### Module Structure

```
├── apps/                          # Executable applications
│   ├── help_proxy_sim.cpp         # Main simulation scenario
│   └── controller_tuner.cpp       # Parameter tuning grid search
├── common/                        # Shared data structures
│   ├── messages.h                 # Protocol message definitions (incl. UwbBeaconMsg)
│   ├── packet.h                   # Packet envelope format
│   └── vector3D.h                 # 3D vector utilities
├── interfaces/                    # Abstract interfaces for modularity
│   ├── position.h                 # Position interface
│   ├── uwb_ranging_manager.h      # UWB ranging interface
│   └── ...
├── modules/
│   ├── communication/             # Packet packing/unpacking & transport delegation
│   ├── controller/                # Virtual spring-damper formation control
│   ├── dispatch/                  # Message routing to protocol handlers
│   ├── flood/                     # Hop discovery via flooding protocol
│   ├── neighbor/                  # Local neighbor state management
│   └── uwb_ranging/              # UWB trilateration engine & position adapter
│       ├── uwb_ranging_manager.*  # ToF-based ranging + least-squares trilateration
│       └── uwb_position.h        # PositionInterface adapter for trilaterated coords
└── platform/ns3/                  # NS-3 specific implementations
    ├── base_station/              # NS-3 base station (extends UWB anchor)
    ├── custom_mobility/           # Kinematic mobility model
    ├── drone/                     # NS-3 drone node logic
    ├── position/                  # NS-3 position interface (used by anchors)
    ├── uwb_anchor/                # UWB anchor beacon broadcaster
    ├── uwb_channel/               # PHY-level UWB channel (propagation delay model)
    ├── uwb_transport/             # Transport interface over UWB channel
    └── velocity_actuator/         # Velocity command application
```

### Communication Stack

All communication uses a single **UWB PHY-layer channel** — no WiFi, no MAC layer, no IP stack, no UDP sockets.

```
Application (drone/base logic)
        ↓
CommunicationManager (serialization)
        ↓
UwbTransport (unicast/broadcast)
        ↓
UwbChannel (PHY propagation: distance/c delay, range cutoff)
```

**DispatchManager** routes received packets to appropriate handlers:
- `FloodManager` → Hop discovery floods
- `NeighborManager` → Neighbor state updates
- `UwbRangingManager` → UWB beacon processing and trilateration
- Node Logic → Position updates, ACKs, and help requests

### Localization

Drones have **no GPS**. Position is determined entirely through UWB trilateration:

1. **UWB anchors** (static, known positions) broadcast beacons at 100 Hz containing their ID, position, and a PHY-level timestamp
2. **Drones** receive beacons and compute range from Time-of-Flight: `range = (rx_time - tx_timestamp) × speed_of_light`
3. **UwbRangingManager** trilaterates from 3+ anchor ranges using linearized least-squares (subtracts first anchor's sphere equation, solves via normal equations + Cramer's rule)
4. **UwbPosition** wraps the ranging manager as a `PositionInterface`, providing trilaterated coordinates to all consumers (controller, neighbor broadcasts, position updates)

Stale anchor measurements (>0.5s without a beacon) are automatically expired, so trilateration always reflects the current set of reachable anchors. The solver detects coplanar anchors and uses 2D mode to avoid degenerate 3D systems.

## Self-Healing Protocol

### Formation Control

The controller uses a virtual spring-damper model with two force components:

**Attractive Forces** (toward relay neighbors):
```
F_att = K_att × (p_neighbor - p_self)   for neighbors with different hop counts
```

**Repulsive Forces** (collision avoidance):
```
F_rep = K_rep × (1/dist²) × unit_vector   when dist < D_safe
```

This drives drones toward the geometric midpoint between their preceding and following neighbors in the relay chain, maximizing SNR for both links.

### Key Parameters

| Parameter | Symbol | Description | Default |
|-----------|--------|-------------|---------|
| Coverage radius | R_max | UWB communication range | 50 m |
| Attractive gain | K_att | Spring constant for attraction | 1.0 |
| Repulsive gain | K_rep | Repulsion strength | 8.0 |
| Safety distance | D_safe | Minimum separation threshold | 2.0 m |
| Max velocity | V_max | Drone speed limit | 1.0 m/s |
| Drone mass | m | Based on Crazyflie 2.1 | 29 g |

## Simulation

The simulation runs in **NS-3** (Network Simulator 3) with:
- UWB PHY-layer channel with propagation delay modeled as `distance / speed_of_light`
- Range-based cutoff (default 50m)
- No MAC layer — direct PHY delivery with nanosecond timing resolution
- Custom kinematic mobility model
- UWB anchor infrastructure for drone trilateration

### Default Scenario

- 1 base station at origin (0, 0, 0) — also acts as UWB anchor
- 3 drones at (40, 15, 0), (70, 10, 0), and (30, 25, 0)
- 5 standalone UWB anchors for trilateration coverage
- Drone 2 starts outside coverage (>50m) and triggers self-healing
- End-of-simulation metrics: trilateration error, midpoint convergence

## Building and Running

### Using Docker (Recommended)

Build the Docker image:

```bash
docker build -t swarm-sim .
```

Run the simulation:

```bash
docker run --rm swarm-sim sim --animOut=/project/output/drone-simulation.xml
```

### Simulation Parameters

Pass custom parameters via command line:

```bash
docker run --rm swarm-sim sim \
  --kAtt=1.0 --kRep=8.0 --dSafe=2.0 --vMax=1.0 \
  --maxRangeMeters=50.0 --simSeconds=60.0 \
  --animOut=/project/output/drone-simulation.xml
```

### Parameter Tuner

Run the grid search tuner to optimize controller parameters:

```bash
docker run --rm swarm-sim tuner
```

Tuner options:

```bash
docker run --rm swarm-sim tuner \
  --kAttMin=0.5 --kAttMax=2.0 --kAttStep=0.5 \
  --kRepMin=4.0 --kRepMax=12.0 --kRepStep=2.0 \
  --dSafeMin=1.0 --dSafeMax=3.0 --dSafeStep=0.5 \
  --simSeconds=150.0
```

## Visualization

To visualize the simulation, use the **NetAnim** tool included in NS-3:

1. The simulation generates `output/drone-simulation.xml`
2. Open with NetAnim following the [NS-3 animation documentation](https://www.nsnam.org/docs/release/3.46/models/html/animation.html)

## Results

With default parameters and UWB trilateration, the system achieves:

| Metric | Value |
|--------|-------|
| Trilateration error (avg) | ~13 cm |
| Trilateration error (max) | ~24 cm |
| Midpoint convergence | < 1 m |
| Connection restoration time | ~1.5 s |

## Authors

- Gianluca Bresolin (University of Padua)
- Riccardo Fabbian (University of Padua)

### Supervisor

- Federico Corò (University of Padua)

## References

This work is inspired by research on virtual spring-damper formation control and the Crazyflie 2.1 drone platform with the DWM1000 UWB Loco Positioning System. See the technical report in `report/` for detailed methodology and analysis.

## License

University of Padua - Advanced Topics in Communication Networks and Systems
