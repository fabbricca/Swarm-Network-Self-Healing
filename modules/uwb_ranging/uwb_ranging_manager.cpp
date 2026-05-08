#include "modules/uwb_ranging/uwb_ranging_manager.h"

#include <cmath>
#include <cstring>
#include <random>

UwbRangingManager::UwbRangingManager(std::function<double()> get_time_s,
                                     double speed_of_light_mps,
                                     double noise_std_dev_m)
    : m_get_time_s(std::move(get_time_s)),
      m_speed_of_light_mps(speed_of_light_mps),
      m_noise_std_dev_m(noise_std_dev_m) {}

void UwbRangingManager::onPacketReceived(const ::Packet& pkt) {
  if (pkt.type != ::PacketType::UWB_BEACON) {
    return;
  }
  if (pkt.payload.size() < sizeof(UwbBeaconMsg)) {
    return;
  }

  UwbBeaconMsg beacon;
  std::memcpy(&beacon, pkt.payload.data(), sizeof(beacon));

  // Compute range from ToF.
  // The UwbChannel schedules delivery at tx_time + distance/c (no MAC).
  // rx_time - tx_time = propagation_delay, so range = ToF * c.
  double rx_time_s = m_get_time_s ? m_get_time_s() : 0.0;

  double tof_s = rx_time_s - beacon.tx_timestamp_s;
  if (tof_s < 0.0) {
    return;  // invalid measurement
  }

  double range_m = tof_s * m_speed_of_light_mps;

  if (m_noise_std_dev_m > 0.0) {
    // We use a local thread_local static generator to be deterministic yet distinct 
    // enough per run if needed, but for identical simple runs we can just seed it.
    // Given ns-3 is single-threaded mostly, static is fine.
    static std::mt19937 gen(12345);
    std::normal_distribution<double> noise_dist(0.0, m_noise_std_dev_m);
    range_m += noise_dist(gen);
  }

  AnchorMeasurement meas;
  meas.anchor_id = beacon.anchor_id;
  meas.range_m = range_m;
  meas.anchor_x = beacon.x;
  meas.anchor_y = beacon.y;
  meas.anchor_z = beacon.z;
  meas.last_seen_s = rx_time_s;

  m_measurements[beacon.anchor_id] = meas;

  // Drop anchors we haven't heard from recently (drone moved out of range).
  expireStale();

  // Re-trilaterate whenever we have enough anchors.
  if (m_measurements.size() >= 3) {
    trilaterate();
  }
}

std::vector<double> UwbRangingManager::getEstimatedPosition() const {
  if (!m_has_position) {
    return {};
  }
  return {m_est_x, m_est_y, m_est_z};
}

void UwbRangingManager::seedEstimatedPosition(double x, double y, double z) {
  m_est_x = x;
  m_est_y = y;
  m_est_z = z;
  m_has_position = true;
}

// Linearized least-squares trilateration.
//
// Given N anchors at known positions (xi, yi, zi) with measured ranges ri,
// the sphere equations are:
//   (x - xi)^2 + (y - yi)^2 + (z - zi)^2 = ri^2
//
// Subtracting the first anchor's equation from each subsequent one
// linearizes the system:
//   2(x1-xi)*x + 2(y1-yi)*y + 2(z1-zi)*z = r_i^2 - r_1^2 - xi^2 + x1^2 - yi^2 + y1^2 - zi^2 + z1^2
//
// This gives (N-1) linear equations in 3 unknowns, solvable via least-squares.
// For N=3 (2 equations, 3 unknowns in 3D) we assume z=0 (planar operation).
// For N>=4 we solve the full 3D system.
bool UwbRangingManager::trilaterate() {
  // Collect measurements into a vector for indexed access.
  std::vector<AnchorMeasurement> anchors;
  anchors.reserve(m_measurements.size());
  for (const auto& [id, meas] : m_measurements) {
    anchors.push_back(meas);
  }

  const size_t n = anchors.size();
  if (n < 3) {
    return false;
  }

  // Reference anchor (first one).
  const auto& a0 = anchors[0];
  const double x1 = a0.anchor_x;
  const double y1 = a0.anchor_y;
  const double z1 = a0.anchor_z;
  const double r1_sq = a0.range_m * a0.range_m;
  const double k1 = x1 * x1 + y1 * y1 + z1 * z1;

  // Decide between 2D and 3D solve based on anchor z diversity.
  //
  // The old heuristic (any anchor_z differing by >1e-6 from the first) is a
  // numerical-singularity test, not a GDOP test.  A handful of ground-level
  // anchors with z ∈ [2, 5] m is geometrically coplanar from a drone flying
  // at ~20 m altitude: range errors amplify 10× in z and the solver returns
  // garbage.  That garbage then feeds the controller, which drives drones up
  // or down chasing phantom altitudes until they leave anchor coverage and
  // the trilat freezes at a nonsensical value.
  //
  // Require meaningful vertical diversity before attempting 3D.  Below this
  // threshold we fall back to 2D and preserve the previous m_est_z so the
  // controller sees a stable altitude instead of an oscillating one.
  double z_min = z1;
  double z_max = z1;
  for (size_t i = 1; i < n; ++i) {
    if (anchors[i].anchor_z < z_min) z_min = anchors[i].anchor_z;
    if (anchors[i].anchor_z > z_max) z_max = anchors[i].anchor_z;
  }
  const double z_spread = z_max - z_min;
  const bool solve_3d = (z_spread >= Z_SPREAD_MIN_FOR_3D_M) && (n >= 4);
  const size_t cols = solve_3d ? 3 : 2;
  const size_t rows = n - 1;

  // Build A matrix and b vector.
  // A is (rows x cols), b is (rows x 1).
  std::vector<double> A(rows * cols, 0.0);
  std::vector<double> b(rows, 0.0);

  for (size_t i = 0; i < rows; ++i) {
    const auto& ai = anchors[i + 1];
    double xi = ai.anchor_x;
    double yi = ai.anchor_y;
    double zi = ai.anchor_z;
    double ri_sq = ai.range_m * ai.range_m;
    double ki = xi * xi + yi * yi + zi * zi;

    A[i * cols + 0] = 2.0 * (x1 - xi);
    A[i * cols + 1] = 2.0 * (y1 - yi);
    if (solve_3d) {
      A[i * cols + 2] = 2.0 * (z1 - zi);
    }

    b[i] = ri_sq - r1_sq - ki + k1;
  }

  // Solve via normal equations: (A^T A) x = A^T b.
  // A^T A is (cols x cols), A^T b is (cols x 1).
  std::vector<double> ATA(cols * cols, 0.0);
  std::vector<double> ATb(cols, 0.0);

  for (size_t i = 0; i < cols; ++i) {
    for (size_t j = 0; j < cols; ++j) {
      double sum = 0.0;
      for (size_t k = 0; k < rows; ++k) {
        sum += A[k * cols + i] * A[k * cols + j];
      }
      ATA[i * cols + j] = sum;
    }
    double sum = 0.0;
    for (size_t k = 0; k < rows; ++k) {
      sum += A[k * cols + i] * b[k];
    }
    ATb[i] = sum;
  }

  // Solve the small (2x2 or 3x3) system via Cramer's rule.
  if (cols == 2) {
    double det = ATA[0] * ATA[3] - ATA[1] * ATA[2];
    if (std::abs(det) < 1e-12) {
      return false;  // degenerate anchor placement
    }
    m_est_x = (ATb[0] * ATA[3] - ATb[1] * ATA[1]) / det;
    m_est_y = (ATA[0] * ATb[1] - ATA[2] * ATb[0]) / det;
    // Preserve the previous z.  In 2D mode we have no information about
    // altitude from ranging alone, so resetting to 0 every tick would
    // artificially snap the reported altitude down and inject a z-step into
    // any controller using distanceFromCoords().
  } else {
    // 3x3 Cramer's rule.
    auto det3 = [](double a00, double a01, double a02,
                   double a10, double a11, double a12,
                   double a20, double a21, double a22) {
      return a00 * (a11 * a22 - a12 * a21)
           - a01 * (a10 * a22 - a12 * a20)
           + a02 * (a10 * a21 - a11 * a20);
    };

    double det = det3(
      ATA[0], ATA[1], ATA[2],
      ATA[3], ATA[4], ATA[5],
      ATA[6], ATA[7], ATA[8]);

    if (std::abs(det) < 1e-12) {
      return false;
    }

    m_est_x = det3(
      ATb[0], ATA[1], ATA[2],
      ATb[1], ATA[4], ATA[5],
      ATb[2], ATA[7], ATA[8]) / det;

    m_est_y = det3(
      ATA[0], ATb[0], ATA[2],
      ATA[3], ATb[1], ATA[5],
      ATA[6], ATb[2], ATA[8]) / det;

    m_est_z = det3(
      ATA[0], ATA[1], ATb[0],
      ATA[3], ATA[4], ATb[1],
      ATA[6], ATA[7], ATb[2]) / det;
  }

  m_has_position = true;
  return true;
}

void UwbRangingManager::expireStale() {
  double now_s = m_get_time_s ? m_get_time_s() : 0.0;
  for (auto it = m_measurements.begin(); it != m_measurements.end(); ) {
    if ((now_s - it->second.last_seen_s) > m_stale_threshold_s) {
      it = m_measurements.erase(it);
    } else {
      ++it;
    }
  }
}
