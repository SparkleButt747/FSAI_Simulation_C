#pragma once
#include <cmath>
#include <cstdio>

// Small, header-only diagnostics for body/world consistency.
namespace physdiag {

// Sideslip angle beta (rad): angle between velocity vector and body x-axis.
inline double beta(double vx, double vy) {
  // atan2 handles vx~0 cleanly; beta is signed with y-left positive.
  return std::atan2(vy, std::max(1e-9, vx));
}

// Heading of world velocity vector (rad, CCW), given world xdot, ydot.
inline double vel_heading(double xdot_world, double ydot_world) {
  return std::atan2(ydot_world, xdot_world);
}

// Wrap angle to [-pi, pi]
inline double wrap_pi(double a) {
  while (a >  M_PI) a -= 2.0*M_PI;
  while (a < -M_PI) a += 2.0*M_PI;
  return a;
}

// Compute world xdot, ydot from body vx, vy and yaw (CCW)
inline void world_vel_from_body(double yaw, double vx, double vy,
                                double& xdot, double& ydot) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  xdot = c*vx - s*vy;
  ydot = s*vx + c*vy;
}

// Soft alignment to kill spurious sideslip and yaw at low/medium speeds.
// This bleeds vy and r toward 0 when steering is small and |beta| too large,
// and also nudges yaw toward the instantaneous velocity heading if needed.
struct AlignParams {
  double v_low      = 0.3;   // [m/s] start heavy damping below this
  double v_mid      = 4.0;   // [m/s] taper off by here
  double beta_soft  = 15.0 * M_PI/180.0; // [rad] tolerate up to 15°
  double beta_hard  = 35.0 * M_PI/180.0; // [rad] clamp above this
  double steer_small= 7.0 * M_PI/180.0;  // [rad] “small steering”
  double vy_gain_lo = 12.0;  // [1/s] vy damping at very low speed
  double vy_gain_md = 4.0;   // [1/s] vy damping in mid band
  double r_gain_lo  = 8.0;   // [1/s] yaw-rate damping at very low speed
  double r_gain_md  = 3.0;   // [1/s]
  double yaw_nudge  = 3.0;   // [1/s] nudge yaw toward velocity heading
};

struct AlignState {
  // For optional logging counters if you want
  int applied_count{0};
};

// Applies alignment; returns true if any correction applied.
inline bool enforce_alignment(double dt,
                              double yaw, double steer,
                              double& vx, double& vy, double& r,
                              AlignState& st,
                              const AlignParams& P = AlignParams{}) {
  const double v = std::hypot(vx, vy);
  if (v < 1e-6) {
    // At rest: kill tiny drift to avoid “spin in place”
    vy *= std::exp(-P.vy_gain_lo * dt);
    r  *= std::exp(-P.r_gain_lo  * dt);
    st.applied_count++;
    return true;
  }

  // Compute sideslip beta and velocity heading vs yaw
  const double b = beta(vx, vy);

  // speed weight in [0,1]: 1 at very low speed, 0 above v_mid
  const double w_lo = (v <= P.v_low) ? 1.0 :
                      (v >= P.v_mid) ? 0.0 :
                      1.0 - (v - P.v_low) / std::max(1e-6, (P.v_mid - P.v_low));

  // steering small?
  const bool steer_small = std::abs(steer) <= P.steer_small;

  // If steering is small, we shouldn’t hold large |beta|: bleed vy & r.
  bool applied = false;
  if (steer_small && (std::abs(b) > P.beta_soft)) {
    // Hard clamp to avoid insane β that can arise from an early sign mistake
    const double b_clamped = std::clamp(b, -P.beta_hard, P.beta_hard);

    // Target vy consistent with allowed beta: vy_target = tan(b_allowed)*vx
    const double vy_target = std::tan(b_clamped) * std::max(0.0, vx);

    // Damping gains blend with speed
    const double g_vy = w_lo*P.vy_gain_lo + (1.0 - w_lo)*P.vy_gain_md;
    const double g_r  = w_lo*P.r_gain_lo  + (1.0 - w_lo)*P.r_gain_md;

    // Exponential toward target/calm
    const double a_vy = std::exp(-g_vy * dt);
    const double a_r  = std::exp(-g_r  * dt);

    vy = vy_target + (vy - vy_target) * a_vy;
    r  = r * a_r;

    applied = true;
  }

  // Secondary: if the *world* velocity heading disagrees strongly with yaw,
  // nudge yaw toward it a bit (prevents “pointing left, moving straight” persistence).
  double xdot, ydot;
  world_vel_from_body(yaw, vx, vy, xdot, ydot);
  const double hv = vel_heading(xdot, ydot);
  const double d_yaw = wrap_pi(hv - yaw);

  if (steer_small && w_lo > 0.15 && std::abs(d_yaw) > 10.0 * M_PI/180.0) {
    const double a_yaw = std::exp(-P.yaw_nudge * w_lo * dt);
    // Equivalent to yaw += (1 - a)*d_yaw in integrated form.
    // Implement as a small correction to r so integrator remains consistent:
    const double r_nudge = (1.0 - a_yaw) * d_yaw / std::max(1e-6, dt);
    r += r_nudge;
    applied = true;
  }

  if (applied) st.applied_count++;
  return applied;
}

// Optional one-shot printf to verify signs the first few frames:
inline void debug_print_once(int frame_idx,
                             double yaw, double vx, double vy, double r) {
  if (frame_idx < 50 && (frame_idx % 5 == 0)) {
    double xdot, ydot; world_vel_from_body(yaw, vx, vy, xdot, ydot);
    const double hv = vel_heading(xdot, ydot);
    const double b  = beta(vx, vy);
    std::printf("[dbg%02d] yaw=%.3f  v=(%.3f,%.3f)  r=%.3f  hv=%.3f  beta=%.3f  hv-yaw=%.3f\n",
                frame_idx, yaw, vx, vy, r, hv, b, hv - yaw);
  }
}

} // namespace physdiag
