#!/usr/bin/env python3
"""
Biobuzz FTC 2026 Offline Ballistics Solver & LUT Generator
Ported from AlphaBots 2026-IndianaJoe FRC Ballistics (Ballistics.java & BallisticsTable.java)
Adapted for Biobuzz 5960 FTC (Pollen balls, SWYFT servo hood, GoBILDA flywheel).

========================================================================================
HOW TO USE:
  1. Tune any constants in the CONFIG section below (or override via CLI flags).
  2. Run the script:
       python ballistics_solver.py
     Or with custom distances:
       python ballistics_solver.py --distances 6 24 36 48 60 72 84 96
     Or generate a dense sweep table:
       python ballistics_solver.py --sweep 12 108 6
     Or use fixed hood angles:
       python ballistics_solver.py --mode fixed_hood
  3. Copy and paste the generated Java array declarations directly into CommandFactory.java:
       public static double[] LUT_DISTANCES_IN = { ... };
       public static double[] LUT_RPMS = { ... };
       public static double[] LUT_HOOD_DEGS = { ... };
========================================================================================

COORDINATE & HARDWARE CONVENTIONS:
  * Distances: inches (converted internally to meters for physics).
  * Hood angles (Biobuzz convention):
      0 deg  = vertical up
      90 deg = horizontal forward
      Launch elevation angle above horizontal = (90.0 - hood_deg)
      Closer distance = steeper trajectory (lower hood_deg value)
      Farther distance = flatter trajectory (higher hood_deg value)
  * Flywheel: goBILDA 5203 series motor, max observed RPM ~4100.
"""

import argparse
import math
import sys

# ======================================================================================
# CONFIGURATION BLOCK - Tune these parameters to match your robot and game pieces
# ======================================================================================

# ---- Target and Shooter Field Geometry (inches) ----
SHOOTER_HEIGHT_IN = 16.75          # Shooter exit / pivot height above floor (inches)
TARGET_HEIGHT_IN = 48.0           # Target hive rim / basket center height above floor (inches)
TARGET_RADIUS_IN = 8.0            # Approximate opening radius of the hive (inches)

# ---- Shooter Hardware Constraints ----
WHEEL_DIAMETER_IN = 3.0           # Flywheel outer diameter (inches)
MIN_FLYWHEEL_RPM = 1200.0         # Minimum search RPM (spool idle floor)
MAX_FLYWHEEL_RPM = 4100.0         # Max observed motor RPM (hard hardware ceiling)

# ---- Hood Kinematic Limits (degrees, Biobuzz convention: 0=up, 90=horiz) ----
# Calibrated stops from Hood.java
MIN_HOOD_DEG = 16.831             # Physical DOWN stop (steepest: 90 - 16.831 = 73.169 deg elevation)
MAX_HOOD_DEG = 46.852             # Physical UP stop (flattest: 90 - 46.852 = 43.148 deg elevation)

# ---- Pollen Ball Physical Properties (FTC 2026 game piece) ----
BALL_DIAMETER_IN = 3.0            # Foam pollen ball diameter (inches)
BALL_MASS_LBS = 0.077             # Ball mass (approx 35 grams = 0.077 lbs = 0.035 kg)

# ---- Aerodynamics & Surface Physics (IndianaJoe model) ----
AIR_DENSITY_KG_M3 = 1.225         # Standard air density at sea level (kg/m^3)
DRAG_COEFFICIENT = 0.45           # Sphere drag coefficient Cd (~0.40 - 0.50)
MAGNUS_LIFT_COEFFICIENT = 0.12    # Magnus backspin lift coefficient Cl (~0.08 - 0.15)
SPIN_PER_WHEEL_SURFACE = 0.75     # Backspin fraction relative to wheel surface rad/s

# ---- Flywheel Transfer & Slip Model ----
# exit velocity = wheel_surface_speed * (EXIT_SPEED_PER_WHEEL_SURFACE - slip_at_high_rpm)
EXIT_SPEED_PER_WHEEL_SURFACE = 0.35  # Nominal exit speed ratio at moderate RPM
HIGH_SPEED_EFFICIENCY_DROP = 0.04    # Efficiency reduction at max RPM due to slip / compression

# ---- Desired Trajectory Shape (Downward plunge into the hive) ----
# Impact angle in degrees relative to horizontal (negative = downward)
# Close shots can plunge steeply (-50 deg), while long shots arrive at a shallower arc (-25 deg)
DESIRED_IMPACT_ANGLE_CLOSE_DEG = -48.0
DESIRED_IMPACT_ANGLE_FAR_DEG = -22.0

# ---- Default LUT Breakpoints (distances in inches) ----
# Matches the breakpoints in CommandFactory.java
DEFAULT_DISTANCES_IN = [6.0, 36.0, 48.0, 60.0, 72.0, 96.0]

# Predefined / baseline hood angles for FIXED_HOOD mode or close-range clamping:
DEFAULT_BASELINE_HOOD_DEGS = [16.831, 20.0, 26.0, 34.0, 42.0, 46.852]

# Close-range clamp: below this distance (inches), direct line-of-sight is blocked by the hive
# or requires launch elevation steeper than MIN_HOOD_DEG. Clamp to baseline values.
CLOSE_RANGE_CLAMP_DIST_IN = 18.0
CLOSE_RANGE_BASE_RPM = 2500.0


# ======================================================================================
# BALLISTICS SOLVER IMPLEMENTATION
# ======================================================================================

class BiobuzzBallisticsSolver:
    def __init__(self,
                 shooter_height_in=SHOOTER_HEIGHT_IN,
                 target_height_in=TARGET_HEIGHT_IN,
                 wheel_diameter_in=WHEEL_DIAMETER_IN,
                 min_rpm=MIN_FLYWHEEL_RPM,
                 max_rpm=MAX_FLYWHEEL_RPM,
                 min_hood_deg=MIN_HOOD_DEG,
                 max_hood_deg=MAX_HOOD_DEG,
                 ball_diameter_in=BALL_DIAMETER_IN,
                 ball_mass_lbs=BALL_MASS_LBS,
                 air_density=AIR_DENSITY_KG_M3,
                 drag_cd=DRAG_COEFFICIENT,
                 magnus_cl=MAGNUS_LIFT_COEFFICIENT,
                 spin_ratio=SPIN_PER_WHEEL_SURFACE,
                 exit_ratio=EXIT_SPEED_PER_WHEEL_SURFACE,
                 eff_drop=HIGH_SPEED_EFFICIENCY_DROP,
                 impact_close=DESIRED_IMPACT_ANGLE_CLOSE_DEG,
                 impact_far=DESIRED_IMPACT_ANGLE_FAR_DEG):

        # Convert to SI units
        self.z_shooter_m = shooter_height_in * 0.0254
        self.z_target_m = target_height_in * 0.0254
        self.wheel_radius_m = (wheel_diameter_in / 2.0) * 0.0254
        self.min_rpm = float(min_rpm)
        self.max_rpm = float(max_rpm)
        self.min_hood_deg = float(min_hood_deg)
        self.max_hood_deg = float(max_hood_deg)

        self.ball_diameter_m = ball_diameter_in * 0.0254
        self.ball_mass_kg = ball_mass_lbs * 0.45359237
        self.cross_sectional_area = math.pi * (self.ball_diameter_m / 2.0) ** 2

        self.air_density = air_density
        self.drag_cd = drag_cd
        self.magnus_cl = magnus_cl
        self.spin_ratio = spin_ratio
        self.exit_ratio = exit_ratio
        self.eff_drop = eff_drop

        self.impact_close = impact_close
        self.impact_far = impact_far

    def rpm_to_exit_velocity(self, rpm: float) -> float:
        """Calculate ball exit velocity (m/s) from flywheel motor RPM."""
        omega_wheel = rpm * 2.0 * math.pi / 60.0
        surface_speed = omega_wheel * self.wheel_radius_m
        t = min(1.0, max(0.0, (rpm - 2000.0) / 2000.0))
        effective_ratio = self.exit_ratio - self.eff_drop * t
        return surface_speed * effective_ratio

    def exit_velocity_to_rpm(self, v_exit: float) -> float:
        """Invert exit velocity (m/s) back to motor RPM."""
        lo = self.min_rpm
        hi = self.max_rpm + 500.0
        for _ in range(25):
            mid = 0.5 * (lo + hi)
            if self.rpm_to_exit_velocity(mid) < v_exit:
                lo = mid
            else:
                hi = mid
        return lo

    def simulate(self, hood_deg: float, exit_vel_mps: float, target_x_m: float, dt: float = 0.001):
        """
        Simulate 2D trajectory with gravity, quadratic drag, and Magnus lift.
        Returns: (valid, height_error_m, impact_angle_deg, time_of_flight_s, apex_z_m)
        """
        # Biobuzz coordinate transform: 0 deg = vertical, 90 deg = horizontal
        launch_elevation_deg = 90.0 - hood_deg
        theta = math.radians(launch_elevation_deg)
        g = 9.80665

        x = 0.0
        z = self.z_shooter_m
        vx = exit_vel_mps * math.cos(theta)
        vz = exit_vel_mps * math.sin(theta)

        # Estimate backspin from wheel surface speed
        omega_wheel = (exit_vel_mps / self.exit_ratio) / self.wheel_radius_m
        spin_rad_s = omega_wheel * self.spin_ratio

        t = 0.0
        last_x, last_z, last_vx, last_vz, last_t = x, z, vx, vz, t
        z_max = z

        max_steps = int(3.0 / dt)
        for _ in range(max_steps):
            if x >= target_x_m or z < -0.2:
                break

            last_x, last_z, last_vx, last_vz, last_t = x, z, vx, vz, t

            v = math.hypot(vx, vz)
            if v < 1e-5:
                return False, 0.0, 0.0, 0.0, z_max

            ux = vx / v
            uz = vz / v

            # Aerodynamic drag opposing motion
            drag_force = 0.5 * self.air_density * self.drag_cd * self.cross_sectional_area * v * v
            drag_accel = drag_force / self.ball_mass_kg
            ax_drag = -drag_accel * ux
            az_drag = -drag_accel * uz

            # Magnus effect backspin lift
            magnus_force = 0.5 * self.air_density * self.magnus_cl * self.cross_sectional_area * v * v
            spin_scale = min(3.0, max(0.0, spin_rad_s / 300.0))
            magnus_accel = (magnus_force * spin_scale) / self.ball_mass_kg
            ax_mag = -uz * magnus_accel
            az_mag = ux * magnus_accel

            # Total acceleration
            ax = ax_drag + ax_mag
            az = -g + az_drag + az_mag

            # Numerical integration
            vx += ax * dt
            vz += az * dt
            x += vx * dt
            z += vz * dt
            t += dt

            if z > z_max:
                z_max = z

        if x < target_x_m:
            return False, (z - self.z_target_m), 0.0, t, z_max

        # Linear interpolation at exact x distance
        dx = x - last_x
        frac = (target_x_m - last_x) / (dx + 1e-9)
        frac = min(1.0, max(0.0, frac))

        z_at_target = last_z + frac * (z - last_z)
        vx_at_target = last_vx + frac * (vx - last_vx)
        vz_at_target = last_vz + frac * (vz - last_vz)
        t_at_target = last_t + frac * (t - last_t)

        impact_angle_deg = math.degrees(math.atan2(vz_at_target, vx_at_target))
        height_error_m = z_at_target - self.z_target_m

        return True, height_error_m, impact_angle_deg, t_at_target, z_max

    def solve_rpm_for_hood(self, hood_deg: float, target_x_m: float):
        """Binary search for the required RPM at a specific hood angle to hit target height."""
        v_max = self.rpm_to_exit_velocity(self.max_rpm)
        ok_max, h_err_max, _, _, _ = self.simulate(hood_deg, v_max, target_x_m)
        if not ok_max or h_err_max < 0.0:
            return False, 0.0, 0.0, 0.0, 0.0, 0.0

        lo_rpm = self.min_rpm
        hi_rpm = self.max_rpm
        best_match = None

        for _ in range(25):
            mid_rpm = 0.5 * (lo_rpm + hi_rpm)
            v_mid = self.rpm_to_exit_velocity(mid_rpm)
            ok, h_err, imp_deg, tof, z_max = self.simulate(hood_deg, v_mid, target_x_m)

            if not ok:
                lo_rpm = mid_rpm
                continue

            best_match = (mid_rpm, v_mid, imp_deg, tof, z_max, h_err)

            if abs(h_err) < 0.005:  # within 5mm
                break

            if h_err > 0.0:
                hi_rpm = mid_rpm
            else:
                lo_rpm = mid_rpm

        if best_match and abs(best_match[5]) < 0.04:
            rpm, v_exit, imp, tof, z_max, _ = best_match
            if rpm <= self.max_rpm + 1e-3:
                return True, rpm, v_exit, imp, tof, z_max

        return False, 0.0, 0.0, 0.0, 0.0, 0.0

    def target_impact_angle_for_distance(self, distance_in: float) -> float:
        """Linear interpolation of desired impact angle with distance."""
        t = min(1.0, max(0.0, (distance_in - 36.0) / 60.0))
        return self.impact_close + t * (self.impact_far - self.impact_close)

    def solve_auto_optimal(self, distance_in: float, hood_step_deg: float = 0.5):
        """
        Search hood angle space to find optimal (hood, rpm) pair meeting downward plunge criteria.
        """
        if distance_in < CLOSE_RANGE_CLAMP_DIST_IN:
            return {
                'distance_in': distance_in,
                'hood_deg': self.min_hood_deg,
                'launch_elevation_deg': 90.0 - self.min_hood_deg,
                'rpm': CLOSE_RANGE_BASE_RPM,
                'exit_vel_mps': self.rpm_to_exit_velocity(CLOSE_RANGE_BASE_RPM),
                'impact_deg': -60.0,
                'tof_s': 0.35,
                'apex_in': 50.0,
                'status': 'CLAMPED_CLOSE'
            }

        target_x_m = distance_in * 0.0254
        desired_impact = self.target_impact_angle_for_distance(distance_in)

        best_sol = None
        best_score = float('inf')

        n_steps = int(round((self.max_hood_deg - self.min_hood_deg) / hood_step_deg))
        for i in range(n_steps + 1):
            hood = self.min_hood_deg + i * hood_step_deg
            valid, rpm, v_exit, imp_deg, tof, z_max = self.solve_rpm_for_hood(hood, target_x_m)
            if not valid:
                continue

            # Trajectory scoring:
            # 1. Penalize upward impact angle (ball must plunge into hive)
            upward_penalty = 200.0 if imp_deg >= 0 else 0.0
            # 2. Closeness to desired downward impact angle
            impact_penalty = abs(imp_deg - desired_impact)
            # 3. RPM preference
            rpm_penalty = (rpm / self.max_rpm) * 5.0

            score = upward_penalty + impact_penalty * 2.0 + rpm_penalty

            if score < best_score:
                best_score = score
                best_sol = {
                    'distance_in': distance_in,
                    'hood_deg': hood,
                    'launch_elevation_deg': 90.0 - hood,
                    'rpm': rpm,
                    'exit_vel_mps': v_exit,
                    'impact_deg': imp_deg,
                    'tof_s': tof,
                    'apex_in': z_max * 39.3701,
                    'status': 'OPTIMAL'
                }

        if best_sol is None:
            return {
                'distance_in': distance_in,
                'hood_deg': self.max_hood_deg,
                'launch_elevation_deg': 90.0 - self.max_hood_deg,
                'rpm': self.max_rpm,
                'exit_vel_mps': self.rpm_to_exit_velocity(self.max_rpm),
                'impact_deg': 0.0,
                'tof_s': 0.0,
                'apex_in': 0.0,
                'status': 'MAX_RPM_REACHED'
            }

        return best_sol

    def solve_fixed_hood(self, distance_in: float, hood_deg: float):
        """Solve exact RPM for an explicitly specified hood angle."""
        if distance_in < CLOSE_RANGE_CLAMP_DIST_IN:
            return {
                'distance_in': distance_in,
                'hood_deg': hood_deg,
                'launch_elevation_deg': 90.0 - hood_deg,
                'rpm': CLOSE_RANGE_BASE_RPM,
                'exit_vel_mps': self.rpm_to_exit_velocity(CLOSE_RANGE_BASE_RPM),
                'impact_deg': -60.0,
                'tof_s': 0.35,
                'apex_in': 50.0,
                'status': 'CLAMPED_CLOSE'
            }

        target_x_m = distance_in * 0.0254
        valid, rpm, v_exit, imp_deg, tof, z_max = self.solve_rpm_for_hood(hood_deg, target_x_m)
        if valid:
            return {
                'distance_in': distance_in,
                'hood_deg': hood_deg,
                'launch_elevation_deg': 90.0 - hood_deg,
                'rpm': rpm,
                'exit_vel_mps': v_exit,
                'impact_deg': imp_deg,
                'tof_s': tof,
                'apex_in': z_max * 39.3701,
                'status': 'VALID'
            }
        else:
            return {
                'distance_in': distance_in,
                'hood_deg': hood_deg,
                'launch_elevation_deg': 90.0 - hood_deg,
                'rpm': self.max_rpm,
                'exit_vel_mps': self.rpm_to_exit_velocity(self.max_rpm),
                'impact_deg': 0.0,
                'tof_s': 0.0,
                'apex_in': 0.0,
                'status': 'OUT_OF_RANGE'
            }


# ======================================================================================
# FORMATTING & CLI
# ======================================================================================

def format_java_code(results, max_rpm):
    dist_strs = [f"{r['distance_in']:.1f}".rstrip('0').rstrip('.') for r in results]
    rpm_strs = [f"{int(round(r['rpm'] / 10.0) * 10)}" for r in results]
    hood_strs = [f"{r['hood_deg']:.3f}" for r in results]

    lines = []
    lines.append("// ==========================================================================")
    lines.append("// BIOBUZZ 5960 BALLISTICS LOOKUP TABLE (GENERATED BY ballistics_solver.py)")
    lines.append("// Copy & paste these 3 lines directly into CommandFactory.java")
    lines.append("// ==========================================================================")
    lines.append(f"public static double[] LUT_DISTANCES_IN = {{{', '.join(dist_strs)}}};")
    lines.append(f"public static double[] LUT_RPMS = {{{', '.join(rpm_strs)}}}; // max observed RPM is {int(max_rpm)}")
    lines.append(f"public static double[] LUT_HOOD_DEGS = {{{', '.join(hood_strs)}}};")
    return "\n".join(lines)


def print_table(results):
    print("=" * 102)
    print(f"{'Dist (in)':>9} | {'Dist (m)':>8} | {'Hood Deg':>8} | {'Launch Elev':>11} | {'RPM':>6} | {'Exit m/s':>8} | {'Exit fps':>8} | {'TOF (s)':>7} | {'Apex (in)':>9} | {'Impact':>7} | {'Status':>12}")
    print("-" * 102)
    for r in results:
        dist_m = r['distance_in'] * 0.0254
        v_fps = r['exit_vel_mps'] * 3.28084
        print(f"{r['distance_in']:9.1f} | {dist_m:8.2f} | {r['hood_deg']:8.3f} | {r['launch_elevation_deg']:11.2f}° | {int(round(r['rpm'])):6d} | {r['exit_vel_mps']:8.2f} | {v_fps:8.2f} | {r['tof_s']:7.3f} | {r['apex_in']:9.1f} | {r['impact_deg']:+6.1f}° | {r['status']:>12}")
    print("=" * 102)


def main():
    parser = argparse.ArgumentParser(
        description="Biobuzz FTC 2026 Offline Ballistics Solver (IndianaJoe FRC Ballistics port)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    # Range / Sweep options
    parser.add_argument("--distances", nargs="+", type=float, default=None,
                        help="List of distances (inches) to solve for.")
    parser.add_argument("--sweep", nargs=3, type=float, metavar=("START", "END", "STEP"), default=None,
                        help="Generate a dense table sweep from START to END by STEP (inches).")
    parser.add_argument("--mode", choices=["auto", "fixed_hood"], default="auto",
                        help="'auto' optimizes hood & RPM (IndianaJoe style). 'fixed_hood' solves RPM for default hood angles.")

    # Hardware & Geometry overrides
    parser.add_argument("--shooter-height", type=float, default=SHOOTER_HEIGHT_IN, help="Shooter exit height (inches)")
    parser.add_argument("--target-height", type=float, default=TARGET_HEIGHT_IN, help="Target hive rim height (inches)")
    parser.add_argument("--wheel-diam", type=float, default=WHEEL_DIAMETER_IN, help="Flywheel wheel diameter (inches)")
    parser.add_argument("--min-rpm", type=float, default=MIN_FLYWHEEL_RPM, help="Minimum search RPM")
    parser.add_argument("--max-rpm", type=float, default=MAX_FLYWHEEL_RPM, help="Maximum flywheel RPM limit")
    parser.add_argument("--min-hood", type=float, default=MIN_HOOD_DEG, help="Minimum hood angle (deg, physical down)")
    parser.add_argument("--max-hood", type=float, default=MAX_HOOD_DEG, help="Maximum hood angle (deg, physical up)")

    # Ball & Aerodynamics overrides
    parser.add_argument("--ball-diam", type=float, default=BALL_DIAMETER_IN, help="Ball diameter (inches)")
    parser.add_argument("--ball-mass", type=float, default=BALL_MASS_LBS, help="Ball mass (lbs)")
    parser.add_argument("--air-density", type=float, default=AIR_DENSITY_KG_M3, help="Air density (kg/m^3)")
    parser.add_argument("--drag-cd", type=float, default=DRAG_COEFFICIENT, help="Drag coefficient Cd")
    parser.add_argument("--magnus-cl", type=float, default=MAGNUS_LIFT_COEFFICIENT, help="Magnus lift coefficient Cl")
    parser.add_argument("--exit-ratio", type=float, default=EXIT_SPEED_PER_WHEEL_SURFACE, help="Exit speed per wheel surface speed")
    parser.add_argument("--eff-drop", type=float, default=HIGH_SPEED_EFFICIENCY_DROP, help="High speed efficiency drop")
    parser.add_argument("--impact-close", type=float, default=DESIRED_IMPACT_ANGLE_CLOSE_DEG, help="Target close impact angle (deg, negative=down)")
    parser.add_argument("--impact-far", type=float, default=DESIRED_IMPACT_ANGLE_FAR_DEG, help="Target far impact angle (deg, negative=down)")

    args = parser.parse_args()

    solver = BiobuzzBallisticsSolver(
        shooter_height_in=args.shooter_height,
        target_height_in=args.target_height,
        wheel_diameter_in=args.wheel_diam,
        min_rpm=args.min_rpm,
        max_rpm=args.max_rpm,
        min_hood_deg=args.min_hood,
        max_hood_deg=args.max_hood,
        ball_diameter_in=args.ball_diam,
        ball_mass_lbs=args.ball_mass,
        air_density=args.air_density,
        drag_cd=args.drag_cd,
        magnus_cl=args.magnus_cl,
        exit_ratio=args.exit_ratio,
        eff_drop=args.eff_drop,
        impact_close=args.impact_close,
        impact_far=args.impact_far
    )

    if args.sweep:
        start_d, end_d, step_d = args.sweep
        distances = []
        d = start_d
        while d <= end_d + 1e-6:
            distances.append(round(d, 2))
            d += step_d
    elif args.distances:
        distances = args.distances
    else:
        distances = DEFAULT_DISTANCES_IN

    results = []
    if args.mode == "fixed_hood" and len(distances) == len(DEFAULT_BASELINE_HOOD_DEGS):
        for d, h in zip(distances, DEFAULT_BASELINE_HOOD_DEGS):
            results.append(solver.solve_fixed_hood(d, h))
    else:
        for d in distances:
            results.append(solver.solve_auto_optimal(d))

    print("\n--- BIOBUZZ BALLISTICS SIMULATION RESULTS ---")
    print_table(results)
    print()
    print(format_java_code(results, args.max_rpm))
    print()


if __name__ == "__main__":
    main()
