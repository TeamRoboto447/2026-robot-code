#!./venv/bin python
"""
FRC Turret Lookup Table Generator

This script helps create a lookup table for an adjustable hood turret system.
It guides you through data collection, interpolation, and JSON export.

Note: Hood angle is measured from VERTICAL plane (0° = straight up, 90° = horizontal)
Note: vpar = robot velocity parallel to target (in/s). Positive = moving toward target.
      Data collection is always STATIC (vpar=0). Velocity-adjusted entries are derived
      analytically via the iterative effective-distance method:
          d_eff = d - vpar * t_flight  (iterated 2× to refine flight time)
"""

import json
import os
import math
import concurrent.futures
from typing import Dict, List, Tuple
from scipy.interpolate import griddata
import numpy as np


class TurretLookupGenerator:
    def __init__(self, data_file='turret_data.json'):
        self.data_file = data_file
        self.data_points = []  # List of (distance, angle, rpm, target_height, vpar=0 for static)
        self.lookup_table = {}
        self.load_data()
    
    def load_data(self):
        """Load previously saved data points"""
        if os.path.exists(self.data_file):
            try:
                with open(self.data_file, 'r') as f:
                    saved = json.load(f)
                    self.data_points = saved.get('data_points', [])
                    print(f"Loaded {len(self.data_points)} existing data points")
            except Exception as e:
                print(f"Error loading data: {e}")
    
    def save_data(self):
        """Save current data points"""
        try:
            with open(self.data_file, 'w') as f:
                json.dump({'data_points': self.data_points}, f, indent=2)
            print(f"Saved {len(self.data_points)} data points to {self.data_file}")
        except Exception as e:
            print(f"Error saving data: {e}")
    
    def calculate_flight_time(self, distance_inches, angle_from_vertical, rpm, 
                             target_height_inches=104, robot_height_inches=24):
        """
        Calculate estimated flight time for projectile.
        
        Args:
            distance_inches: Horizontal distance to target
            angle_from_vertical: Hood angle from vertical plane (0° = up, 90° = horizontal)
            rpm: Shooter wheel RPM
            target_height_inches: Target height (default 104" for high goal)
            robot_height_inches: Height of shooter on robot
        
        Returns:
            Estimated flight time in seconds
        """
        # Convert angle from vertical to angle from horizontal for physics
        angle_from_horizontal = 90 - angle_from_vertical
        angle_rad = math.radians(angle_from_horizontal)
        
        # Estimate exit velocity from RPM (this is a rough approximation)
        # Typical FRC ball diameter ~9.5", circumference ~30"
        wheel_circumference_inches = 30.0  # Adjust based on your flywheel
        velocity_ips = (rpm * wheel_circumference_inches) / 60.0  # inches per second
        
        # Adjust for compression/efficiency (typically 70-90%)
        efficiency = 0.8
        velocity_ips *= efficiency
        
        # Calculate components
        v_x = velocity_ips * math.cos(angle_rad)
        v_y = velocity_ips * math.sin(angle_rad)
        
        # Height difference
        delta_h = target_height_inches - robot_height_inches
        
        # Using kinematic equation: delta_y = v_y*t - 0.5*g*t^2
        # gravity in inches/s^2 (32.2 ft/s^2 = 386.4 in/s^2)
        g = 386.4
        
        # Solve quadratic: -0.5*g*t^2 + v_y*t - delta_h = 0
        # t = (v_y ± sqrt(v_y^2 - 2*g*delta_h)) / g
        discriminant = v_y**2 - 2*g*delta_h
        
        if discriminant < 0:
            return None  # No solution
        
        t1 = (v_y + math.sqrt(discriminant)) / g
        t2 = (v_y - math.sqrt(discriminant)) / g
        
        # Use the positive, reasonable time
        valid_times = [t for t in [t1, t2] if t > 0]
        if not valid_times:
            return None
        
        # Verify horizontal distance matches
        time_estimate = min(valid_times)
        calculated_distance = v_x * time_estimate
        
        # If calculated distance is way off, return None
        if abs(calculated_distance - distance_inches) / distance_inches > 0.3:
            return None
        
        return time_estimate
    
    def suggest_starting_values(self, distance_inches, target_height_inches=104):
        """
        Suggest starting RPM and angle values based on collected data or physics
        
        Args:
            distance_inches: Horizontal distance to target
            target_height_inches: Target height
        
        Returns:
            (suggested_angle_from_vertical, suggested_rpm, source, confidence)
            source: 'interpolated', 'extrapolated', or 'heuristic'
            confidence: 'high', 'medium', or 'low'
        """
        # Filter data points for matching target_height (static tests only, vpar=0)
        matching_points = [
            pt for pt in self.data_points 
            if abs(pt['target_height'] - target_height_inches) < 0.5
        ]
        
        if len(matching_points) >= 2:
            # Sort by distance
            matching_points.sort(key=lambda p: p['distance'])
            distances = [pt['distance'] for pt in matching_points]
            angles = [pt['angle'] for pt in matching_points]
            rpms = [pt['rpm'] for pt in matching_points]
            
            min_dist = min(distances)
            max_dist = max(distances)
            
            # Check if we're interpolating or extrapolating
            if min_dist <= distance_inches <= max_dist:
                # Interpolation - high confidence
                angle = np.interp(distance_inches, distances, angles)
                rpm = np.interp(distance_inches, distances, rpms)
                return float(angle), int(rpm), 'interpolated', 'high'
            elif distance_inches < min_dist:
                # Extrapolating below - medium confidence
                if distance_inches > min_dist - 24:  # Within 24" below
                    angle = np.interp(distance_inches, distances, angles)
                    rpm = np.interp(distance_inches, distances, rpms)
                    return float(angle), int(rpm), 'extrapolated', 'medium'
            elif distance_inches > max_dist:
                # Extrapolating above - medium confidence
                if distance_inches < max_dist + 24:  # Within 24" above
                    angle = np.interp(distance_inches, distances, angles)
                    rpm = np.interp(distance_inches, distances, rpms)
                    return float(angle), int(rpm), 'extrapolated', 'medium'
        
        # Fall back to heuristics if no data or too far from existing data
        if distance_inches < 60:
            angle_from_vertical = 20  # Steep shot
            base_rpm = 3000
        elif distance_inches < 120:
            angle_from_vertical = 30
            base_rpm = 3500
        elif distance_inches < 180:
            angle_from_vertical = 40
            base_rpm = 4000
        elif distance_inches < 240:
            angle_from_vertical = 50
            base_rpm = 4500
        else:
            angle_from_vertical = 60
            base_rpm = 5000
        
        return angle_from_vertical, base_rpm, 'heuristic', 'low'
    
    def collect_data_point(self):
        """Interactive data point collection"""
        print("\n" + "="*60)
        print("DATA POINT COLLECTION (Static Position)")
        print("="*60)
        
        # Get target height
        target_height = input("Target height in inches (default 104 for high goal): ").strip()
        target_height = float(target_height) if target_height else 104.0
        
        # Static testing only - vpar will be calculated later for shoot-on-fly
        vpar = 0.0  # Robot velocity parallel to target (static = 0)
        
        # Get distance
        distance = input("Enter test distance in inches: ").strip()
        if not distance:
            print("Distance required!")
            return
        distance = float(distance)
        
        # Suggest starting values
        suggested_angle, suggested_rpm, source, confidence = self.suggest_starting_values(
            distance, target_height
        )
        
        print(f"\nSuggested starting values ({source}, {confidence} confidence):")
        print(f"  Hood angle (from vertical): {suggested_angle:.1f}°")
        print(f"  RPM: {suggested_rpm}")
        
        if source == 'interpolated':
            print("  ✓ Based on your collected data (interpolated)")
        elif source == 'extrapolated':
            print("  ⚠ Based on your data, but outside tested range (extrapolated)")
        else:
            print("  ℹ Based on general heuristics (collect more data for better suggestions)")
        
        print("\nTest these values and adjust until shots are consistently hitting the target.")
        print("Remember: Hood angle is from VERTICAL (0° = straight up, 90° = horizontal)")
        
        # Get tuned values
        angle_str = input(f"\nEnter final hood angle from vertical (suggested {suggested_angle}°): ").strip()
        angle = float(angle_str) if angle_str else suggested_angle
        
        rpm_str = input(f"Enter final RPM (suggested {suggested_rpm}): ").strip()
        rpm = int(float(rpm_str)) if rpm_str else suggested_rpm
        
        # Calculate flight time
        flight_time = self.calculate_flight_time(distance, angle, rpm, target_height)
        if flight_time:
            print(f"\nEstimated flight time: {flight_time:.3f} seconds")
        else:
            print("\nCouldn't calculate flight time (physics check failed)")
            flight_time_str = input("Enter measured flight time in seconds (or leave blank): ").strip()
            flight_time = float(flight_time_str) if flight_time_str else 0.0
        
        # Confirm
        print(f"\nData Point Summary:")
        print(f"  Target Height: {target_height}\"")
        print(f"  Distance: {distance}\"")
        print(f"  Robot Velocity: {vpar} in/s (static)")
        print(f"  Hood Angle: {angle}° (from vertical)")
        print(f"  RPM: {rpm}")
        print(f"  Flight Time: {flight_time:.3f}s")
        
        confirm = input("\nSave this data point? (y/n): ").strip().lower()
        if confirm == 'y':
            self.data_points.append({
                'target_height': target_height,
                'distance': distance,
                'vpar': vpar,
                'angle': angle,
                'rpm': rpm,
                'flight_time': flight_time
            })
            self.save_data()
            print("✓ Data point saved!")
        else:
            print("Data point discarded")
    
    def view_data_points(self):
        """Display all collected data points"""
        if not self.data_points:
            print("\nNo data points collected yet.")
            return
        
        print("\n" + "="*60)
        print("COLLECTED DATA POINTS (Static Tests)")
        print("="*60)
        print(f"{'#':<4} {'Height':<8} {'Dist':<8} {'Vel':<8} {'Angle':<8} {'RPM':<8} {'Time':<8}")
        print("-"*60)
        
        for i, pt in enumerate(self.data_points, 1):
            print(f"{i:<4} {pt['target_height']:<8.1f} {pt['distance']:<8.1f} "
                  f"{pt['vpar']:<8.1f} {pt['angle']:<8.1f} {pt['rpm']:<8} "
                  f"{pt['flight_time']:<8.3f}")
        
        # Show data coverage assessment
        self.assess_data_coverage()
    
    def assess_data_coverage(self):
        """Assess the quality and coverage of collected data"""
        if len(self.data_points) < 3:
            print("\n⚠ Need at least 3 data points to generate a lookup table")
            print(f"   Current: {len(self.data_points)} point(s)")
            return
        
        # Group by (target_height, vpar)
        grouped = {}
        for pt in self.data_points:
            key = (pt['target_height'], pt['vpar'])
            if key not in grouped:
                grouped[key] = []
            grouped[key].append(pt)
        
        print("\n" + "="*60)
        print("DATA COVERAGE ASSESSMENT")
        print("="*60)
        
        for (target_height, vpar), points in grouped.items():
            points.sort(key=lambda p: p['distance'])
            distances = [pt['distance'] for pt in points]
            min_dist, max_dist = min(distances), max(distances)
            
            print(f"\nTarget Height: {target_height}\", Velocity: {vpar} in/s (static)")
            print(f"  Data points: {len(points)}")
            print(f"  Distance range: {min_dist:.1f}\" to {max_dist:.1f}\"")
            print(f"  Coverage: {max_dist - min_dist:.1f}\" span")
            
            # Check spacing
            if len(points) >= 2:
                gaps = [distances[i+1] - distances[i] for i in range(len(distances)-1)]
                max_gap = max(gaps)
                avg_gap = sum(gaps) / len(gaps)
                print(f"  Average spacing: {avg_gap:.1f}\"")
                if max_gap > 48:
                    print(f"  ⚠ Large gap detected: {max_gap:.1f}\" (consider adding points)")
                
            # Quality assessment
            if len(points) < 3:
                quality = "Poor - need at least 3 points"
                symbol = "❌"
            elif len(points) < 5:
                quality = "Fair - more points recommended"
                symbol = "⚠"
            elif max_dist - min_dist < 60:
                quality = "Fair - limited distance range"
                symbol = "⚠"
            else:
                quality = "Good - sufficient for interpolation"
                symbol = "✓"
            
            print(f"  {symbol} Quality: {quality}")
        
        print("\nRecommendations:")
        for (target_height, vpar), points in grouped.items():
            if len(points) < 5:
                print(f"  • Add {5 - len(points)} more point(s) for target_height={target_height}")
        
        total_unique_configs = len(grouped)
        if total_unique_configs == 1:
            print(f"  • Consider testing different target heights if needed")
        print(f"  • After static table is complete, use option 6 to calculate velocity adjustments")

    
    def delete_data_point(self):
        """Delete a data point"""
        self.view_data_points()
        if not self.data_points:
            return
        
        try:
            idx = int(input("\nEnter data point number to delete (0 to cancel): ")) - 1
            if idx >= 0 and idx < len(self.data_points):
                deleted = self.data_points.pop(idx)
                self.save_data()
                print(f"✓ Deleted data point at {deleted['distance']}\"")
        except (ValueError, IndexError):
            print("Invalid selection")
    
    def generate_lookup_table(self):
        """Generate interpolated lookup table from collected data"""
        if len(self.data_points) < 3:
            print("\nNeed at least 3 data points to generate lookup table")
            return
        
        print("\n" + "="*60)
        print("GENERATE LOOKUP TABLE")
        print("="*60)
        
        # Get interpolation parameters
        print("\nEnter distance range for lookup table:")
        min_dist = float(input("  Minimum distance (inches): "))
        max_dist = float(input("  Maximum distance (inches): "))
        dist_step = float(input("  Distance step (inches, e.g., 6): "))
        
        # Group data by (target_height, vpar)
        grouped_data = {}
        for pt in self.data_points:
            key = (pt['target_height'], pt['vpar'])
            if key not in grouped_data:
                grouped_data[key] = []
            grouped_data[key].append(pt)
        
        # Build lookup table
        self.lookup_table = {}
        
        for (target_height, vpar), points in grouped_data.items():
            if len(points) < 3:
                print(f"Skipping target_height={target_height}, vpar={vpar} (need at least 3 points)")
                continue
            
            # Sort points by distance (CRITICAL for interpolation)
            points.sort(key=lambda p: p['distance'])
            
            # Extract data for interpolation
            distances = np.array([pt['distance'] for pt in points])
            angles = np.array([pt['angle'] for pt in points])
            rpms = np.array([pt['rpm'] for pt in points])
            times = np.array([pt['flight_time'] for pt in points])
            
            # Create interpolation grid
            grid_distances = np.arange(min_dist, max_dist + dist_step, dist_step)
            
            # Interpolate angle, rpm, and time
            interp_angles = np.interp(grid_distances, distances, angles)
            interp_rpms = np.interp(grid_distances, distances, rpms)
            interp_times = np.interp(grid_distances, distances, times)
            
            # Build nested structure
            height_key = str(target_height)
            if height_key not in self.lookup_table:
                self.lookup_table[height_key] = {}
            
            for dist, angle, rpm, time in zip(grid_distances, interp_angles, interp_rpms, interp_times):
                dist_key = str(int(dist))
                if dist_key not in self.lookup_table[height_key]:
                    self.lookup_table[height_key][dist_key] = {}
                
                vpar_key = str(int(vpar))
                self.lookup_table[height_key][dist_key][vpar_key] = {
                    'angle_deg': round(float(angle), 2),
                    'rpm': int(round(float(rpm))),
                    'time_s': round(float(time), 3)
                }
        
        print(f"\n✓ Generated lookup table with {len(self.lookup_table)} target heights")
        print("  → Run option 6 to add velocity-adjusted entries for shoot-on-fly.")

        # Preview
        preview = input("\nPreview lookup table? (y/n): ").strip().lower()
        if preview == 'y':
            print(json.dumps(self.lookup_table, indent=2))
    
    def export_lookup_table(self):
        """Export lookup table to JSON file"""
        if not self.lookup_table:
            print("\nNo lookup table generated yet. Generate one first.")
            return
        
        filename = input("\nEnter output filename (default: lookup_table.json): ").strip()
        if not filename:
            filename = "lookup_table.json"

        # Count total entries and how many have velocity-adjusted data
        total_entries = 0
        velocity_entries = 0
        for dist_dict in self.lookup_table.values():
            for vpar_dict in dist_dict.values():
                total_entries += 1
                if len(vpar_dict) > 1:
                    velocity_entries += 1

        try:
            with open(filename, 'w') as f:
                json.dump(self.lookup_table, f, indent=2)
            print(f"✓ Exported lookup table to {filename}")
            print(f"  Distances: {total_entries}  "
                  f"({velocity_entries} with velocity-adjusted entries)")
            if velocity_entries == 0:
                print("  Tip: Run option 6 to add shoot-on-fly velocity adjustments.")
        except Exception as e:
            print(f"Error exporting: {e}")
    
    @staticmethod
    def _compute_velocity_entry(task):
        """
        Compute one velocity-adjusted lookup entry.
        Returns (height_key, dist_key, vpar, entry_dict, out_of_range, d_eff, d_min, d_max)
        """
        height_key, dist_key, d, vpar, interp = task
        d_arr = interp['distances']
        a_arr = interp['angles']
        r_arr = interp['rpms']
        t_arr = interp['times']
        d_min = float(d_arr[0])
        d_max = float(d_arr[-1])

        # Iterative effective-distance refinement (2 passes)
        t_eff = float(np.interp(d, d_arr, t_arr))
        for _ in range(2):
            d_eff     = d - vpar * t_eff
            d_clamped = float(np.clip(d_eff, d_min, d_max))
            new_angle = float(np.interp(d_clamped, d_arr, a_arr))
            new_rpm   = int(round(float(np.interp(d_clamped, d_arr, r_arr))))
            t_eff     = float(np.interp(d_clamped, d_arr, t_arr))

        # Velocity-based RPM compensation:
        # Shoot-on-fly consistently undershoots vs. stationary; add ~100 RPM at a
        # reference speed of 60 in/s (5 ft/s), scaled linearly with |vpar|.
        # Tune VELOCITY_RPM_COMP_FACTOR (RPM per in/s) to adjust the compensation.
        # TODO: This value needs tuned
        VELOCITY_RPM_COMP_FACTOR = 500.0 / 60.0
        rpm_velocity_comp = int(round(VELOCITY_RPM_COMP_FACTOR * abs(vpar)))
        new_rpm += rpm_velocity_comp

        out_of_range = d_eff < d_min or d_eff > d_max
        entry = {
            'angle_deg':         round(new_angle, 2),
            'rpm':               new_rpm,
            'time_s':            round(t_eff, 3),
            'effective_dist_in': round(d_eff, 1),
            'rpm_velocity_comp': rpm_velocity_comp,
        }
        return height_key, dist_key, vpar, entry, out_of_range, d_eff, d_min, d_max

    def calculate_velocity_adjustments(self):
        """
        Calculate trajectory adjustments for robot velocity parallel to target.

        Physics:
            When the robot moves at vpar (in/s, positive = toward target) the ball
            inherits that velocity.  The effective launch distance in the field frame is

                d_eff = d - vpar * t_flight

            so we look up the static angle/RPM/time for d_eff instead of d.  Two
            iterations of the refinement loop are enough for convergence to < 0.5 %.

        The static lookup table (vpar=0) must be generated first (option 4).
        Results are stored directly in self.lookup_table under new vpar keys and can
        then be exported with option 5.
        """
        print("\n" + "="*60)
        print("VELOCITY ADJUSTMENT CALCULATOR")
        print("="*60)

        if not self.lookup_table:
            print("\n⚠  Generate the static lookup table first (option 4).")
            return

        print("\nEnter robot velocity range to compute adjustments for.")
        print("Positive = moving toward target, Negative = moving away.")
        min_vpar  = float(input("  Minimum velocity (in/s): "))
        max_vpar  = float(input("  Maximum velocity (in/s): "))
        vpar_step = float(input("  Velocity step  (in/s, e.g., 12): "))

        # Generate values; exclude 0.0 (already covered by the static entries)
        raw_values  = np.arange(min_vpar, max_vpar + vpar_step / 2, vpar_step)
        vpar_values = [float(v) for v in raw_values if abs(v) > 1e-6]

        if not vpar_values:
            print("No non-zero velocities in the specified range.")
            return

        print(f"\n  Velocities to compute: "
              f"{[round(v, 1) for v in vpar_values]} in/s  ({len(vpar_values)} values)")

        # ── Build per-target_height interpolation arrays from static (vpar=0) entries ──
        static_interp = {}
        for height_key, dist_dict in self.lookup_table.items():
            dists, angles, rpms, times = [], [], [], []
            for dist_key, vpar_dict in dist_dict.items():
                if '0' in vpar_dict:
                    entry = vpar_dict['0']
                    dists.append(float(dist_key))
                    angles.append(entry['angle_deg'])
                    rpms.append(entry['rpm'])
                    times.append(entry['time_s'])
            if len(dists) >= 2:
                sorted_data = sorted(zip(dists, angles, rpms, times))
                dists, angles, rpms, times = zip(*sorted_data)
                static_interp[height_key] = {
                    'distances': np.array(dists),
                    'angles':    np.array(angles),
                    'rpms':      np.array(rpms),
                    'times':     np.array(times),
                }

        if not static_interp:
            print("\n⚠  No static (vpar=0) entries found in lookup table.")
            return

        # ── Build task list (one task per height × dist × vpar combination) ──
        tasks = []
        for height_key, interp in static_interp.items():
            for dist_key in self.lookup_table[height_key]:
                d         = float(dist_key)
                vpar_dict = self.lookup_table[height_key][dist_key]
                for vpar in vpar_values:
                    if str(int(vpar)) not in vpar_dict:  # skip already-present entries
                        tasks.append((height_key, dist_key, d, vpar, interp))

        if not tasks:
            print("\n✓  All requested velocity entries already exist in the table.")
            return

        print(f"  Computing {len(tasks)} entries across "
              f"{os.cpu_count()} logical CPUs…")

        # ── Parallel computation ──────────────────────────────────────────────────
        results  = []
        warnings = []
        with concurrent.futures.ThreadPoolExecutor() as executor:
            for result in executor.map(self._compute_velocity_entry, tasks):
                results.append(result)

        # ── Write results back to lookup table (single-threaded, no lock needed) ─
        for height_key, dist_key, vpar, entry, out_of_range, d_eff, d_min, d_max in results:
            self.lookup_table[height_key][dist_key][str(int(vpar))] = entry
            if out_of_range:
                warnings.append(
                    f"  ⚠ height={height_key}, dist={dist_key}\", "
                    f"vpar={vpar:+.0f} in/s → d_eff={d_eff:.1f}\" "
                    f"is outside measured range [{d_min:.0f}–{d_max:.0f}\"]"
                )

        print(f"\n✓  Added {len(results)} velocity-adjusted entries.")
        if warnings:
            print("\nExtrapolation warnings (results clamped to measured range):")
            for w in warnings:
                print(w)
        print("\n   Export the table (option 5) to save to JSON.")
    
    def main_menu(self):
        """Main interactive menu"""
        while True:
            print("\n" + "="*60)
            print("FRC TURRET LOOKUP TABLE GENERATOR")
            print("="*60)
            print(f"Current data points: {len(self.data_points)} (static)")
            print("\n1. Collect new data point (static)")
            print("2. View all data points")
            print("3. Delete a data point")
            print("4. Generate static lookup table")
            print("5. Export lookup table to JSON")
            print("6. Calculate velocity adjustments (shoot-on-fly)")
            print("7. Exit")
            
            choice = input("\nSelect option: ").strip()
            
            if choice == '1':
                self.collect_data_point()
            elif choice == '2':
                self.view_data_points()
            elif choice == '3':
                self.delete_data_point()
            elif choice == '4':
                self.generate_lookup_table()
            elif choice == '5':
                self.export_lookup_table()
            elif choice == '6':
                self.calculate_velocity_adjustments()
            elif choice == '7':
                print("\nGoodbye!")
                break
            else:
                print("Invalid option")


def main():
    generator = TurretLookupGenerator()
    generator.main_menu()


if __name__ == '__main__':
    main()
