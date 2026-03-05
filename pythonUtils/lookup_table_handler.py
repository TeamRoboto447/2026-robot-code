import json
import time
import logging
import os
import math
from networktables import NetworkTables

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("TurretPi")

TABLE_PATH = os.path.join(os.path.dirname(__file__), "lookup_table.json")
TEAM_NUMBER = 447
ROBORIO_IP = f"10.{TEAM_NUMBER // 100}.{TEAM_NUMBER % 100}.2"
RUN_AS_SERVER = False # Set to True to host the server for testing

DIST_MIN = 100
DIST_MAX = 360
DIST_STEP = 1

VPAR_MIN = -240
VPAR_MAX = 240
VPAR_STEP = 10

TURRET_ANGLE_MIN = -90  # degrees, hardware limit
TURRET_ANGLE_MAX = 80   # degrees, hardware limit

def load_lookup_table():
    logger.info("Loading lookup table from {TABLE_PATH}")

    try:
        with open(TABLE_PATH, "r") as f:
            return json.load(f)
    except FileNotFoundError:
        logger.error(f"Lookup table not found at {TABLE_PATH}")
        return None
    except Exception as e:
        logger.error(f"Error loading lookup table: {e}")
        return None

def get_closest_height(height_in, available_heights):
    """Finds the closest target height key from the available options."""
    try:
        height_floats = [float(h) for h in available_heights]
        closest = min(height_floats, key=lambda x: abs(x - height_in))
        for k in available_heights:
            if abs(float(k) - closest) < 0.001:
                return k
    except (ValueError, TypeError):
        pass
    return None

def calculate_distance_and_vpar(robot_x, robot_y, target_x, target_y, robot_vx, robot_vy):
    """Calculate distance, vpar, and vperp from positions and velocities.
    
    Args:
        robot_x, robot_y: Robot position coordinates
        target_x, target_y: Target position coordinates
        robot_vx, robot_vy: Robot velocity components
    
    Returns:
        tuple: (distance, vpar, vperp) where:
            - distance is in same units as positions
            - vpar is velocity component parallel to line-of-sight
            - vperp is velocity component perpendicular to line-of-sight
    """
    # Calculate distance using Pythagorean theorem
    dx = target_x - robot_x
    dy = target_y - robot_y
    distance = math.sqrt(dx**2 + dy**2)
    
    if distance > 0:
        # Unit vector from robot to target
        ux = dx / distance
        uy = dy / distance
        
        # vpar: velocity parallel to line of sight (dot product)
        vpar = robot_vx * ux + robot_vy * uy
        
        # vperp: velocity perpendicular to line of sight
        # Perpendicular unit vector is (-uy, ux) rotated 90 degrees
        vperp = -robot_vx * uy + robot_vy * ux
    else:
        vpar = 0.0
        vperp = 0.0
    
    return distance, vpar, vperp

def main():
    lookup_table = load_lookup_table()
    if not lookup_table:
        return
    
    if RUN_AS_SERVER:
        logger.info("Stating networkTables in SERVER mode")
        NetworkTables.startServer(listenAddress="127.0.0.1")
    else:
        logger.info(f"Starting NetworkTables in CLIENT mode and connecting to {ROBORIO_IP}")
        NetworkTables.initialize(server=ROBORIO_IP)
    
    # Get separate tables to match robot structure
    turret_assembly_table = NetworkTables.getTable("TurretAssembly")
    targeting_table = turret_assembly_table.getSubTable("targetting")
    flywheel_table = turret_assembly_table.getSubTable("flywheel")
    hood_table = turret_assembly_table.getSubTable("hood")
    rotation_table = turret_assembly_table.getSubTable("rotation")
    
    logger.info("NT Client started. Waiting for inputs...")

    # Initialize expected entries from robot (inputs)
    targeting_table.putNumber("robot_x", 0.0)
    targeting_table.putNumber("robot_y", 0.0)
    targeting_table.putNumber("robot_angle", 0.0)
    targeting_table.putNumber("robot_vx", 0.0)
    targeting_table.putNumber("robot_vy", 0.0)
    targeting_table.putNumber("target_x", 0.0)
    targeting_table.putNumber("target_y", 136.0)
    targeting_table.putNumber("target_height", 72)

    # Outputs to robot's expected tables
    hood_table.putNumber("Target Hood Angle", 0.0)
    flywheel_table.putNumber("Target Flywheel RPM", 0.0)
    rotation_table.putNumber("Target Turret Angle", 0.0)
    targeting_table.putBoolean("has_valid_shot", False)

    last_processed_inputs = None

    while True:
        robot_x = targeting_table.getNumber("robot_x", 0.0)
        robot_y = targeting_table.getNumber("robot_y", 0.0)
        robot_angle = targeting_table.getNumber("robot_angle", 0.0)
        robot_angle = targeting_table.getNumber("robot_angle", 0.0)
        robot_vx = targeting_table.getNumber("robot_vx", 0.0)
        robot_vy = targeting_table.getNumber("robot_vy", 0.0)
        target_x = targeting_table.getNumber("target_x", 0.0)
        target_y = targeting_table.getNumber("target_y", 0.0)
        height_raw = targeting_table.getNumber("target_height", 72.0)

        current_inputs = (robot_x, robot_y, robot_angle, robot_vx, robot_vy, target_x, target_y, height_raw)
        
        # Calculate distance, vpar, and vperp from positions and velocities
        dist_raw, vpar_raw, vperp_raw = calculate_distance_and_vpar(robot_x, robot_y, target_x, target_y, robot_vx, robot_vy)

        if current_inputs != last_processed_inputs:
            last_processed_inputs = current_inputs
            
            height_key = get_closest_height(height_raw, lookup_table.keys())
            if height_key and height_key in lookup_table:
                dist_rounded = int(round(dist_raw / DIST_STEP) * DIST_STEP) # round to distance step
                dist_key = str(dist_rounded)

                vpar_rounded = int(round(vpar_raw / VPAR_STEP) * VPAR_STEP) # round to velocity step
                vpar_key = str(vpar_rounded)

                logger.debug(f"Input: dist={dist_raw:.1f}, vpar={vpar_raw:.1f}, height={height_raw:.1f} -> Keys: {height_key}, {dist_key}, {vpar_key}")
                
                height_data = lookup_table.get(height_key, {})
                dist_data = height_data.get(dist_key, {})
                solution = dist_data.get(vpar_key)

                if solution:
                    hood_table.putNumber("Target Hood Angle", solution["angle_deg"])
                    flywheel_table.putNumber("Target Flywheel RPM", solution["rpm"])
                    targeting_table.putBoolean("has_valid_shot", True)
                    
                    # Calculate turret angle: angle to target relative to robot heading + lead
                    # angle_to_target is the field-relative bearing from robot to target
                    dx = target_x - robot_x
                    dy = target_y - robot_y
                    angle_to_target_deg = math.degrees(math.atan2(dy, dx))
                    # Turret angle needed, in CW-positive convention (turret frame):
                    # angle_to_target and robot_angle are both CCW-positive, so their
                    # difference is CCW-positive. Negate to convert to CW-positive, and
                    # subtract 180° to account for turret 0 facing the back of the robot.
                    # Simplified: turret_aim = robot_angle - angle_to_target + 180
                    turret_aim_deg = robot_angle - angle_to_target_deg + 180
                    # Normalize to [-180, 180]
                    turret_aim_deg = (turret_aim_deg + 180) % 360 - 180

                    # Add lead angle from perpendicular velocity.
                    # vperp is CCW-positive; the turret is CW-positive.
                    # Moving left (vperp > 0) → projectile drifts left → aim right (+CW). Same sign.
                    # Moving right (vperp < 0) → projectile drifts right → aim left (-CW). Same sign.
                    # θ_lead ≈ arctan(v_perp * t_flight / distance)
                    time_of_flight = solution.get("time_s", 0.0)
                    if dist_raw > 0 and time_of_flight > 0:
                        lead_angle_rad = math.atan2(vperp_raw * time_of_flight, dist_raw)
                        lead_angle_deg = math.degrees(lead_angle_rad)
                    else:
                        lead_angle_deg = 0.0

                    turret_angle_deg = turret_aim_deg + lead_angle_deg
                    # Normalize final angle to [-180, 180]
                    turret_angle_deg = (turret_angle_deg + 180) % 360 - 180

                    if TURRET_ANGLE_MIN <= turret_angle_deg <= TURRET_ANGLE_MAX:
                        rotation_table.putNumber("Target Turret Angle", turret_angle_deg)
                        logger.info(f"MATCH: Height={height_key} Dist={dist_key} Vpar={vpar_key} -> Angle={solution['angle_deg']:.2f}, RPM={solution['rpm']:.0f}, Turret={turret_angle_deg:.2f}° (aim={turret_aim_deg:.2f}°, lead={lead_angle_deg:.2f}°)")
                    else:
                        targeting_table.putBoolean("has_valid_shot", False)
                        rotation_table.putNumber("Target Turret Angle", 0.0)
                        logger.warning(f"OUT OF RANGE: Turret angle {turret_angle_deg:.2f}° exceeds limits [{TURRET_ANGLE_MIN}, {TURRET_ANGLE_MAX}]°")
                else:
                    targeting_table.putBoolean("has_valid_shot", False)
                    rotation_table.putNumber("Target Turret Angle", 0.0)
            else:
                targeting_table.putBoolean("has_valid_shot", False)
                rotation_table.putNumber("Target Turret Angle", 0.0)

        time.sleep(0.02)


if __name__ == "__main__":
    main()
