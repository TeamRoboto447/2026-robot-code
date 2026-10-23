package frc.robot.utils.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class TurretAnglesTest {
    private static final double EPS = 1e-9;

    @Test
    void normalize180WrapsIntoHalfOpenRange() {
        assertEquals(0.0, TurretAngles.normalize180(360.0), EPS);
        assertEquals(-180.0, TurretAngles.normalize180(180.0), EPS);
        assertEquals(-180.0, TurretAngles.normalize180(-180.0), EPS);
        assertEquals(170.0, TurretAngles.normalize180(-190.0), EPS);
        assertEquals(-90.0, TurretAngles.normalize180(270.0), EPS);
    }

    @Test
    void turretAngleFollowsConvention() {
        // Target straight ahead of an unrotated robot: the turret points at the front, 180 from the back.
        assertEquals(-180.0, TurretAngles.turretAngleForBearingDeg(0.0, 0.0), EPS);
        // Target to the robot's left (CCW): clockwise-positive from the back, that is +90.
        assertEquals(90.0, TurretAngles.turretAngleForBearingDeg(0.0, Math.PI / 2), EPS);
        // Target behind the robot: turret angle 0.
        assertEquals(0.0, TurretAngles.turretAngleForBearingDeg(0.0, Math.PI), EPS);
        // Rotating the robot CCW by 90 degrees with the same field bearing moves the turret by +90.
        assertEquals(-90.0, TurretAngles.turretAngleForBearingDeg(Math.PI / 2, 0.0), EPS);
    }

    @Test
    void wrapIntoTravelKeepsAnglesAlreadyInTravel() {
        assertEquals(170.0, TurretAngles.wrapIntoTravel(170.0, -190.0, 170.0), EPS);
        assertEquals(-190.0, TurretAngles.wrapIntoTravel(-190.0, -190.0, 170.0), EPS);
        assertEquals(12.5, TurretAngles.wrapIntoTravel(12.5, -190.0, 170.0), EPS);
    }

    @Test
    void wrapIntoTravelUsesEquivalentAngle() {
        // The (170, 180] band used to be dropped; its equivalent is within travel.
        assertEquals(-185.0, TurretAngles.wrapIntoTravel(175.0, -190.0, 170.0), EPS);
        assertEquals(-180.0, TurretAngles.wrapIntoTravel(180.0, -190.0, 170.0), EPS);
        assertEquals(10.0, TurretAngles.wrapIntoTravel(370.0, -190.0, 170.0), EPS);
        assertEquals(160.0, TurretAngles.wrapIntoTravel(-200.0, -190.0, 170.0), EPS);
    }

    @Test
    void wrapIntoTravelClampsToNearerLimitInDeadZone() {
        assertEquals(90.0, TurretAngles.wrapIntoTravel(150.0, -90.0, 90.0), EPS);
        assertEquals(-90.0, TurretAngles.wrapIntoTravel(-150.0, -90.0, 90.0), EPS);
    }
}
