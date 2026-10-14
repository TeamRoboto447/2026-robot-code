package frc.robot.utils.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleUnaryOperator;
import org.junit.jupiter.api.Test;

class ShotPlannerTest {
    private static final double EPS = 1e-6;
    private static final Translation3d TARGET = new Translation3d(5.0, 0.0, 1.83);
    private static final Pose2d ORIGIN = new Pose2d();
    private static final ChassisSpeeds STILL = new ChassisSpeeds();

    /** A solver whose flight time is a function of distance; hood and RPM are placeholders. */
    private static ShotSolver tofSolver(DoubleUnaryOperator tofOfDistance) {
        return (d, h) -> new ShotSolution(30.0, 4000.0, tofOfDistance.applyAsDouble(d), true, ShotSolution.Source.LUT);
    }

    private static final ShotSolver ONE_SECOND = tofSolver(d -> 1.0);

    @Test
    void staticShotAimsStraightAtTarget() {
        var plan = ShotPlanner.plan(ORIGIN, STILL, new Translation2d(), TARGET, ONE_SECOND, 0.1, 0.0);
        assertTrue(plan.hasSolution());
        assertEquals(5.0, plan.targetDistanceM(), EPS);
        assertEquals(5.0, plan.virtualDistanceM(), EPS);
        assertEquals(0.0, plan.aimBearingRad(), EPS);
        assertEquals(-180.0, plan.turretAngleDeg(), EPS);
    }

    @Test
    void pivotOffsetRotatesWithRobot() {
        Pose2d pose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90));
        var plan = ShotPlanner.plan(pose, STILL, new Translation2d(-0.2, 0.1), TARGET, ONE_SECOND, 0.0, 0.0);
        // Robot-frame (-0.2, 0.1) rotated 90 degrees CCW is field (-0.1, -0.2).
        assertEquals(0.9, plan.pivot().getX(), EPS);
        assertEquals(1.8, plan.pivot().getY(), EPS);
        assertEquals(Math.hypot(5.0 - 0.9, -1.8), plan.targetDistanceM(), EPS);
    }

    @Test
    void sidewaysMotionAimsAtVirtualTarget() {
        // Moving +Y at 1 m/s with a 1 s flight: aim 1 m to the -Y side of the target.
        var plan = ShotPlanner.plan(ORIGIN, new ChassisSpeeds(0, 1.0, 0), new Translation2d(), TARGET, ONE_SECOND, 0.0, 0.0);
        assertEquals(Math.hypot(5.0, 1.0), plan.virtualDistanceM(), EPS);
        assertEquals(Math.atan2(-1.0, 5.0), plan.aimBearingRad(), EPS);
    }

    @Test
    void newtonSolvesDistanceDependentFlightTime() {
        // Driving at the target at 1 m/s with tof = 0.2 * D: tau = 0.2 * (5 - tau) -> tau = 1/1.2.
        ShotSolver solver = tofSolver(d -> 0.2 * d);
        var plan = ShotPlanner.plan(ORIGIN, new ChassisSpeeds(1.0, 0, 0), new Translation2d(), TARGET, solver, 0.0, 0.0);
        double tau = 1.0 / 1.2;
        assertEquals(5.0 - tau, plan.virtualDistanceM(), 1e-3);
        assertEquals(0.2 * plan.virtualDistanceM(), plan.solution().timeOfFlightS(), 1e-3);
    }

    @Test
    void latencyProjectsPositionAndHeading() {
        // 0.1 s at 1 m/s toward the target, while turning at 1 rad/s.
        var plan = ShotPlanner.plan(ORIGIN, new ChassisSpeeds(1.0, 0, 1.0), new Translation2d(), TARGET, tofSolver(d -> 0.0), 0.1, 0.0);
        assertEquals(4.9, plan.targetDistanceM(), EPS);
        // The turret is aimed from the projected heading (0.1 rad), the same way position is projected.
        assertEquals(TurretAngles.turretAngleForBearingDeg(0.1, 0.0), plan.turretAngleDeg(), EPS);
    }

    @Test
    void rangeLatencyScalesOnlyTheDistance() {
        // Driving toward the target at 1 m/s with a -0.02 s radial term shortens the distance by 2 cm.
        var plan = ShotPlanner.plan(ORIGIN, new ChassisSpeeds(1.0, 0, 0), new Translation2d(), TARGET, tofSolver(d -> 0.0), 0.0, -0.02);
        assertEquals(4.98, plan.targetDistanceM(), EPS);
        assertEquals(0.0, plan.aimBearingRad(), EPS);
    }

    @Test
    void noTargetOrSolverGivesNoSolutionButKeepsPivot() {
        Pose2d pose = new Pose2d(3.0, 4.0, new Rotation2d());
        var noTarget = ShotPlanner.plan(pose, STILL, new Translation2d(), null, ONE_SECOND, 0.0, 0.0);
        assertFalse(noTarget.hasSolution());
        assertEquals(3.0, noTarget.pivot().getX(), EPS);

        var noSolver = ShotPlanner.plan(pose, STILL, new Translation2d(), TARGET, null, 0.0, 0.0);
        assertFalse(noSolver.hasSolution());
        assertTrue(Double.isNaN(noSolver.turretAngleDeg()));
    }

    @Test
    void solverWithoutSolutionGivesNoSolution() {
        var plan = ShotPlanner.plan(ORIGIN, STILL, new Translation2d(), TARGET, (d, h) -> null, 0.0, 0.0);
        assertNull(plan.solution());
        assertEquals(5.0, plan.targetDistanceM(), EPS);
    }

    @Test
    void passesTargetHeightToSolver() {
        double[] seenHeight = {Double.NaN};
        ShotSolver recording = (d, h) -> {
            seenHeight[0] = h;
            return new ShotSolution(30.0, 4000.0, 0.5, true, ShotSolution.Source.PHYSICS);
        };
        ShotPlanner.plan(ORIGIN, STILL, new Translation2d(), TARGET, recording, 0.0, 0.0);
        assertEquals(1.83, seenHeight[0], EPS);
    }
}
