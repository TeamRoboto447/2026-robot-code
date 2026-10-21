package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Random;
import org.junit.jupiter.api.Test;

class QuestOffsetCalibratorTest {
    private static final double MOUNT_YAW = Math.PI / 2;
    private static final Translation2d TRUE_OFFSET = new Translation2d(-0.45, 0.19);

    /** Quest field poses for a robot spinning in place at (12.9, 3.8), with position noise. */
    private static QuestOffsetCalibrator spin(double degrees, double noiseM) {
        QuestOffsetCalibrator cal = new QuestOffsetCalibrator(MOUNT_YAW);
        Transform2d robotToQuest = new Transform2d(TRUE_OFFSET, new Rotation2d(MOUNT_YAW));
        Random rng = new Random(1);
        for (double deg = 30; deg <= 30 + degrees; deg += 2) {
            Pose2d quest = new Pose2d(12.9, 3.8, Rotation2d.fromDegrees(deg)).transformBy(robotToQuest);
            cal.addSample(new Pose2d(
                quest.getX() + rng.nextGaussian() * noiseM,
                quest.getY() + rng.nextGaussian() * noiseM,
                quest.getRotation()));
        }
        return cal;
    }

    @Test
    void recoversOffsetFromFullTurn() {
        Translation2d result = spin(360, 0.0).solve().orElseThrow();
        assertEquals(TRUE_OFFSET.getX(), result.getX(), 1e-6);
        assertEquals(TRUE_OFFSET.getY(), result.getY(), 1e-6);
    }

    @Test
    void toleratesNoise() {
        Translation2d result = spin(360, 0.01).solve().orElseThrow();
        assertEquals(TRUE_OFFSET.getX(), result.getX(), 0.01);
        assertEquals(TRUE_OFFSET.getY(), result.getY(), 0.01);
    }

    @Test
    void halfTurnIsEnough() {
        Translation2d result = spin(200, 0.0).solve().orElseThrow();
        assertEquals(TRUE_OFFSET.getX(), result.getX(), 1e-6);
    }

    @Test
    void refusesTooLittleRotation() {
        assertFalse(spin(90, 0.0).solve().isPresent());
    }
}
