package frc.robot.utils.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.ArrayDeque;
import org.junit.jupiter.api.Test;

class TrackingLagEstimatorTest {
    private static final double DT = 0.02;
    private static final double SEED = 0.045;

    private static TrackingLagEstimator estimator() {
        return new TrackingLagEstimator(SEED, 20.0, 400.0, 15.0, 1.0, 0.25);
    }

    /** First-order servo with time constant tau: its tracking lag on a ramp is tau. */
    private static double runFirstOrderRamp(TrackingLagEstimator est, double tau, double rateDegPerS,
            double measurementAgeS, double seconds) {
        int ageLoops = (int) Math.round(measurementAgeS / DT);
        ArrayDeque<Double> history = new ArrayDeque<>();
        double setpoint = 0.0;
        double position = 0.0;
        for (int i = 0; i < (int) (seconds / DT); i++) {
            setpoint += rateDegPerS * DT;
            position += (setpoint - position) * (DT / tau);
            history.addLast(position);
            if (history.size() > ageLoops + 1) history.removeFirst();
            double measured = history.peekFirst(); // position as of measurementAgeS ago
            est.update(true, setpoint, measured, measurementAgeS, DT);
        }
        return est.getLagSeconds();
    }

    @Test
    void convergesToServoLag() {
        double lag = runFirstOrderRamp(estimator(), 0.08, 60.0, 0.0, 10.0);
        // Discrete first-order lag on a ramp settles at tau - dt.
        assertEquals(0.08 - DT, lag, 0.005);
    }

    @Test
    void subtractsMeasurementAge() {
        double lag = runFirstOrderRamp(estimator(), 0.08, 60.0, 0.04, 10.0);
        assertEquals(0.08 - DT, lag, 0.005);
    }

    @Test
    void ignoresStandingStillAndJumps() {
        TrackingLagEstimator est = estimator();
        for (int i = 0; i < 200; i++) est.update(true, 30.0, 25.0, 0.0, DT);
        assertEquals(SEED, est.getLagSeconds(), 1e-12);

        // A 90 degree jump in one loop (target switch / seam wrap) is far above the rate limit.
        est.update(true, 120.0, 25.0, 0.0, DT);
        est.update(true, 120.0, 100.0, 0.0, DT);
        assertEquals(SEED, est.getLagSeconds(), 1e-12);
    }

    @Test
    void notTrackingLeavesEstimateAlone() {
        TrackingLagEstimator est = estimator();
        double setpoint = 0.0;
        for (int i = 0; i < 300; i++) {
            setpoint += 60.0 * DT;
            est.update(false, setpoint, setpoint - 10.0, 0.0, DT);
        }
        assertEquals(SEED, est.getLagSeconds(), 1e-12);
    }

    @Test
    void largeErrorsAreIgnored() {
        TrackingLagEstimator est = estimator();
        double setpoint = 0.0;
        for (int i = 0; i < 300; i++) {
            setpoint += 60.0 * DT;
            est.update(true, setpoint, setpoint - 30.0, 0.0, DT);
        }
        assertEquals(SEED, est.getLagSeconds(), 1e-12);
    }

    @Test
    void clampsToMaximum() {
        // Pure 0.6 s delay at 21 deg/s: 12.6 deg of error, within the error limit, reads as 0.6 s.
        TrackingLagEstimator est = estimator();
        double setpoint = 0.0;
        for (int i = 0; i < 1000; i++) {
            setpoint += 21.0 * DT;
            est.update(true, setpoint, setpoint - 21.0 * 0.6, 0.0, DT);
        }
        assertEquals(0.25, est.getLagSeconds(), 1e-12);
    }
}
