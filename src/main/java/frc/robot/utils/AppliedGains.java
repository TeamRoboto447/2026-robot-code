package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;

/**
 * Remembers which gains were last sent to a motor controller so unchanged NetworkTables values can skip the
 * blocking CAN {@code apply()}. A rejected apply is not retried for the same values until
 * {@link #RETRY_AFTER_FAILURE_S} has passed, so an unplugged or timing-out controller doesn't stall the loop
 * on every poll.
 */
public final class AppliedGains {
    private static final double RETRY_AFTER_FAILURE_S = 5.0;

    private double[] applied;
    private double[] rejected = null;
    private double rejectedAtS = Double.NEGATIVE_INFINITY;

    /** @param initial the gains already on the controller (e.g. the constructor's values) */
    public AppliedGains(double... initial) {
        this.applied = initial.clone();
    }

    /** True if these values should be sent to the controller now. */
    public boolean shouldApply(boolean force, double... values) {
        if (force) return true;
        if (Arrays.equals(values, applied)) return false;
        boolean recentlyRejected =
            Arrays.equals(values, rejected) && Timer.getFPGATimestamp() - rejectedAtS < RETRY_AFTER_FAILURE_S;
        return !recentlyRejected;
    }

    /** Records the outcome of an apply of {@code values}. */
    public void record(boolean ok, double... values) {
        if (ok) {
            applied = values.clone();
            rejected = null;
        } else {
            rejected = values.clone();
            rejectedAtS = Timer.getFPGATimestamp();
        }
    }
}
