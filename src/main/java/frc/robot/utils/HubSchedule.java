package frc.robot.utils;

/**
 * Pure (no WPILib / DriverStation) model of the 2026 REBUILT teleop hub schedule, so it can be unit tested.
 *
 * <p>All times are <b>seconds remaining in teleop</b>, counting down from {@link #TELEOP_LENGTH}:
 * Transition 140→130 (both hubs active), Shifts 1-4 at 25 s each (130→105→80→55→30, hubs alternate),
 * End Game 30→0 (both hubs active).
 *
 * <p>Boundary convention: a time exactly on a boundary belongs to the <i>earlier</i> phase
 * (e.g. 130.0 is still {@link Phase#TRANSITION}, 30.0 is still {@link Phase#SHIFT_4}, 0.0 is
 * {@link Phase#ENDGAME}).
 */
public final class HubSchedule {
    private HubSchedule() {}

    public static final double TELEOP_LENGTH = 140.0;

    /** Phases of a match, in order. The ordinal increases through the match (used to detect phase changes). */
    public enum Phase {
        /** No usable match clock (disabled, test, NaN, negative, or out of range). */
        NONE(Double.NaN),
        AUTO(Double.NaN),
        TRANSITION(130.0),
        SHIFT_1(105.0),
        SHIFT_2(80.0),
        SHIFT_3(55.0),
        SHIFT_4(30.0),
        ENDGAME(0.0);

        /** Remaining teleop time at which this phase ends (NaN for NONE / AUTO). */
        public final double endsAt;

        Phase(double endsAt) {
            this.endsAt = endsAt;
        }

        public boolean isTeleopPhase() {
            return this != NONE && this != AUTO;
        }
    }

    /** Phase for a given amount of teleop time remaining. Unusable times map to {@link Phase#NONE}. */
    public static Phase phaseAt(double teleopRemaining) {
        if (Double.isNaN(teleopRemaining) || teleopRemaining < 0.0 || teleopRemaining > TELEOP_LENGTH) {
            return Phase.NONE;
        }
        if (teleopRemaining >= Phase.TRANSITION.endsAt) return Phase.TRANSITION;
        if (teleopRemaining >= Phase.SHIFT_1.endsAt) return Phase.SHIFT_1;
        if (teleopRemaining >= Phase.SHIFT_2.endsAt) return Phase.SHIFT_2;
        if (teleopRemaining >= Phase.SHIFT_3.endsAt) return Phase.SHIFT_3;
        if (teleopRemaining >= Phase.SHIFT_4.endsAt) return Phase.SHIFT_4;
        return Phase.ENDGAME;
    }

    /**
     * Whether <i>our</i> hub is active in the given phase.
     *
     * @param ourHubInactiveFirst true if our alliance's hub is the one that is inactive in Shift 1
     */
    public static boolean isActive(Phase phase, boolean ourHubInactiveFirst) {
        switch (phase) {
            case SHIFT_1:
            case SHIFT_3:
                return !ourHubInactiveFirst;
            case SHIFT_2:
            case SHIFT_4:
                return ourHubInactiveFirst;
            default:
                // NONE, AUTO, TRANSITION, ENDGAME
                return true;
        }
    }

    /** Seconds left in the current phase; 0 for phases without a teleop clock. */
    public static double secondsRemainingInPhase(Phase phase, double teleopRemaining) {
        if (!phase.isTeleopPhase()) return 0.0;
        return Math.max(0.0, teleopRemaining - phase.endsAt);
    }

    /**
     * Seconds until our hub is next active: 0 if it is already active. Shifts alternate, so an inactive
     * shift always ends with the hub becoming active.
     */
    public static double secondsUntilActive(Phase phase, double teleopRemaining, boolean ourHubInactiveFirst) {
        if (isActive(phase, ourHubInactiveFirst)) return 0.0;
        return secondsRemainingInPhase(phase, teleopRemaining);
    }
}
