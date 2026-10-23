package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utils.HubSchedule.Phase;

/**
 * Hub active/inactive timing for the 2026 game, read live from the DriverStation.
 *
 * <p>Call {@link #update()} once per loop; every getter then reads that same snapshot so they always
 * agree within a loop. The schedule itself lives in {@link HubSchedule}.
 *
 * <p><b>Fail-open:</b> if the FMS game data or alliance is missing/unparseable, or there is no usable
 * teleop clock, the hub is reported <i>active</i> (never block a shot because of bad data) and
 * {@link #isDataValid()} is false so dashboards can show the timer is not trustworthy.
 */
public class GameState {
    private static final class Snapshot {
        final double matchTime;
        final Phase phase;
        final Optional<Alliance> ourAlliance;
        final Optional<Alliance> inactiveFirst;

        Snapshot(double matchTime, Phase phase, Optional<Alliance> ourAlliance, Optional<Alliance> inactiveFirst) {
            this.matchTime = matchTime;
            this.phase = phase;
            this.ourAlliance = ourAlliance;
            this.inactiveFirst = inactiveFirst;
        }

        boolean dataValid() {
            return ourAlliance.isPresent() && inactiveFirst.isPresent();
        }

        /** True when the schedule can actually be applied (valid data and a teleop clock). */
        boolean scheduleApplies() {
            return dataValid() && phase.isTeleopPhase();
        }

        boolean ourHubInactiveFirst() {
            return ourAlliance.get() == inactiveFirst.get();
        }
    }

    private volatile Snapshot snapshot = new Snapshot(-1.0, Phase.NONE, Optional.empty(), Optional.empty());

    public GameState() {
        update();
    }

    /** Re-reads the DriverStation. Call once per robot loop before using any getter. */
    public void update() {
        double matchTime = DriverStation.getMatchTime();

        Phase phase;
        if (DriverStation.isAutonomousEnabled()) {
            phase = Phase.AUTO;
        } else if (DriverStation.isTeleopEnabled()) {
            phase = HubSchedule.phaseAt(matchTime);
        } else {
            phase = Phase.NONE;
        }

        snapshot = new Snapshot(matchTime, phase, DriverStation.getAlliance(), parseInactiveFirst());
    }

    /**
     * The FMS game-specific message names the alliance whose hub goes inactive first ('R' or 'B').
     * Anything else (empty, other characters) is treated as no data.
     */
    private static Optional<Alliance> parseInactiveFirst() {
        String message = DriverStation.getGameSpecificMessage();
        if (message == null || message.isEmpty()) return Optional.empty();
        switch (Character.toUpperCase(message.charAt(0))) {
            case 'R':
                return Optional.of(Alliance.Red);
            case 'B':
                return Optional.of(Alliance.Blue);
            default:
                return Optional.empty();
        }
    }

    /** Raw match time from the DriverStation (seconds remaining in the current period, -1 if unknown). */
    public double getMatchTime() {
        return snapshot.matchTime;
    }

    public Phase getPhase() {
        return snapshot.phase;
    }

    /** Increases through the match; changes whenever the phase changes. */
    public int getPhaseIndex() {
        return snapshot.phase.ordinal();
    }

    /** True when both the alliance and the FMS game data are available. */
    public boolean isDataValid() {
        return snapshot.dataValid();
    }

    /** The alliance whose hub is inactive first, if the game data has arrived. */
    public Optional<Alliance> getInactiveFirstAlliance() {
        return snapshot.inactiveFirst;
    }

    /** Whether our hub is active. Fails open (true) when the schedule cannot be applied. */
    public boolean isHubActive() {
        Snapshot s = snapshot;
        if (!s.scheduleApplies()) return true;
        return HubSchedule.isActive(s.phase, s.ourHubInactiveFirst());
    }

    public boolean isHubInactive() {
        return !isHubActive();
    }

    /** Seconds until our hub is next active; 0 if it is already active or the schedule cannot be applied. */
    public double getSecondsUntilHubActive() {
        Snapshot s = snapshot;
        if (!s.scheduleApplies()) return 0.0;
        return HubSchedule.secondsUntilActive(s.phase, s.matchTime, s.ourHubInactiveFirst());
    }

    /** Seconds left in the current phase (shift); 0 when there is no teleop clock. */
    public double getRemainingShiftTime() {
        Snapshot s = snapshot;
        return HubSchedule.secondsRemainingInPhase(s.phase, s.matchTime);
    }
}
