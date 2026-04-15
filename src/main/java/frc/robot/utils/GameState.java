package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class GameState {
    private final double TELEOP_GAME_LENGTH = 140.0;
    private final double AUTONOMOUS_PERIOD_LENGTH = 20.0;
    private final double TRANSITION_END_TIME = 130.0;
    private final double SHIFT_1_END_TIME = 105.0;
    private final double SHIFT_2_END_TIME = 80.0;
    private final double SHIFT_3_END_TIME = 55.0;
    private final double SHIFT_4_END_TIME = 30.0;

    private DriverStation.Alliance currentAlliance = null;
    private DriverStation.Alliance inactiveFirst = null;

    private Boolean isCountingDown = null;
    private double lastMatchTime = Double.NaN;

    private Trigger gameEnableTrigger = new Trigger(DriverStation::isEnabled);

    public GameState() {
        update();
        gameEnableTrigger.onChange(Commands.runOnce(() -> update()));
    }

    private void updateAlliance() {
        currentAlliance = DriverStation.getAlliance().orElse(Alliance.Red);
    }

    private void updateGameData() {
        String gameData = DriverStation.getGameSpecificMessage();
        boolean validGameData = gameData.length() > 0
                && (gameData.charAt(0) == 'B' || gameData.charAt(0) == 'R');

        if (validGameData) {
            inactiveFirst = (gameData.charAt(0) == 'B') ? DriverStation.Alliance.Blue : DriverStation.Alliance.Red;
        }
    }

    public double getMatchTime() {
        double gameTime = DriverStation.getMatchTime();

        if (Double.isNaN(lastMatchTime)) { // Make sure last match time is good before trying to use it
            lastMatchTime = gameTime;
            return gameTime;
        }
        // Auto-detect direction on first tick of a new period
        if (isCountingDown == null && lastMatchTime >= 0) {
            isCountingDown = gameTime < lastMatchTime;
        }
        lastMatchTime = gameTime;

        if (Boolean.TRUE.equals(isCountingDown)) { // Because we are using a Boolean object instead of a boolean
                                                   // primative, we have to use a null-safe comparison
            return gameTime; // Practice mode: already time remaining
        }
        return gameTime; // Plain DS: convert elapsed → remaining
    }

    public boolean isHubActive() { /*
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }

        // Fail-safe default: keep hub active when not teleop enabled.
        if (!DriverStation.isTest()) {
            return true;
        }

        if (inactiveFirst == null || currentAlliance == null) {
            return true;
        } */

        double matchTime = getMatchTime();

        // Transition and endgame are always active.
        if (matchTime > TRANSITION_END_TIME || matchTime <= SHIFT_4_END_TIME) {
            return true;
        }

        boolean shift1Active = inactiveFirst != currentAlliance;
        if (matchTime > SHIFT_1_END_TIME) {
            return shift1Active;
        } else if (matchTime > SHIFT_2_END_TIME) {
            return !shift1Active;
        } else if (matchTime > SHIFT_3_END_TIME) {
            return shift1Active;
        }
        return !shift1Active;
    }

    public boolean isHubInactive() {
        return !isHubActive();
    }

    public double getRemainingShiftTime() {
        double matchTime = getMatchTime();

        if (matchTime > TRANSITION_END_TIME) {
            return matchTime - TRANSITION_END_TIME;
        } else if (matchTime > SHIFT_1_END_TIME) {
            return matchTime - SHIFT_1_END_TIME;
        } else if (matchTime > SHIFT_2_END_TIME) {
            return matchTime - SHIFT_2_END_TIME;
        } else if (matchTime > SHIFT_3_END_TIME) {
            return matchTime - SHIFT_3_END_TIME;
        } else if (matchTime > SHIFT_4_END_TIME) {
            return matchTime - SHIFT_4_END_TIME;
        }

        return Math.max(0.0, matchTime);
    }

    public void update() {
        updateGameData();
        updateAlliance();
    }
}
