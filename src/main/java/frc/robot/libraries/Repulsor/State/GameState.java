/*
 * Copyright (C) 2026 Paul Hodges
 *
 * This file is part of Repulsor.
 *
 * Repulsor is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Repulsor is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Repulsor. If not, see https://www.gnu.org/licenses/.
 */
package frc.robot.libraries.Repulsor.State;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

public class GameState extends StaticState {
  private final double TELEOP_GAME_LENGTH = 140.0;
  private final double AUTONOMOUS_PERIOD_LENGTH = 20.0;
  private final double TRANSITION_END_TIME = 130.0;
  private final double SHIFT_1_END_TIME = 105.0;
  private final double SHIFT_2_END_TIME = 80.0;
  private final double SHIFT_3_END_TIME = 55.0;
  private final double SHIFT_4_END_TIME = 30.0;

  private Optional<DriverStation.Alliance> alliance = Optional.empty();
  private Optional<DriverStation.Alliance> inactiveFirst = Optional.empty();

  private Alert noAllianceAlert = new Alert("No Alliance Read", AlertType.kWarning);
  private Alert noGameDataAlert = new Alert("No Game Data Read", AlertType.kWarning);

  private void updateAlliance() {
    Optional<DriverStation.Alliance> readAlliance = DriverStation.getAlliance();
    if (alliance.isEmpty() && readAlliance.isPresent()) {
      alliance = readAlliance;
      // Alliance successfully read — clear the alert only if it was active.
      if (noAllianceAlert.get()) noAllianceAlert.set(false);
    } else if (alliance.isEmpty()) {
      // Still no alliance — raise the alert only if it wasn't already raised.
      if (!noAllianceAlert.get()) noAllianceAlert.set(true);
    }
  }

  private void updateGameData() {
    String gameData = DriverStation.getGameSpecificMessage();
    boolean validGameData = gameData.length() > 0
        && (gameData.charAt(0) == 'B' || gameData.charAt(0) == 'R');

    // Only call Alert.set() when the state actually changes.
    // Alert internally modifies a TreeSet on every set() call regardless of
    // whether the value changed; doing so while SmartDashboard is iterating
    // that same set causes a ConcurrentModificationException.
    if (noGameDataAlert.get() == validGameData) {
      noGameDataAlert.set(!validGameData);
    }

    if (inactiveFirst.isEmpty() && validGameData) {
      inactiveFirst = Optional.of(
          (gameData.charAt(0) == 'B') ? DriverStation.Alliance.Blue : DriverStation.Alliance.Red);
      
      System.out.print("Inactive first: ");
      System.out.println(inactiveFirst.get());
    }
  }


private Boolean isCountingDown = null; // We use a Boolean (capital B) here so that we can set the value to null to represent an 'unknown' state
private double lastMatchTime = -1;

private double getMatchTime() {
    double gameTime = DriverStation.getMatchTime();
    // Auto-detect direction on first tick of a new period
    if (isCountingDown == null && lastMatchTime >= 0) {
        isCountingDown = gameTime < lastMatchTime;
    }
    lastMatchTime = gameTime;

    if (Boolean.TRUE.equals(isCountingDown)) { // Because we are using a Boolean object instead of a boolean primative, we have to use a null-safe comparison
        return gameTime; // Practice mode: already time remaining
    }
    return TELEOP_GAME_LENGTH - gameTime; // Plain DS: convert elapsed → remaining
}

  public boolean isHubActive() {
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }

    // Fail-safe default: keep hub active when not teleop enabled.
    if (!DriverStation.isTeleopEnabled()) {
      return true;
    }

    if (!(inactiveFirst.isPresent() && alliance.isPresent())) {
      return true;
    }

    double matchTime = getMatchTime();

    // Transition and endgame are always active.
    if (matchTime > TRANSITION_END_TIME || matchTime <= SHIFT_4_END_TIME) {
      return true;
    }

    boolean shift1Active = inactiveFirst.get() != alliance.get();
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
    double previousRawMatchTime = lastMatchTime;
    double matchTime = getMatchTime();
    double currentRawMatchTime = lastMatchTime;

    boolean normalizedTimeCountingDown = true;
    if (previousRawMatchTime >= 0.0) {
      if (Boolean.TRUE.equals(isCountingDown)) {
        normalizedTimeCountingDown = true;
      } else {
        normalizedTimeCountingDown = currentRawMatchTime > previousRawMatchTime;
      }
    }

    if (normalizedTimeCountingDown) {
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

    if (matchTime <= SHIFT_4_END_TIME) {
      return SHIFT_4_END_TIME - matchTime;
    } else if (matchTime <= SHIFT_3_END_TIME) {
      return SHIFT_3_END_TIME - matchTime;
    } else if (matchTime <= SHIFT_2_END_TIME) {
      return SHIFT_2_END_TIME - matchTime;
    } else if (matchTime <= SHIFT_1_END_TIME) {
      return SHIFT_1_END_TIME - matchTime;
    } else if (matchTime <= TRANSITION_END_TIME) {
      return TRANSITION_END_TIME - matchTime;
    }
    return Math.max(0.0, TELEOP_GAME_LENGTH - matchTime);
  }

  @Override
  public void update(double dt) {
    updateGameData();
    updateAlliance();
  }
}
