package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.utils.HubSchedule.Phase;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Exercises the DriverStation-reading wrapper (fail-open + game-data parsing) using DriverStationSim. */
class GameStateTest {
    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void resetDs() {
        DriverStationSim.resetData();
        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setGameSpecificMessage("");
        DriverStationSim.setMatchTime(140.0);
        DriverStationSim.notifyNewData();
    }

    private GameState state(String message, double matchTime) {
        DriverStationSim.setGameSpecificMessage(message);
        DriverStationSim.setMatchTime(matchTime);
        DriverStationSim.notifyNewData();
        GameState gs = new GameState();
        gs.update();
        return gs;
    }

    @Test
    void noGameDataFailsOpenAndIsFlaggedInvalid() {
        GameState gs = state("", 120.0);
        assertFalse(gs.isDataValid());
        assertTrue(gs.isHubActive());
        assertEquals(0.0, gs.getSecondsUntilHubActive(), 1e-9);
    }

    @Test
    void garbageGameDataIsInvalid() {
        GameState gs = state("X", 120.0);
        assertFalse(gs.isDataValid());
        assertTrue(gs.isHubActive());
    }

    @Test
    void ourAllianceInactiveFirstIsInactiveInShiftOne() {
        GameState gs = state("R", 120.0); // we are Red1
        assertTrue(gs.isDataValid());
        assertEquals(Phase.SHIFT_1, gs.getPhase());
        assertFalse(gs.isHubActive());
        assertEquals(15.0, gs.getSecondsUntilHubActive(), 1e-9);
    }

    @Test
    void otherAllianceInactiveFirstMeansWeAreActiveInShiftOne() {
        GameState gs = state("B", 120.0);
        assertTrue(gs.isDataValid());
        assertTrue(gs.isHubActive());
        assertEquals(0.0, gs.getSecondsUntilHubActive(), 1e-9);
    }

    @Test
    void lowercaseGameDataIsAccepted() {
        assertFalse(state("r", 120.0).isHubActive());
    }

    @Test
    void autoAndDisabledAreAlwaysActive() {
        DriverStationSim.setAutonomous(true);
        assertTrue(state("R", 10.0).isHubActive());
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(false);
        assertTrue(state("R", 120.0).isHubActive());
        assertEquals(Phase.NONE, state("R", 120.0).getPhase());
    }

    @Test
    void unusableMatchTimeFailsOpen() {
        assertTrue(state("R", -1.0).isHubActive());
        assertTrue(state("R", Double.NaN).isHubActive());
    }

    @Test
    void blueAllianceIsComplementOfRed() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue2);
        GameState gs = state("R", 120.0); // Red inactive first -> Blue active in shift 1
        assertTrue(gs.isHubActive());
        assertEquals(Alliance.Red, gs.getInactiveFirstAlliance().get());
    }
}
