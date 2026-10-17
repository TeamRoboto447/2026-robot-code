package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.utils.HubSchedule.Phase;
import org.junit.jupiter.api.Test;

class HubScheduleTest {
    private static final double EPS = 1e-9;

    @Test
    void phaseBoundariesBelongToEarlierPhase() {
        assertEquals(Phase.TRANSITION, HubSchedule.phaseAt(140.0));
        assertEquals(Phase.TRANSITION, HubSchedule.phaseAt(130.0));
        assertEquals(Phase.SHIFT_1, HubSchedule.phaseAt(129.99));
        assertEquals(Phase.SHIFT_1, HubSchedule.phaseAt(105.0));
        assertEquals(Phase.SHIFT_2, HubSchedule.phaseAt(104.99));
        assertEquals(Phase.SHIFT_2, HubSchedule.phaseAt(80.0));
        assertEquals(Phase.SHIFT_3, HubSchedule.phaseAt(79.99));
        assertEquals(Phase.SHIFT_3, HubSchedule.phaseAt(55.0));
        assertEquals(Phase.SHIFT_4, HubSchedule.phaseAt(54.99));
        assertEquals(Phase.SHIFT_4, HubSchedule.phaseAt(30.0));
        assertEquals(Phase.ENDGAME, HubSchedule.phaseAt(29.99));
        assertEquals(Phase.ENDGAME, HubSchedule.phaseAt(0.0));
    }

    @Test
    void unusableTimesAreNone() {
        assertEquals(Phase.NONE, HubSchedule.phaseAt(Double.NaN));
        assertEquals(Phase.NONE, HubSchedule.phaseAt(-1.0));
        assertEquals(Phase.NONE, HubSchedule.phaseAt(140.01));
        assertEquals(Phase.NONE, HubSchedule.phaseAt(Double.POSITIVE_INFINITY));
    }

    @Test
    void transitionEndgameAutoAndNoneAreAlwaysActive() {
        for (Phase p : new Phase[] {Phase.NONE, Phase.AUTO, Phase.TRANSITION, Phase.ENDGAME}) {
            assertTrue(HubSchedule.isActive(p, true), p + " (inactive first)");
            assertTrue(HubSchedule.isActive(p, false), p + " (active first)");
        }
    }

    @Test
    void shiftsAlternateAndAreExactComplementsBetweenAlliances() {
        // Alliance inactive first: inactive in shifts 1 and 3, active in 2 and 4.
        assertFalse(HubSchedule.isActive(Phase.SHIFT_1, true));
        assertTrue(HubSchedule.isActive(Phase.SHIFT_2, true));
        assertFalse(HubSchedule.isActive(Phase.SHIFT_3, true));
        assertTrue(HubSchedule.isActive(Phase.SHIFT_4, true));

        for (Phase p : new Phase[] {Phase.SHIFT_1, Phase.SHIFT_2, Phase.SHIFT_3, Phase.SHIFT_4}) {
            assertNotEquals(HubSchedule.isActive(p, true), HubSchedule.isActive(p, false), p.toString());
        }
    }

    @Test
    void secondsUntilActiveIsZeroWhenActive() {
        assertEquals(0.0, HubSchedule.secondsUntilActive(Phase.SHIFT_2, 90.0, true), EPS);
        assertEquals(0.0, HubSchedule.secondsUntilActive(Phase.TRANSITION, 135.0, true), EPS);
        assertEquals(0.0, HubSchedule.secondsUntilActive(Phase.ENDGAME, 10.0, false), EPS);
    }

    @Test
    void secondsUntilActiveCountsToEndOfInactiveShift() {
        // Inactive first alliance during shift 1 (130 -> 105): at 120 s remaining there are 15 s left.
        assertEquals(15.0, HubSchedule.secondsUntilActive(Phase.SHIFT_1, 120.0, true), EPS);
        assertEquals(25.0, HubSchedule.secondsUntilActive(Phase.SHIFT_1, 130.0, true), EPS);
        assertEquals(0.0, HubSchedule.secondsUntilActive(Phase.SHIFT_1, 105.0, true), EPS);
        // Other alliance is inactive in shift 2 (105 -> 80).
        assertEquals(5.0, HubSchedule.secondsUntilActive(Phase.SHIFT_2, 85.0, false), EPS);
        // Shift 3 for inactive-first alliance (80 -> 55), shift 4 for the other (55 -> 30).
        assertEquals(10.0, HubSchedule.secondsUntilActive(Phase.SHIFT_3, 65.0, true), EPS);
        assertEquals(20.0, HubSchedule.secondsUntilActive(Phase.SHIFT_4, 50.0, false), EPS);
    }

    @Test
    void secondsRemainingInPhase() {
        assertEquals(0.0, HubSchedule.secondsRemainingInPhase(Phase.NONE, 100.0), EPS);
        assertEquals(0.0, HubSchedule.secondsRemainingInPhase(Phase.AUTO, 10.0), EPS);
        assertEquals(10.0, HubSchedule.secondsRemainingInPhase(Phase.TRANSITION, 140.0), EPS);
        assertEquals(30.0, HubSchedule.secondsRemainingInPhase(Phase.ENDGAME, 30.0), EPS);
        assertEquals(0.0, HubSchedule.secondsRemainingInPhase(Phase.ENDGAME, 0.0), EPS);
    }

    @Test
    void phaseOrdinalsIncreaseThroughTheMatch() {
        assertTrue(Phase.AUTO.ordinal() < Phase.TRANSITION.ordinal());
        assertTrue(Phase.TRANSITION.ordinal() < Phase.SHIFT_1.ordinal());
        assertTrue(Phase.SHIFT_3.ordinal() < Phase.SHIFT_4.ordinal());
        assertTrue(Phase.SHIFT_4.ordinal() < Phase.ENDGAME.ordinal());
    }
}
