package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AppliedGainsTest {
    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void pauseTime() {
        SimHooks.pauseTiming();
        SimHooks.restartTiming();
    }

    @AfterEach
    void resumeTime() {
        SimHooks.resumeTiming();
    }

    @Test
    void unchangedValuesAreSkipped() {
        var g = new AppliedGains(1.0, 2.0);
        assertFalse(g.shouldApply(false, 1.0, 2.0));
    }

    @Test
    void changedValuesAreApplied() {
        var g = new AppliedGains(1.0, 2.0);
        assertTrue(g.shouldApply(false, 1.0, 2.5));
    }

    @Test
    void forceAlwaysApplies() {
        var g = new AppliedGains(1.0, 2.0);
        assertTrue(g.shouldApply(true, 1.0, 2.0));
    }

    @Test
    void successfulApplyBecomesTheNewBaseline() {
        var g = new AppliedGains(1.0);
        assertTrue(g.shouldApply(false, 3.0));
        g.record(true, 3.0);
        assertFalse(g.shouldApply(false, 3.0));
        assertTrue(g.shouldApply(false, 1.0)); // back to the old value is a change again
    }

    @Test
    void rejectedValuesAreNotRetriedUntilTheBackoffPasses() {
        var g = new AppliedGains(1.0);
        assertTrue(g.shouldApply(false, 3.0));
        g.record(false, 3.0);
        assertFalse(g.shouldApply(false, 3.0), "should back off right after a failure");
        SimHooks.stepTiming(2.0);
        assertFalse(g.shouldApply(false, 3.0), "still inside the backoff window");
        SimHooks.stepTiming(4.0);
        assertTrue(g.shouldApply(false, 3.0), "retry after the backoff");
    }

    @Test
    void differentValuesRetryImmediatelyAfterAFailure() {
        var g = new AppliedGains(1.0);
        g.record(false, 3.0);
        assertTrue(g.shouldApply(false, 4.0));
    }

    @Test
    void forceOverridesTheBackoff() {
        var g = new AppliedGains(1.0);
        g.record(false, 3.0);
        assertTrue(g.shouldApply(true, 3.0));
    }

    @Test
    void failedApplyDoesNotChangeTheBaseline() {
        var g = new AppliedGains(1.0);
        g.record(false, 3.0);
        assertFalse(g.shouldApply(false, 1.0)); // still what the motor has
    }
}
