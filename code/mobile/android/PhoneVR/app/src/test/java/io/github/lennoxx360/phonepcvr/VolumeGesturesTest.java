/* (C)2026 */
package io.github.lennoxx360.phonepcvr;

import static io.github.lennoxx360.phonepcvr.VolumeGestures.DOWN;
import static io.github.lennoxx360.phonepcvr.VolumeGestures.Gesture.*;
import static io.github.lennoxx360.phonepcvr.VolumeGestures.UP;
import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.junit.Test;

public class VolumeGesturesTest {
    private final List<VolumeGestures.Gesture> got = new ArrayList<>();
    private final VolumeGestures g = new VolumeGestures(got::add);

    /** Runs the timers the activity would post, in time order, up to t. */
    private long now = 0;

    private void at(long t) {
        for (long s = now + 1; s <= t; s++) g.onTimer(s);
        now = t;
    }

    private void down(int key, long t) {
        at(t);
        g.onDown(key, t);
    }

    private void up(int key, long t) {
        at(t);
        g.onUp(key, t);
    }

    private void expect(VolumeGestures.Gesture... want) {
        assertEquals(Arrays.asList(want), got);
    }

    @Test
    public void tapUp() {
        down(UP, 0);
        up(UP, 150);
        at(2000);
        expect(TAP_UP);
    }

    @Test
    public void tapDown() {
        down(DOWN, 0);
        up(DOWN, 300);
        expect(TAP_DOWN);
    }

    @Test
    public void holdFiresOnceAndReleaseDoesNothing() {
        down(UP, 0);
        at(1500);
        up(UP, 1500);
        expect(HOLD_UP);
    }

    @Test
    public void holdDown() {
        down(DOWN, 0);
        up(DOWN, 700);
        expect(HOLD_DOWN);
    }

    @Test
    public void bothTapIsOneGestureNotTwoTaps() {
        down(UP, 0);
        down(DOWN, 80);
        up(UP, 200);
        up(DOWN, 260);
        expect(TAP_BOTH);
    }

    @Test
    public void bothHold() {
        down(DOWN, 0);
        down(UP, 100);
        at(900);
        up(DOWN, 900);
        up(UP, 950);
        expect(HOLD_BOTH);
    }

    @Test
    public void lateSecondKeyIsIgnored() {
        // Second key after the chord window: still a single-key gesture, no extra action.
        down(UP, 0);
        down(DOWN, 300);
        up(DOWN, 350);
        up(UP, 400);
        expect(TAP_UP);
    }

    @Test
    public void bothHoldReleasedEarlyIsATap() {
        down(UP, 0);
        down(DOWN, 50);
        up(DOWN, 400); // let go of one before the hold time
        at(1000);
        up(UP, 1000);
        expect(TAP_BOTH);
    }

    @Test
    public void triggerQuickClick() {
        g.setTriggerMode(true);
        down(DOWN, 0);
        up(DOWN, 60);
        expect(TRIGGER_PRESS, TRIGGER_RELEASE);
    }

    @Test
    public void triggerHoldDragsAndNeverBecomesHold() {
        g.setTriggerMode(true);
        down(DOWN, 0);
        at(130);
        expect(TRIGGER_PRESS);
        at(2000);
        up(DOWN, 2000);
        expect(TRIGGER_PRESS, TRIGGER_RELEASE);
    }

    @Test
    public void triggerModeChordDoesNotClick() {
        g.setTriggerMode(true);
        down(DOWN, 0);
        down(UP, 90);
        up(DOWN, 200);
        up(UP, 220);
        expect(TAP_BOTH);
    }

    @Test
    public void triggerModeUpKeyStillWorks() {
        g.setTriggerMode(true);
        down(UP, 0);
        up(UP, 100);
        down(UP, 1000);
        up(UP, 1800);
        expect(TAP_UP, HOLD_UP);
    }

    @Test
    public void upPressedWhileTriggerHeldIsIgnored() {
        g.setTriggerMode(true);
        down(DOWN, 0);
        at(200); // trigger pulled
        down(UP, 300);
        up(UP, 400);
        up(DOWN, 500);
        expect(TRIGGER_PRESS, TRIGGER_RELEASE);
    }

    @Test
    public void leavingTriggerModeReleasesTheTrigger() {
        g.setTriggerMode(true);
        down(DOWN, 0);
        at(200);
        g.setTriggerMode(false);
        expect(TRIGGER_PRESS, TRIGGER_RELEASE);
    }

    @Test
    public void autoRepeatDownsAreIgnored() {
        down(UP, 0);
        down(UP, 50);
        down(UP, 100);
        up(UP, 150);
        expect(TAP_UP);
    }

    @Test
    public void cancelReleasesTrigger() {
        g.setTriggerMode(true);
        down(DOWN, 0);
        at(200);
        g.cancel();
        expect(TRIGGER_PRESS, TRIGGER_RELEASE);
    }
}
