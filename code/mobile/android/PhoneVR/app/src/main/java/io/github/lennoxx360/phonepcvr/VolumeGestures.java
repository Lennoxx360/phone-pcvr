/* (C)2026 */
package io.github.lennoxx360.phonepcvr;

/**
 * Turns the two volume keys into non-overlapping gestures: tap / hold of either key, and tap / hold
 * of both together. Nothing fires on the way to another gesture:
 *
 * <ul>
 *   <li>"Both" only counts when the second key goes down within {@link #CHORD_MS} of the first. A
 *       key pressed later is ignored for the rest of the gesture.
 *   <li>A tap is decided when every key is released; a hold fires once at {@link #LONG_MS}, and the
 *       release afterwards does nothing.
 *   <li>Trigger mode (gaze pointer): Vol- is a trigger. It is pulled {@link #CHORD_MS} after going
 *       down (so a two-key press never clicks) and released with the key; a release before that is
 *       a quick click.
 * </ul>
 *
 * <p>Pure logic with injected times (KeyEvent / uptimeMillis), so it is unit-tested on the host.
 * The caller calls {@link #onTimer} at down + CHORD_MS and down + LONG_MS.
 */
final class VolumeGestures {
    static final long CHORD_MS = 120;
    static final long LONG_MS = 600;

    static final int UP = 0;
    static final int DOWN = 1;

    enum Gesture {
        TAP_UP,
        TAP_DOWN,
        HOLD_UP,
        HOLD_DOWN,
        TAP_BOTH,
        HOLD_BOTH,
        TRIGGER_PRESS,
        TRIGGER_RELEASE
    }

    interface Listener {
        void onGesture(Gesture g);
    }

    private final Listener listener;
    private boolean triggerMode = false;

    private final boolean[] pressed = new boolean[2];
    private final boolean[] inGesture = new boolean[2];
    private boolean active = false;
    private long startMs;
    private boolean longFired;
    private boolean triggerPending;
    private boolean triggerHeld;

    VolumeGestures(Listener listener) {
        this.listener = listener;
    }

    /** Gaze pointer on and not adjusting the fit: Vol- becomes the trigger. */
    void setTriggerMode(boolean on) {
        if (!on && triggerHeld) {
            triggerHeld = false;
            listener.onGesture(Gesture.TRIGGER_RELEASE);
        }
        triggerMode = on;
    }

    void onDown(int key, long t) {
        if (pressed[key]) return; // auto-repeat
        pressed[key] = true;
        if (!active) {
            active = true;
            startMs = t;
            longFired = false;
            inGesture[UP] = inGesture[DOWN] = false;
            inGesture[key] = true;
            triggerPending = triggerMode && key == DOWN;
            return;
        }
        // Second key: a chord only if it's quick and nothing has fired yet.
        if (t - startMs <= CHORD_MS && !longFired && !triggerHeld) {
            inGesture[key] = true;
            triggerPending = false;
        }
    }

    void onUp(int key, long t) {
        if (!pressed[key]) return;
        pressed[key] = false;
        if (!active) return;

        if (key == DOWN && triggerHeld) {
            triggerHeld = false;
            listener.onGesture(Gesture.TRIGGER_RELEASE);
        }
        if (pressed[UP] || pressed[DOWN]) return; // wait until every key is up

        boolean both = inGesture[UP] && inGesture[DOWN];
        if (triggerPending) {
            // Released before the trigger was pulled: a quick click.
            listener.onGesture(Gesture.TRIGGER_PRESS);
            listener.onGesture(Gesture.TRIGGER_RELEASE);
        } else if (!longFired && !wasTrigger) {
            if (both) listener.onGesture(Gesture.TAP_BOTH);
            else if (inGesture[UP]) listener.onGesture(Gesture.TAP_UP);
            else listener.onGesture(Gesture.TAP_DOWN);
        }
        active = false;
        triggerPending = false;
        wasTrigger = false;
    }

    private boolean wasTrigger = false;

    void onTimer(long t) {
        if (!active) return;
        if (triggerPending && t - startMs >= CHORD_MS) {
            triggerPending = false;
            if (pressed[DOWN] && !inGesture[UP]) {
                triggerHeld = true;
                wasTrigger = true;
                listener.onGesture(Gesture.TRIGGER_PRESS);
            }
        }
        if (!longFired && !wasTrigger && t - startMs >= LONG_MS) {
            boolean both = inGesture[UP] && inGesture[DOWN];
            boolean stillHeld = both ? pressed[UP] && pressed[DOWN] : pressed[UP] || pressed[DOWN];
            if (stillHeld) {
                longFired = true;
                if (both) listener.onGesture(Gesture.HOLD_BOTH);
                else if (inGesture[UP]) listener.onGesture(Gesture.HOLD_UP);
                else listener.onGesture(Gesture.HOLD_DOWN);
            }
        }
    }

    /** Leaving the screen: release a held trigger and forget the gesture. */
    void cancel() {
        if (triggerHeld) {
            triggerHeld = false;
            listener.onGesture(Gesture.TRIGGER_RELEASE);
        }
        pressed[UP] = pressed[DOWN] = false;
        active = false;
        triggerPending = false;
        wasTrigger = false;
    }
}
