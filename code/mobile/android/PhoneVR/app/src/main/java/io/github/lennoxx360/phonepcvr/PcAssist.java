/* (C)2026 */
package io.github.lennoxx360.phonepcvr;

import android.util.Log;
import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;

/**
 * Phone side of PC assist: the PC assist app (pc-assist/pc_assist.py) connects here, receives the
 * hand-camera frames (H.264 or JPEG) and sends MediaPipe's landmarks back. Everything after that
 * (3D placement, filtering, pinch, ALVR) stays on the phone, so the ALVR server is untouched.
 *
 * <p>Flow control: a frame is only started ({@link #beginFrame}) while fewer than {@link
 * #MAX_IN_FLIGHT} frames wait for a result, so a slow PC or network drops camera frames before
 * they're encoded instead of adding delay. Started frames are always sent, in order (an H.264
 * stream can't lose packets); an unsent JPEG is still replaced by a newer one.
 */
public final class PcAssist {
    public interface Listener {
        /** Receiver thread. */
        void onResult(PcAssistProtocol.Result r);

        /** Server thread: a PC app just connected (an H.264 stream must restart on a keyframe). */
        void onConnected();
    }

    private static final String TAG = "PVR-Assist";
    static final int MAX_IN_FLIGHT = 2;
    static final long RESULT_TIMEOUT_MS = 500;
    // Connected but no answer for this long while frames wait: the PC app is stuck. Hang up so
    // the phone tracks hands itself again (and the PC app reconnects).
    static final long STALL_MS = 1500;
    private long lastProgressMs = 0; // guarded by `this`

    private final Listener listener;
    private volatile boolean running = false;
    private Thread serverThread, beaconThread, senderThread;
    private ServerSocket server;
    private volatile Socket client;
    private volatile boolean ready = false;
    private volatile String peer = "";

    private static final class Outgoing {
        int id, width, height, numHands, codec, flags, length;
        byte[] data;
    }

    // Guarded by `this`.
    private final ArrayDeque<Outgoing> queue = new ArrayDeque<>();
    private int nextId = 1;
    private final int[] flightIds = new int[MAX_IN_FLIGHT];
    private final long[] flightMs = new long[MAX_IN_FLIGHT];
    private int flightCount = 0;

    // Stats (any thread, approximate).
    volatile long bytesSent = 0;
    volatile int framesSent = 0, results = 0, skipped = 0, timeouts = 0;

    public PcAssist(Listener listener) {
        this.listener = listener;
    }

    public synchronized void start() {
        if (running) return;
        running = true;
        serverThread = new Thread(this::serverLoop, "PVR-Assist-server");
        beaconThread = new Thread(this::beaconLoop, "PVR-Assist-beacon");
        senderThread = new Thread(this::senderLoop, "PVR-Assist-send");
        serverThread.start();
        beaconThread.start();
        senderThread.start();
        Log.i(TAG, "PC assist on: waiting for the PC app on port " + PcAssistProtocol.PORT);
    }

    public void stop() {
        synchronized (this) {
            if (!running) return;
            running = false;
            notifyAll();
        }
        closeQuietly(server);
        closeClient();
        if (beaconThread != null) beaconThread.interrupt();
        Log.i(TAG, "PC assist off");
    }

    /** True while a PC assist app is connected and has said hello. */
    public boolean isReady() {
        return running && ready;
    }

    public String peer() {
        return peer;
    }

    /**
     * Reserves an id for a frame about to be encoded, or returns -1 if not connected or too many
     * frames wait for a result (skip this camera frame).
     */
    public synchronized int beginFrame() {
        if (!isReady()) return -1;
        long now = System.currentTimeMillis();
        if (flightCount > 0 && now - lastProgressMs > STALL_MS) {
            Log.w(
                    TAG,
                    "PC assist stalled (no answer for "
                            + (now - lastProgressMs)
                            + " ms): phone tracks hands itself, waiting for the PC to reconnect");
            Socket s = client;
            client = null;
            ready = false;
            flightCount = 0;
            queue.clear();
            new Thread(() -> closeQuietly(s), "PVR-Assist-close").start();
            return -1;
        }
        if (flightCount == 0) lastProgressMs = now; // nothing outstanding: nothing to wait for
        expireFlights(now);
        if (flightCount >= MAX_IN_FLIGHT) return -1;
        int id = nextId++;
        flightIds[flightCount] = id;
        flightMs[flightCount] = System.currentTimeMillis();
        flightCount++;
        return id;
    }

    /** A begun frame that won't be sent after all. */
    public synchronized void cancelFrame(int id) {
        dropFlight(id);
    }

    /** Queues a begun frame (data is copied). */
    public synchronized void sendFrame(
            int id,
            int codec,
            int flags,
            int width,
            int height,
            int numHands,
            byte[] data,
            int offset,
            int length) {
        if (!isReady()) return;
        if (codec == PcAssistProtocol.CODEC_JPEG) {
            // An older JPEG still waiting to go out is stale now: replace it.
            for (java.util.Iterator<Outgoing> it = queue.iterator(); it.hasNext(); ) {
                Outgoing o = it.next();
                if (o.codec == PcAssistProtocol.CODEC_JPEG) {
                    dropFlight(o.id);
                    it.remove();
                }
            }
        }
        Outgoing o = new Outgoing();
        o.id = id;
        o.codec = codec;
        o.flags = flags;
        o.width = width;
        o.height = height;
        o.numHands = numHands;
        o.data = new byte[length];
        System.arraycopy(data, offset, o.data, 0, length);
        o.length = length;
        queue.addLast(o);
        notifyAll();
    }

    private void expireFlights(long nowMs) {
        for (int i = flightCount - 1; i >= 0; i--) {
            if (nowMs - flightMs[i] > RESULT_TIMEOUT_MS) {
                removeFlightAt(i);
                timeouts++;
            }
        }
    }

    private synchronized void dropFlight(int id) {
        for (int i = 0; i < flightCount; i++) {
            if (flightIds[i] == id) {
                removeFlightAt(i);
                return;
            }
        }
    }

    private void removeFlightAt(int i) {
        for (int k = i; k < flightCount - 1; k++) {
            flightIds[k] = flightIds[k + 1];
            flightMs[k] = flightMs[k + 1];
        }
        flightCount--;
    }

    private void serverLoop() {
        try {
            server = new ServerSocket();
            server.setReuseAddress(true);
            server.bind(new InetSocketAddress(PcAssistProtocol.PORT));
        } catch (IOException e) {
            Log.w(TAG, "can't listen on port " + PcAssistProtocol.PORT, e);
            return;
        }
        while (running) {
            Socket s;
            try {
                s = server.accept();
            } catch (IOException e) {
                if (running) Log.w(TAG, "accept failed", e);
                break;
            }
            closeClient(); // one PC at a time: the newest connection wins
            try {
                s.setTcpNoDelay(true);
                s.setSoTimeout(5000);
                OutputStream out = s.getOutputStream();
                out.write(PcAssistProtocol.hello());
                out.flush();
                DataInputStream in =
                        new DataInputStream(new BufferedInputStream(s.getInputStream()));
                int version = PcAssistProtocol.readHello(in);
                if (version != PcAssistProtocol.VERSION) {
                    Log.w(
                            TAG,
                            "PC assist app version "
                                    + version
                                    + ", phone wants "
                                    + PcAssistProtocol.VERSION
                                    + ": update pc_assist.py");
                    s.close();
                    continue;
                }
                s.setSoTimeout(0);
                synchronized (this) {
                    client = s;
                    flightCount = 0;
                    queue.clear();
                    lastProgressMs = System.currentTimeMillis();
                }
                peer = s.getInetAddress().getHostAddress();
                listener.onConnected();
                ready = true;
                Log.i(TAG, "PC assist connected: " + peer);
                Thread reader = new Thread(() -> readLoop(s, in), "PVR-Assist-recv");
                reader.start();
            } catch (IOException e) {
                Log.w(TAG, "PC assist handshake failed", e);
                closeQuietly(s);
            }
        }
        closeQuietly(server);
    }

    private void readLoop(Socket s, DataInputStream in) {
        try {
            while (running && client == s) {
                PcAssistProtocol.Result r = PcAssistProtocol.readResult(in);
                synchronized (this) {
                    lastProgressMs = System.currentTimeMillis();
                }
                dropFlight(r.frameId);
                if (r.skipped) {
                    skipped++;
                    continue;
                }
                results++;
                listener.onResult(r);
            }
        } catch (IOException e) {
            if (running && client == s) Log.i(TAG, "PC assist disconnected: " + e.getMessage());
        }
        synchronized (this) {
            if (client == s) {
                ready = false;
                client = null;
                flightCount = 0;
                queue.clear();
            }
        }
        closeQuietly(s);
    }

    private void senderLoop() {
        while (true) {
            Outgoing o;
            Socket s;
            synchronized (this) {
                while (running && (queue.isEmpty() || client == null)) {
                    try {
                        wait();
                    } catch (InterruptedException e) {
                        return;
                    }
                }
                if (!running) return;
                o = queue.pollFirst();
                s = client;
            }
            try {
                OutputStream out = s.getOutputStream();
                out.write(
                        PcAssistProtocol.frameHeader(
                                o.id, o.width, o.height, o.numHands, o.codec, o.flags, o.length));
                out.write(o.data, 0, o.length);
                out.flush();
                bytesSent += PcAssistProtocol.FRAME_HEADER_BYTES + o.length;
                framesSent++;
            } catch (IOException e) {
                Log.i(TAG, "PC assist send failed: " + e.getMessage());
                closeClient();
            }
        }
    }

    private void beaconLoop() {
        byte[] msg = PcAssistProtocol.beacon().getBytes(StandardCharsets.US_ASCII);
        try (DatagramSocket sock = new DatagramSocket()) {
            sock.setBroadcast(true);
            InetAddress all = InetAddress.getByName("255.255.255.255");
            while (running) {
                if (!ready) {
                    try {
                        sock.send(
                                new DatagramPacket(
                                        msg, msg.length, all, PcAssistProtocol.BEACON_PORT));
                    } catch (IOException e) {
                        // No network yet: try again next second.
                    }
                }
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    break;
                }
            }
        } catch (IOException e) {
            Log.w(TAG, "PC assist beacon failed", e);
        }
    }

    private void closeClient() {
        Socket s;
        synchronized (this) {
            s = client;
            client = null;
            ready = false;
            flightCount = 0;
            queue.clear();
        }
        closeQuietly(s);
    }

    private static void closeQuietly(java.io.Closeable c) {
        if (c == null) return;
        try {
            c.close();
        } catch (IOException ignored) {
        }
    }
}
