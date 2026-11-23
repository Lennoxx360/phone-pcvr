/* (C)2026 */
package io.github.lennoxx360.phonepcvr;

import android.util.Log;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;

/**
 * Manual network mode: sends ALVR's discovery packet straight to a typed-in PC IP, once a second.
 * ALVR itself only broadcasts to 255.255.255.255, which some networks (guest Wi-Fi, some hotspots,
 * VPNs, routers with client isolation) drop. The packet layout matches ALVR v20's
 * client_core/src/sockets.rs: "ALVR" padded to 16 bytes, protocol id (u64 LE), hostname, 56 bytes.
 */
final class ManualAnnouncer {
    private static final String TAG = "PVR-Net";
    private static final int CONTROL_PORT = 9943;
    private static final int PACKET_SIZE = 56;

    /**
     * ALVR's protocol_id_u64() for v20.x: Rust's DefaultHasher (SipHash-1-3, zero keys) of the
     * string "20". Must change together with the pinned ALVR version.
     */
    private static final long PROTOCOL_ID_V20 = 1686174404061257063L;

    private Thread thread;
    private volatile boolean running;

    static boolean isValidIp(String ip) {
        if (ip == null) return false;
        String[] parts = ip.trim().split("\\.");
        if (parts.length != 4) return false;
        for (String p : parts) {
            try {
                int v = Integer.parseInt(p);
                if (v < 0 || v > 255 || p.isEmpty() || p.length() > 3) return false;
            } catch (NumberFormatException e) {
                return false;
            }
        }
        return true;
    }

    static byte[] buildPacket(String hostname) {
        ByteBuffer buf = ByteBuffer.allocate(PACKET_SIZE).order(ByteOrder.LITTLE_ENDIAN);
        buf.put("ALVR".getBytes(StandardCharsets.US_ASCII));
        buf.position(16);
        buf.putLong(PROTOCOL_ID_V20);
        byte[] name = hostname.getBytes(StandardCharsets.UTF_8);
        buf.put(name, 0, Math.min(name.length, PACKET_SIZE - 24));
        return buf.array();
    }

    synchronized void start(String pcIp, String hostname) {
        stop();
        if (!isValidIp(pcIp) || hostname == null || hostname.isEmpty()) {
            Log.w(TAG, "Manual mode not started: ip=" + pcIp + " hostname=" + hostname);
            return;
        }
        final byte[] packet = buildPacket(hostname);
        final String ip = pcIp.trim();
        running = true;
        thread =
                new Thread(
                        () -> {
                            Log.i(TAG, "Manual mode: announcing " + hostname + " to " + ip);
                            try (DatagramSocket socket = new DatagramSocket()) {
                                InetAddress addr = InetAddress.getByName(ip);
                                DatagramPacket p =
                                        new DatagramPacket(
                                                packet, packet.length, addr, CONTROL_PORT);
                                while (running) {
                                    try {
                                        socket.send(p);
                                    } catch (Exception e) {
                                        Log.w(TAG, "Announce to " + ip + " failed: " + e);
                                    }
                                    Thread.sleep(1000);
                                }
                            } catch (InterruptedException e) {
                                // stopping
                            } catch (Exception e) {
                                Log.w(TAG, "Manual announcer stopped", e);
                            }
                        },
                        "pcvr-announce");
        thread.setDaemon(true);
        thread.start();
    }

    synchronized void stop() {
        running = false;
        if (thread != null) {
            thread.interrupt();
            thread = null;
        }
    }
}
