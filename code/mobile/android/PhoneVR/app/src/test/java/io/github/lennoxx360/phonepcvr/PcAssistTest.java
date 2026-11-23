/* (C)2026 */
package io.github.lennoxx360.phonepcvr;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.DataInputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.Test;

/** PcAssist against a fake PC app on localhost (real sockets). */
public class PcAssistTest {
    private static PcAssist start(AtomicInteger connects) {
        PcAssist pa =
                new PcAssist(
                        new PcAssist.Listener() {
                            @Override
                            public void onResult(PcAssistProtocol.Result r) {}

                            @Override
                            public void onConnected() {
                                connects.incrementAndGet();
                            }
                        });
        pa.start();
        return pa;
    }

    private static Socket connectFakePc() throws Exception {
        for (int i = 0; i < 50; i++) {
            try {
                Socket s = new Socket("127.0.0.1", PcAssistProtocol.PORT);
                OutputStream out = s.getOutputStream();
                out.write(new byte[] {'H', (byte) PcAssistProtocol.VERSION});
                out.flush();
                InputStream in = s.getInputStream();
                assertEquals('P', in.read());
                assertEquals(PcAssistProtocol.VERSION, in.read());
                return s;
            } catch (java.net.ConnectException e) {
                Thread.sleep(50); // server not listening yet
            }
        }
        throw new AssertionError("phone side never listened");
    }

    private static void waitReady(PcAssist pa, boolean want) throws Exception {
        for (int i = 0; i < 100 && pa.isReady() != want; i++) Thread.sleep(20);
        assertEquals(want, pa.isReady());
    }

    @Test
    public void silentPcIsDroppedSoThePhoneTracksHandsItself() throws Exception {
        AtomicInteger connects = new AtomicInteger();
        PcAssist pa = start(connects);
        try (Socket pc = connectFakePc()) {
            waitReady(pa, true);
            assertEquals(1, connects.get());
            // Two frames go out; the "PC" reads them but never answers.
            byte[] data = new byte[100];
            int a = pa.beginFrame(), b = pa.beginFrame();
            assertTrue(a > 0 && b > 0);
            assertEquals("in-flight limit", -1, pa.beginFrame());
            pa.sendFrame(a, PcAssistProtocol.CODEC_H264, 1, 960, 720, 1, data, 0, data.length);
            pa.sendFrame(b, PcAssistProtocol.CODEC_H264, 0, 960, 720, 1, data, 0, data.length);
            DataInputStream in = new DataInputStream(pc.getInputStream());
            byte[] got = new byte[2 * (PcAssistProtocol.FRAME_HEADER_BYTES + data.length)];
            in.readFully(got); // both frames arrived, in order
            assertEquals('F', got[0]);
            Thread.sleep(PcAssist.STALL_MS + 200);
            assertEquals("stalled: frame refused", -1, pa.beginFrame());
            waitReady(pa, false);
            assertEquals("phone hung up on the silent PC", -1, in.read());
        }
        // The PC app reconnects and is used again.
        try (Socket pc2 = connectFakePc()) {
            waitReady(pa, true);
            assertEquals(2, connects.get());
            assertTrue(pa.beginFrame() > 0);
        } finally {
            pa.stop();
        }
    }

    @Test
    public void answeringPcIsKept() throws Exception {
        AtomicInteger connects = new AtomicInteger();
        PcAssist pa = start(connects);
        try (Socket pc = connectFakePc()) {
            waitReady(pa, true);
            DataInputStream in = new DataInputStream(pc.getInputStream());
            OutputStream out = pc.getOutputStream();
            byte[] data = new byte[10];
            long end = System.currentTimeMillis() + PcAssist.STALL_MS * 2;
            int answered = 0;
            while (System.currentTimeMillis() < end) {
                int id = pa.beginFrame();
                if (id < 0) {
                    Thread.sleep(5);
                    continue;
                }
                pa.sendFrame(id, PcAssistProtocol.CODEC_JPEG, 0, 960, 720, 1, data, 0, data.length);
                byte[] frame = new byte[PcAssistProtocol.FRAME_HEADER_BYTES + data.length];
                in.readFully(frame);
                int fid =
                        java.nio.ByteBuffer.wrap(frame, 1, 4)
                                .order(java.nio.ByteOrder.LITTLE_ENDIAN)
                                .getInt();
                PcAssistProtocol.Result r = new PcAssistProtocol.Result();
                r.frameId = fid;
                out.write(PcAssistProtocol.encodeResult(r));
                out.flush();
                answered++;
                Thread.sleep(33);
            }
            assertTrue("frames kept flowing: " + answered, answered > 30);
            assertTrue(pa.isReady());
            assertFalse(connects.get() > 1);
        } finally {
            pa.stop();
        }
    }
}
