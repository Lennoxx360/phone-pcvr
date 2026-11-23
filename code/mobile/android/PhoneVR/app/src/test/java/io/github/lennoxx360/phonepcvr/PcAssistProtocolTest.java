/* (C)2026 */
package io.github.lennoxx360.phonepcvr;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.ByteArrayInputStream;
import java.io.DataInputStream;
import org.junit.Test;

public class PcAssistProtocolTest {
    private static String hex(byte[] b) {
        StringBuilder sb = new StringBuilder();
        for (byte x : b) sb.append(String.format("%02x", x & 0xff));
        return sb.toString();
    }

    @Test
    public void frameHeaderMatchesThePcApp() {
        // Same bytes as test_pc_assist.py's GOLDEN_FRAME_HEADER.
        byte[] h =
                PcAssistProtocol.frameHeader(
                        7,
                        960,
                        720,
                        2,
                        PcAssistProtocol.CODEC_H264,
                        PcAssistProtocol.FLAG_KEYFRAME,
                        1000);
        assertEquals("4607000000c003d002020101e8030000", hex(h));
        assertEquals(PcAssistProtocol.FRAME_HEADER_BYTES, h.length);
    }

    @Test
    public void resultRoundTrip() throws Exception {
        PcAssistProtocol.Result r = new PcAssistProtocol.Result();
        r.frameId = 42;
        r.pcInferMs = 12.5f;
        r.handCount = 2;
        r.labelledLeft[0] = true;
        for (int i = 0; i < r.image2d.length; i++) r.image2d[i] = i * 0.01f;
        for (int i = 0; i < r.world3d.length; i++) r.world3d[i] = -i * 0.001f;
        byte[] wire = PcAssistProtocol.encodeResult(r);
        PcAssistProtocol.Result back =
                PcAssistProtocol.readResult(new DataInputStream(new ByteArrayInputStream(wire)));
        assertEquals(42, back.frameId);
        assertEquals(12.5f, back.pcInferMs, 0);
        assertEquals(2, back.handCount);
        assertTrue(back.labelledLeft[0]);
        assertFalse(back.labelledLeft[1]);
        assertArrayEquals(r.image2d, back.image2d, 0);
        assertArrayEquals(r.world3d, back.world3d, 0);
    }

    @Test
    public void skippedFrame() throws Exception {
        PcAssistProtocol.Result r = new PcAssistProtocol.Result();
        r.frameId = 9;
        r.skipped = true;
        byte[] wire = PcAssistProtocol.encodeResult(r);
        assertEquals("5209000000" + "00000000" + "ff", hex(wire));
        PcAssistProtocol.Result back =
                PcAssistProtocol.readResult(new DataInputStream(new ByteArrayInputStream(wire)));
        assertTrue(back.skipped);
        assertEquals(9, back.frameId);
    }

    @Test
    public void helloAndBeacon() throws Exception {
        assertEquals("5002", hex(PcAssistProtocol.hello()));
        assertEquals("PCVR-ASSIST 2 9950", PcAssistProtocol.beacon());
        int v =
                PcAssistProtocol.readHello(
                        new DataInputStream(new ByteArrayInputStream(new byte[] {'H', 2})));
        assertEquals(2, v);
    }
}
