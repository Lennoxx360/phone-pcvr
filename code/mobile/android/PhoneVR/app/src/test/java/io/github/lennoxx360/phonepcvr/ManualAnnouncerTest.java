/* (C)2026 */
package io.github.lennoxx360.phonepcvr;

import static org.junit.Assert.*;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import org.junit.Test;

public class ManualAnnouncerTest {
    @Test
    public void packetMatchesAlvrV20Layout() {
        byte[] p = ManualAnnouncer.buildPacket("1234.client");
        assertEquals(56, p.length);
        assertEquals('A', p[0]);
        assertEquals('L', p[1]);
        assertEquals('V', p[2]);
        assertEquals('R', p[3]);
        for (int i = 4; i < 16; i++) assertEquals(0, p[i]);
        long id = ByteBuffer.wrap(p, 16, 8).order(ByteOrder.LITTLE_ENDIAN).getLong();
        assertEquals(1686174404061257063L, id);
        assertEquals("1234.client", new String(p, 24, 11));
        for (int i = 35; i < 56; i++) assertEquals(0, p[i]);
    }

    @Test
    public void longHostnameIsTruncated() {
        byte[] p = ManualAnnouncer.buildPacket("x".repeat(40));
        assertEquals(56, p.length);
        assertEquals('x', p[55]);
    }

    @Test
    public void validatesIps() {
        assertTrue(ManualAnnouncer.isValidIp("192.168.137.1"));
        assertTrue(ManualAnnouncer.isValidIp(" 10.0.0.5 "));
        assertFalse(ManualAnnouncer.isValidIp("192.168.137"));
        assertFalse(ManualAnnouncer.isValidIp("256.1.1.1"));
        assertFalse(ManualAnnouncer.isValidIp("1..1.1"));
        assertFalse(ManualAnnouncer.isValidIp(""));
        assertFalse(ManualAnnouncer.isValidIp(null));
    }
}
