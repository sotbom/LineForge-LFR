package com.lineforge.lfr;

import java.util.Arrays;

/**
 * Recovering streaming decoder for LineForge Bluetooth SPP.
 * SPP is a byte stream, not a packet transport. Handles arbitrary read splits,
 * garbage/ROM boot logs, broken checksums, legacy replies and framed packets.
 */
public final class Protocol {
    private Protocol() { }
    public static final int MAX_PAYLOAD = 48;
    public static final int STATUS = 0x51;
    public static final int SENSORS = 0x52;
    public static final int CALIBRATION = 0x53;

    public interface Listener {
        void onPacket(int type, byte[] payload);
        void onLegacyStatus(byte[] payload);
        void onReply(int code);
        void onDiscarded(int count, boolean checksumError);
    }

    public static int crc8(int crc, int b) {
        crc ^= b;
        for (int k = 0; k < 8; k++)
            crc = ((crc & 0x80) != 0) ? (((crc << 1) ^ 0x07) & 255) : ((crc << 1) & 255);
        return crc;
    }

    public static byte[] frame(int type, int seq, byte[] payload) {
        if (payload.length > MAX_PAYLOAD) throw new IllegalArgumentException("oversize packet");
        byte[] out = new byte[6 + payload.length];
        out[0] = (byte)0xA5; out[1] = 0x5A;
        out[2] = (byte)type; out[3] = (byte)payload.length; out[4] = (byte)seq;
        int crc = 0;
        for (int i = 2; i < 5; i++) crc = crc8(crc, out[i] & 255);
        for (int i = 0; i < payload.length; i++) {
            out[5+i] = payload[i];
            crc = crc8(crc, payload[i] & 255);
        }
        out[5+payload.length] = (byte)crc;
        return out;
    }

    public static final class Parser {
        private final Listener listener;
        private final byte[] buffered = new byte[1024];
        private int count = 0;
        public Parser(Listener listener) { this.listener = listener; }

        public void feed(byte[] b, int length) {
            for (int i = 0; i < length; i++) {
                if (count >= buffered.length) { discard(1); listener.onDiscarded(1, false); }
                buffered[count++] = b[i];
                parse();
            }
        }
        private void discard(int countToRemove) {
            int next = count - countToRemove;
            if (next > 0) System.arraycopy(buffered, countToRemove, buffered, 0, next);
            count = Math.max(0, next);
        }
        private static boolean recognizedReply(int b) {
            return b == 0x06 || b == 0x15 || b == 0xC0 ||
                   b == 0xC1 || b == 0xC2 || b == 0xD1;
        }
        private void parse() {
            while (count > 0) {
                int first = buffered[0] & 255;
                if (first == 0xA5) {
                    if (count < 2) return;
                    if ((buffered[1] & 255) != 0x5A) {
                        discard(1); listener.onDiscarded(1, false); continue;
                    }
                    if (count < 5) return;
                    int len = buffered[3] & 255;
                    int type = buffered[2] & 255;
                    if (len > MAX_PAYLOAD || (type != STATUS && type != SENSORS && type != CALIBRATION)) {
                        discard(1); listener.onDiscarded(1, false); continue;
                    }
                    int size = 6 + len;
                    if (count < size) return;
                    int crc = 0;
                    for (int i = 2; i < 5 + len; i++) crc = crc8(crc, buffered[i] & 255);
                    if (crc != (buffered[5+len] & 255)) {
                        discard(1); listener.onDiscarded(1, true); continue;
                    }
                    byte[] payload = Arrays.copyOfRange(buffered, 5, 5+len);
                    discard(size);
                    listener.onPacket(type, payload);
                    continue;
                }
                if (first == 0xD0) {
                    if (count < 9) return;
                    int x = 0;
                    for (int i = 1; i <= 7; i++) x ^= buffered[i] & 255;
                    if (x != (buffered[8] & 255) ||
                        (buffered[2] & 255) > 4 ||
                        (buffered[4] & 255) > 4 ||
                        (buffered[6] & 255) > 4 ||
                        (buffered[7] & 255) > 7) {
                        discard(1); listener.onDiscarded(1, true); continue;
                    }
                    byte[] body = Arrays.copyOfRange(buffered, 1, 9);
                    discard(9);
                    listener.onLegacyStatus(body);
                    continue;
                }
                if (recognizedReply(first)) {
                    discard(1);
                    listener.onReply(first);
                    continue;
                }
                discard(1);
                listener.onDiscarded(1, false);
            }
        }
    }
}
