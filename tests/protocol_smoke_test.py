"""Offline checks for LineForge two-byte commands and nine-byte readback frame."""

def gain_packet(kp, kp_div, ki, ki_div, kd, kd_div):
    for v in [kp, ki, kd]:
        assert 0 <= v <= 255
    for d in [kp_div, ki_div, kd_div]:
        assert 0 <= d <= 4
    return bytes([1, kp, 2, kp_div, 3, ki, 4, ki_div, 5, kd, 6, kd_div])

def status_frame(values, flags):
    assert len(values) == 6
    body = bytes([*values, flags])
    crc = 0
    for b in body:
        crc ^= b
    return bytes([0xD0]) + body + bytes([crc])

def run():
    gain = gain_packet(40, 0, 0, 0, 1, 0)
    frame = status_frame([40, 0, 0, 0, 1, 0], 0x01)
    assert gain == bytes([1, 40, 2, 0, 3, 0, 4, 0, 5, 1, 6, 0])
    assert len(gain) == 12
    assert len(frame) == 9
    assert (len(gain) + 6 + 9 + 2) < 256
    assert frame[-1] == 40 ^ 1 ^ 1
    print('PASS: two-byte command encoding, 12-byte PID batch, 9-byte status, XOR and size budget')

if __name__ == '__main__':
    run()
