import struct

import numpy as np
from radar_vitals_core.tlv import HEADER, MAGIC, PacketDecoder, TLV_HEADER


def make_packet(number, tlvs):
    body = b''.join(TLV_HEADER.pack(kind, len(data)) + data for kind, data in tlvs)
    length = HEADER.size + len(body)
    return HEADER.pack(MAGIC, 0x03060000, length, 0xA6843, number, 0, 0, len(tlvs), 0) + body


def heatmap_bytes(bins=2, antennas=3):
    # Imaginary comes first on the wire: value = real + 1j * imag
    values = [(10 * b + a, 100 * b + a) for b in range(bins) for a in range(antennas)]
    return struct.pack(f'<{2 * len(values)}h', *[part for pair in values for part in pair])


def test_decodes_iq_with_imaginary_first():
    frames = PacketDecoder(range_bins=2).feed(make_packet(7, [(8, heatmap_bytes())]))
    assert len(frames) == 1
    assert frames[0].number == 7
    assert frames[0].iq.shape == (2, 3)
    assert frames[0].iq[1, 2] == complex(102, 12)


def test_decodes_points_and_side_info():
    points = struct.pack('<4f', 1.0, 2.0, 0.5, -0.25) + struct.pack('<4f', 0.0, 1.0, 0.0, 0.0)
    side = struct.pack('<4h', 150, 40, 200, 50)
    frame = PacketDecoder(range_bins=2).feed(make_packet(1, [(1, points), (7, side)]))[0]
    assert frame.iq is None
    assert frame.points['y'][0] == 2.0
    assert frame.side_info['snr'][1] == 200


def test_packet_split_across_reads_with_leading_noise():
    packet = b'noise' + make_packet(3, [(8, heatmap_bytes())])
    decoder = PacketDecoder(range_bins=2)
    assert decoder.feed(packet[:20]) == []
    frames = decoder.feed(packet[20:])
    assert [f.number for f in frames] == [3]


def test_corrupt_length_recovers_at_next_packet():
    good = make_packet(4, [(8, heatmap_bytes())])
    bad = bytearray(good)
    bad[12:16] = struct.pack('<I', 10 ** 7)
    decoder = PacketDecoder(range_bins=2)
    assert [f.number for f in decoder.feed(bytes(bad) + good)] == [4]
    assert decoder.bad_packets == 1


def test_wrong_heatmap_size_is_rejected():
    decoder = PacketDecoder(range_bins=2)
    assert decoder.feed(make_packet(5, [(8, b'\0' * 12)]) + MAGIC[:0]) == []
    assert decoder.bad_packets == 1
    assert np.frombuffer(b'', dtype='<f4').size == 0
