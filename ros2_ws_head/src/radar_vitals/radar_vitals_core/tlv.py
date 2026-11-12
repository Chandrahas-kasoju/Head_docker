"""Decoder for the UART packets sent by the TI out-of-box demo (mmWave SDK 3.x)."""
from dataclasses import dataclass
import struct
from typing import Optional

import numpy as np

MAGIC = bytes([2, 1, 4, 3, 6, 5, 8, 7])
HEADER = struct.Struct('<8s8I')  # magic, version, length, platform, frame, cycles, objects, TLVs, subframe
TLV_HEADER = struct.Struct('<2I')
MAX_PACKET_BYTES = 65536

TLV_DETECTED_POINTS = 1
TLV_POINTS_SIDE_INFO = 7
TLV_AZIMUTH_HEATMAP = 4
TLV_AZIMUTH_ELEVATION_HEATMAP = 8  # The AOP build uses this type

POINT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('velocity', '<f4')])
SIDE_INFO_DTYPE = np.dtype([('snr', '<i2'), ('noise', '<i2')])  # Units of 0.1 dB


@dataclass
class RadarFrame:
    number: int
    iq: Optional[np.ndarray]  # Complex, shape (range bins, antennas)
    points: np.ndarray  # POINT_DTYPE, radar axes: x right, y forward, z up
    side_info: Optional[np.ndarray]  # SIDE_INFO_DTYPE, or None


class PacketDecoder:
    """Turns a stream of UART bytes into RadarFrame objects, resynchronising on the magic word."""

    def __init__(self, range_bins):
        self.range_bins = range_bins
        self._buffer = b''
        self.bad_packets = 0

    def feed(self, data):
        self._buffer += data
        frames = []
        while True:
            start = self._buffer.find(MAGIC)
            if start < 0:
                self._buffer = self._buffer[-(len(MAGIC) - 1):]
                return frames
            if len(self._buffer) - start < HEADER.size:
                self._buffer = self._buffer[start:]
                return frames
            fields = HEADER.unpack_from(self._buffer, start)
            length, number, num_tlvs = fields[2], fields[4], fields[7]
            if not HEADER.size <= length <= MAX_PACKET_BYTES:
                self.bad_packets += 1
                self._buffer = self._buffer[start + len(MAGIC):]
                continue
            if len(self._buffer) - start < length:
                self._buffer = self._buffer[start:]
                return frames
            packet = self._buffer[start:start + length]
            self._buffer = self._buffer[start + length:]
            frame = self._decode(packet, number, num_tlvs)
            if frame is None:
                self.bad_packets += 1
            else:
                frames.append(frame)

    def _decode(self, packet, number, num_tlvs):
        tlvs = {}
        offset = HEADER.size
        for _ in range(num_tlvs):
            if offset + TLV_HEADER.size > len(packet):
                return None
            kind, size = TLV_HEADER.unpack_from(packet, offset)
            offset += TLV_HEADER.size
            if offset + size > len(packet):
                return None
            tlvs[kind] = packet[offset:offset + size]
            offset += size

        iq = None
        heatmap = tlvs.get(TLV_AZIMUTH_ELEVATION_HEATMAP, tlvs.get(TLV_AZIMUTH_HEATMAP))
        if heatmap is not None:
            if len(heatmap) % (4 * self.range_bins):
                return None
            # Firmware order: imaginary then real, for each antenna within each range bin
            raw = np.frombuffer(heatmap, dtype='<i2').astype(np.float32)
            iq = (raw[1::2] + 1j * raw[0::2]).reshape(self.range_bins, -1).astype(np.complex64)

        points = np.frombuffer(tlvs.get(TLV_DETECTED_POINTS, b''), dtype=POINT_DTYPE)
        side_info = None
        if TLV_POINTS_SIDE_INFO in tlvs:
            side_info = np.frombuffer(tlvs[TLV_POINTS_SIDE_INFO], dtype=SIDE_INFO_DTYPE)
            if len(side_info) != len(points):
                side_info = None
        return RadarFrame(number, iq, points, side_info)


def read_radar_config(path):
    """Return the config commands plus the range bin count, bin size in metres and frame period in ms."""
    with open(path) as f:
        lines = [line.strip() for line in f]
    commands = [line for line in lines if line and not line.startswith('%')]
    profile = next(c.split() for c in commands if c.startswith('profileCfg'))
    frame = next(c.split() for c in commands if c.startswith('frameCfg'))
    slope_mhz_per_us, samples, rate_ksps = float(profile[8]), int(profile[10]), float(profile[11])
    range_bins = 1 << int(np.ceil(np.log2(samples)))
    # With complex sampling, the full sample rate maps to the maximum range
    max_range_m = rate_ksps * 1e3 * 3e8 / (2 * slope_mhz_per_us * 1e12)
    return commands, range_bins, max_range_m / range_bins, float(frame[5])
