"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class tenDOF_t(object):
    __slots__ = ["time", "Yaw", "Pitch", "Roll", "MagX", "MagY", "MagZ", "AccelX", "AccelY", "AccelZ", "GyroX", "GyroY", "GyroZ", "Temp", "Press"]

    def __init__(self):
        self.time = 0
        self.Yaw = 0.0
        self.Pitch = 0.0
        self.Roll = 0.0
        self.MagX = 0.0
        self.MagY = 0.0
        self.MagZ = 0.0
        self.AccelX = 0.0
        self.AccelY = 0.0
        self.AccelZ = 0.0
        self.GyroX = 0.0
        self.GyroY = 0.0
        self.GyroZ = 0.0
        self.Temp = 0.0
        self.Press = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(tenDOF_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qdddddddddddddd", self.time, self.Yaw, self.Pitch, self.Roll, self.MagX, self.MagY, self.MagZ, self.AccelX, self.AccelY, self.AccelZ, self.GyroX, self.GyroY, self.GyroZ, self.Temp, self.Press))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != tenDOF_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return tenDOF_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = tenDOF_t()
        self.time, self.Yaw, self.Pitch, self.Roll, self.MagX, self.MagY, self.MagZ, self.AccelX, self.AccelY, self.AccelZ, self.GyroX, self.GyroY, self.GyroZ, self.Temp, self.Press = struct.unpack(">qdddddddddddddd", buf.read(120))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if tenDOF_t in parents: return 0
        tmphash = (0xa53d1745ef404369) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if tenDOF_t._packed_fingerprint is None:
            tenDOF_t._packed_fingerprint = struct.pack(">Q", tenDOF_t._get_hash_recursive([]))
        return tenDOF_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
