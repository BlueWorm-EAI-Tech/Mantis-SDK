"""
ROS2 CDR (Common Data Representation) 编解码工具
"""

import struct
from typing import List, Optional


class CDREncoder:
    """ROS2 CDR 编码器"""
    
    # CDR 头部 (Little Endian)
    HEADER = bytes([0x00, 0x01, 0x00, 0x00])
    
    @staticmethod
    def _align(buf: bytearray, alignment: int):
        """添加对齐填充"""
        while len(buf) % alignment != 0:
            buf += b'\x00'
    
    @staticmethod
    def encode_float32(value: float) -> bytes:
        """编码 std_msgs/Float32"""
        return CDREncoder.HEADER + struct.pack('<f', value)
    
    @staticmethod
    def encode_float64(value: float) -> bytes:
        """编码 std_msgs/Float64"""
        return CDREncoder.HEADER + struct.pack('<d', value)
    
    @staticmethod
    def encode_bool(value: bool) -> bytes:
        """编码 std_msgs/Bool"""
        return CDREncoder.HEADER + struct.pack('<?', value)
    
    @staticmethod
    def encode_joint_state(
        names: List[str],
        positions: List[float],
        velocities: Optional[List[float]] = None,
        efforts: Optional[List[float]] = None,
        sec: int = 0,
        nanosec: int = 0,
        frame_id: str = ""
    ) -> bytes:
        """
        编码 sensor_msgs/JointState
        
        CDR 格式要求:
        - 字符串: 4字节长度 + 数据 + null + 对齐到4
        - float64数组: 4字节长度 + 对齐到8 + 数据
        """
        buf = bytearray(CDREncoder.HEADER)
        
        # Header.stamp.sec (int32)
        buf += struct.pack('<i', sec)
        # Header.stamp.nanosec (uint32)
        buf += struct.pack('<I', nanosec)
        
        # Header.frame_id (string)
        frame_bytes = frame_id.encode('utf-8')
        buf += struct.pack('<I', len(frame_bytes) + 1)
        buf += frame_bytes + b'\x00'
        CDREncoder._align(buf, 4)
        
        # name[] (sequence<string>)
        buf += struct.pack('<I', len(names))
        for name in names:
            name_bytes = name.encode('utf-8')
            buf += struct.pack('<I', len(name_bytes) + 1)
            buf += name_bytes + b'\x00'
            CDREncoder._align(buf, 4)
        
        # position[] (sequence<float64>)
        # 写入数组长度
        buf += struct.pack('<I', len(positions))
        # 关键：在写 float64 数据前必须对齐到 8 字节
        CDREncoder._align(buf, 8)
        for pos in positions:
            buf += struct.pack('<d', float(pos))
        
        # velocity[] (sequence<float64>)
        vel = velocities if velocities else []
        buf += struct.pack('<I', len(vel))
        if vel:
            CDREncoder._align(buf, 8)
            for v in vel:
                buf += struct.pack('<d', float(v))
        
        # effort[] (sequence<float64>)
        eff = efforts if efforts else []
        buf += struct.pack('<I', len(eff))
        if eff:
            CDREncoder._align(buf, 8)
            for e in eff:
                buf += struct.pack('<d', float(e))
        
        return bytes(buf)
    
    @staticmethod
    def encode_twist(
        linear_x: float, linear_y: float, linear_z: float,
        angular_x: float, angular_y: float, angular_z: float
    ) -> bytes:
        """编码 geometry_msgs/Twist"""
        buf = bytearray(CDREncoder.HEADER)
        buf += b'\x00' * 4  # 8字节对齐
        
        # linear
        buf += struct.pack('<d', linear_x)
        buf += struct.pack('<d', linear_y)
        buf += struct.pack('<d', linear_z)
        
        # angular
        buf += struct.pack('<d', angular_x)
        buf += struct.pack('<d', angular_y)
        buf += struct.pack('<d', angular_z)
        
        return bytes(buf)


class CDRDecoder:
    """ROS2 CDR 解码器"""
    
    @staticmethod
    def decode_joint_state(data: bytes) -> dict:
        """解码 sensor_msgs/JointState"""
        offset = 4  # 跳过 CDR header
        
        # stamp
        sec = struct.unpack_from('<i', data, offset)[0]
        offset += 4
        nanosec = struct.unpack_from('<I', data, offset)[0]
        offset += 4
        
        # frame_id
        frame_len = struct.unpack_from('<I', data, offset)[0]
        offset += 4
        frame_id = data[offset:offset + frame_len - 1].decode('utf-8')
        offset += frame_len
        offset = (offset + 3) // 4 * 4
        
        # names
        num_names = struct.unpack_from('<I', data, offset)[0]
        offset += 4
        names = []
        for _ in range(num_names):
            name_len = struct.unpack_from('<I', data, offset)[0]
            offset += 4
            name = data[offset:offset + name_len - 1].decode('utf-8')
            offset += name_len
            offset = (offset + 3) // 4 * 4
            names.append(name)
        
        # 8字节对齐
        offset = (offset + 7) // 8 * 8
        
        # positions
        num_pos = struct.unpack_from('<I', data, offset)[0]
        offset += 4
        offset += 4  # padding
        positions = []
        for _ in range(num_pos):
            positions.append(struct.unpack_from('<d', data, offset)[0])
            offset += 8
        
        # velocities
        num_vel = struct.unpack_from('<I', data, offset)[0]
        offset += 4
        velocities = []
        if num_vel > 0:
            offset += 4
            for _ in range(num_vel):
                velocities.append(struct.unpack_from('<d', data, offset)[0])
                offset += 8
        
        # efforts
        num_eff = struct.unpack_from('<I', data, offset)[0]
        offset += 4
        efforts = []
        if num_eff > 0:
            offset += 4
            for _ in range(num_eff):
                efforts.append(struct.unpack_from('<d', data, offset)[0])
                offset += 8
        
        return {
            'stamp': (sec, nanosec),
            'frame_id': frame_id,
            'name': names,
            'position': positions,
            'velocity': velocities,
            'effort': efforts
        }
