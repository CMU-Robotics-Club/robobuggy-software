import struct
import time
from dataclasses import dataclass

from serial import Serial

class Crc16:
    def __init__(self):
        self.accum = 0

    def update(self, b):
        if isinstance(b, int):
            assert(0 <= b < 256)

            crc_table = [
                0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
                0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
                0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
                0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
                0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
                0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
                0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
                0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
                0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
                0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
                0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
                0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
                0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
                0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
                0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
                0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
                0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
                0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
                0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
                0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
                0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
                0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
                0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
                0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
                0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
                0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
                0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
                0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
                0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
                0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
                0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
                0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
            ]

            i = ((self.accum >> 8) ^ b) & 0xFF;
            i %= 256
            self.accum = ((self.accum << 8) & 0xFF00) ^ crc_table[i];
        else:
            for one_byte in b:
                self.update(one_byte)

SYNC_WORD = b'\xAA\xFF\x00\x55'

MAX_PAYLOAD_LEN = 100


# firmware --> software
MSG_TYPE_NAND_DEBUG = b'ND'
MSG_TYPE_NAND_UKF = b'NU'
MSG_TYPE_NAND_GPS = b'NG'
MSG_TYPE_RADIO = b'RA'
MSG_TYPE_SC_DEBUG = b'SD'
MSG_TYPE_SC_SENSORS = b'SS'
MSG_TYPE_ROUNDTRIP_TIMESTAMP = b'RT'

# software --> firmware
MSG_TYPE_STEERING = b'ST'
MSG_TYPE_ALARM    = b'AL'
MSG_TYPE_SOFTWARE_TIMESTAMP = b'TM'



@dataclass
class NANDDebug:
    timestamp: int
    rc_steering_angle: float
    stepper_steering_angle: float
    accelerometer: float
    encoder_angle: float
    operator_ready: bool
    brake_status: bool
    auton_steer: bool
    rfm69_timeout: bool
    tx12_state: bool
    stepper_alarm: bool
    rc_uplink_quality: int

@dataclass
class NANDUKF:
    timestamp: int
    easting: float
    northing: float
    theta: float
    omega: float
    velocity: float

@dataclass
class NANDGPS:
    gps_seqnum: int
    easting: float
    northing: float
    gps_fix: int
    # this is a 2D accuracy value
    accuracy: int
    timestamp: int

@dataclass
class Radio:
    gps_seqnum: int
    nand_x_gps: float
    nand_y_gps: float
    nand_gps_fix: float

@dataclass
class SCDebug:
    timestamp: int
    rc_steering_angle: float
    stepper_steering_angle: float
    # encoder_angle: float
    operator_ready: bool
    brake_status: bool
    auton_steer: bool
    tx12_state: bool
    stepper_alarm: bool
    rc_uplink_quality: int


@dataclass
class SCSensors:
    steering: float
    # velocity: float
    timestamp: int

@dataclass
class RoundtripTimestamp:
    returned_time: int


class IncompletePacket(Exception):
    pass

class ChecksumMismatch(Exception):
    pass

class Comms:
    def __init__(self, path_to_port):
        self.port = Serial(path_to_port, 1_000_000)
        self.rx_buffer = b''

    def send_packet_raw(self, msg_type: bytes, payload: bytes):
        assert(len(msg_type) == 2)
        assert(len(payload) < MAX_PAYLOAD_LEN)

        checksum = Crc16()
        def write_and_checksum(data: bytes):
            nonlocal checksum

            self.port.write(data)
            checksum.update(data)

        self.port.write(SYNC_WORD)
        write_and_checksum(msg_type)
        write_and_checksum(len(payload).to_bytes(2, 'little'))
        write_and_checksum(payload)
        self.port.write(checksum.accum.to_bytes(2, 'little'))

    def send_steering(self, angle: float):
        self.send_packet_raw(MSG_TYPE_STEERING, struct.pack('<d', angle))

    def send_alarm(self, status: int):
        self.send_packet_raw(MSG_TYPE_ALARM, struct.pack('<B', status))

    def send_timestamp(self, time: float):
        self.send_packet_raw(MSG_TYPE_SOFTWARE_TIMESTAMP, struct.pack('<d', time))

    def read_packet_raw(self):
        self.rx_buffer += self.port.read_all() #type:ignore
        try:
            return self._try_parse_buffer()
        except IncompletePacket:
            return None
        except ChecksumMismatch:
            return None


    def _try_parse_buffer(self):
        while True:
            checksum = Crc16()
            index = 0
            def read(count: int):
                nonlocal index

                if index + count > len(self.rx_buffer):
                    raise IncompletePacket()

                data = self.rx_buffer[index:][:count]
                index += count
                return data

            def read_and_checksum(count: int):
                nonlocal checksum

                data = read(count)
                checksum.update(data)
                return data

            if read(1) != SYNC_WORD[0:1]:
                self.rx_buffer = self.rx_buffer[1:]
                continue
            if read(1) != SYNC_WORD[1:2]:
                self.rx_buffer = self.rx_buffer[1:]
                continue
            if read(1) != SYNC_WORD[2:3]:
                self.rx_buffer = self.rx_buffer[1:]
                continue
            if read(1) != SYNC_WORD[3:4]:
                self.rx_buffer = self.rx_buffer[1:]
                continue

            msg_type = read_and_checksum(2)
            msg_len = int.from_bytes(read_and_checksum(2), 'little')

            if msg_len > MAX_PAYLOAD_LEN:
                self.rx_buffer = self.rx_buffer[1:]
                continue

            payload = read_and_checksum(msg_len)

            rx_crc = int.from_bytes(read(2), 'little')

            self.rx_buffer = self.rx_buffer[4 + 3 + 2 + msg_len + 2:]

            if rx_crc != checksum.accum:
                raise ChecksumMismatch()

            return (msg_type, payload)

    def read_packet(self):
        packet = self.read_packet_raw()
        if packet is None:
            return None

        msg_type, payload = packet
        if msg_type == MSG_TYPE_NAND_DEBUG:
            x, y, radio_seqnum, gps_seqnum = struct.unpack('<Idddd??????B', payload)
            return NANDDebug(x, y, radio_seqnum, gps_seqnum)

        elif msg_type == MSG_TYPE_NAND_UKF:
            x, y, radio_seqnum, gps_seqnum = struct.unpack('<ddII', payload)
            return NANDUKF(x, y, radio_seqnum, gps_seqnum)

        elif msg_type == MSG_TYPE_NAND_GPS:
            x, y, vel, steering, heading, heading_rate = struct.unpack('<dddddd', payload)
            return NANDGPS(x, y, vel, steering, heading, heading_rate)

        elif msg_type == MSG_TYPE_RADIO:
            x, y, vel, steering, heading, heading_rate = struct.unpack('<dddddd', payload)
            return Radio(x, y, vel, steering, heading, heading_rate)

        elif msg_type == MSG_TYPE_SC_DEBUG:
            x, y, vel, steering, heading, heading_rate = struct.unpack('<dddddd', payload)
            return SCDebug(x, y, vel, steering, heading, heading_rate)

        elif msg_type == MSG_TYPE_SC_SENSORS:
            x, y, vel, steering, heading, heading_rate = struct.unpack('<dddddd', payload)
            return SCSensors(x, y, vel, steering, heading, heading_rate)

        elif msg_type == MSG_TYPE_ROUNDTRIP_TIMESTAMP:
            x, y, vel, steering, heading, heading_rate = struct.unpack('<dddddd', payload)
            return RoundtripTimestamp(x, y, vel, steering, heading, heading_rate)
        else:
            print(f'Unknown packet type {msg_type}')
            return None



def main():
    comms = Comms('/dev/ttyUSB0')

    print('Starting!')

    last_time = time.time()
    while True:
        packet = comms.read_packet()
        if time.time() - last_time > 0.01:
            print(packet)
            last_time = time.time()
            comms.send_steering(1234.5)

if __name__ == '__main__':
    main()