import struct

from crc import CrcCalculator, Configuration

CRC16_XMODEM = Configuration(
    width=16,
    polynomial=0x1021,
    init_value=0x0000,
    final_xor_value=0x0000,
    reverse_input=False,
    reverse_output=False,
)

crc_calculator = CrcCalculator(CRC16_XMODEM, table_based=True)

if __name__ == "__main__":
    identifier = b"\x01"

    write_struct = struct.Struct(">f")
    msg_packed = write_struct.pack(0.3)

    checksum = crc_calculator.calculate_checksum(identifier + msg_packed)

    checksum_struct = struct.Struct(">H")
    checksum_packed = checksum_struct.pack(checksum)

    crc_calculator.calculate_checksum(b"abcdefg")
