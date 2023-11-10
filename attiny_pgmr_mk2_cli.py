import serial
import sys

def main(argv):

    ser = serial.Serial(argv[0])
    ser.baudrate = 500000
    ser.timeout = 0.1


    while(True):
        byte=ser.read()
        if byte != b'':
            read_packet(ser=ser, byte=byte)




def read_packet(ser: serial.Serial, byte: bytes):
    packet_len = int.from_bytes(byte, "big")

    # finish reading in packet
    packet_in = byte
    packet_in += ser.read(size=packet_len - 1)

    checksum = 0
    for data in packet_in:
        checksum += data
    checksum &= 0xff

    if checksum == 0 and len(packet_in) > 2:
        if packet_in[1] == 0x03:
            log_printer(packet_in=packet_in)
        else:
            print(f"\033[92mPacket from programmer:\033[39m {packet_in[1:-1].hex()}")
    else:
        print(f"\033[91mRandom Crap:\033[39m {packet_in.hex()}")




def log_printer(packet_in):

    # find text len in packet
    i = 3;
    while packet_in[i] != 0:
        i += 1

    # insert number into text
    init_text = str(packet_in[2:i], encoding='ascii')
    data = int.from_bytes(packet_in[i:-1], "big")
    if init_text.find("%x") > 0:
        loc = init_text.find("%x")
        text = init_text[0:loc] + hex(data) + init_text[loc+2:]
    elif init_text.find("%d") > 0:
        loc = init_text.find("%d")
        text = init_text[0:loc] + str(data) + init_text[loc+2:]
    else:
        text = init_text
    
    print("\u001b[32mPGMR Says: \u001b[0m", end='')
    print(text)

if __name__ == "__main__":
    main(sys.argv[1:])

