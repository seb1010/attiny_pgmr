import serial
import time

import defs

class PGMR_Utils:
    
    def __init__(self, ser_in: serial.Serial):
        self.ser = ser_in

    def enable_programming(self):
        data = (defs.CONFIG_DUT_PID.to_bytes(1, "big") + 
            defs.ENABLE_PROGRAMMING_PID.to_bytes(1, 'big'))
        data = self.generate_packet(data)
        return self.send_packet(data)


    def send_packet(self, data_to_send: bytes, tries: int=1):
        send_worked = False
        i=0
        while i < tries:
            self.ser.write(data_to_send)
            [read_fail, data] = self.read_packet(3)
            if read_fail == 0 and data == defs.ACK_PID.to_bytes(1, 'big'):
                send_worked = True
                i = tries
            else:
                print(f"failed on try {i + 1}")
                if read_fail == defs.TIMEOUT_ERROR_SEB - 0x100:
                    print("timeout_error")
                elif read_fail == defs.CHECKSUM_ERROR - 0x100:
                    print("invalid packet. checksum error")
            time.sleep(0.001)
            i+=1
        return send_worked
            
    
    def read_packet(self, bytes_to_read: int):
        data_in = self.ser.read(bytes_to_read)
        # print(data_in.hex())
        data = b''
        if data_in == b'':
            read_fail = defs.TIMEOUT_ERROR_SEB - 0x100
            print("host timeout error")
        elif(self.check_sum(data_in) != b'\x00'):
            read_fail = defs.CHECKSUM_ERROR - 0x100
            print("host checksum error")
        else:
            read_fail = 0
            data = data_in[1:-1]
        # print(f"returning {data}, {read_fail}")
        return read_fail, data


    def send_passthrough(self, data: bytes):
        data = defs.DATA_PID.to_bytes(1, 'big') + data
        data = self.generate_packet(data)
        return self.send_packet(data)


    def set_gpio_dd(self, gpio_num: int, set_output: bool):
        data = ((defs.CONFIG_DUT_PID).to_bytes(1, 'big') +
                (defs.SET_DD_PID).to_bytes(1, 'big') + 
                gpio_num.to_bytes(1, 'big'))

        if set_output:
            data = data + b'\x01'
        else:
            data = data + b'\x00'

        data = self.generate_packet(data)
        return self.send_packet(data)


    def set_gpio(self, gpio_num: int, set_high: bool):
        data = ((defs.CONFIG_DUT_PID).to_bytes(1, 'big') +
                (defs.SET_VAL_PID).to_bytes(1, 'big') + 
                gpio_num.to_bytes(1, 'big'))

        if set_high:
            data = data + b'\x01'
        else:
            data = data + b'\x00'

        data = self.generate_packet(data)
        return self.send_packet(data)


    def generate_packet(self, data: bytes):
        first_byte = (len(data) + 2).to_bytes(1, 'big')
        data = first_byte + data
        data = data + self.check_sum(data)
        print(data.hex())
        return data


    def check_sum(self, data: bytes):
        total = 0
        for byte in data:
            total += byte
        total = ((~(total) + 1) & 0xff).to_bytes(1, 'big')
        return total
