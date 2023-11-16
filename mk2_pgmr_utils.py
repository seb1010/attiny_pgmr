import serial
import time

import defs

class PGMR_Utils:
    
    def __init__(self, ser_in: serial.Serial):
        self.ser = ser_in
        self.ACK_PACKET = b'\x03' + defs.ACK_PID.to_bytes(1, "big")
        self.ACK_PACKET += self.check_sum(self.ACK_PACKET)
        self.NAK_PACKET = b'\x03' + defs.NAK_PID.to_bytes(1, "big")
        self.NAK_PACKET += self.check_sum(self.NAK_PACKET)

    def enable_programming(self, enable: bool):
        data = (defs.CONFIG_DUT_PID.to_bytes(1, "big") + 
            defs.ENABLE_PROGRAMMING_PID.to_bytes(1, 'big'))
        if(enable):
            data += b'\x01'
        else:
            data += b'\x00'
        data = self.generate_packet(data)
        return self.send_packet(data)


    def and_read_spi(self, and_read: bool):
        data = (defs.CONFIG_PGMR_PID.to_bytes(1, "big") + 
                defs.READ_SPI_PID.to_bytes(1, "big"))
        if and_read:
            data += b'\x01'
        else:
            data += b'\x00'
        data = self.generate_packet(data)
        return self.send_packet(data)

    def set_spi_rate(self, spi_per: int):
        data = (defs.CONFIG_PGMR_PID.to_bytes(1, "big") + 
                defs.SET_SPI_RATE_PID.to_bytes(1, "big"))
        data += spi_per.to_bytes(1, "big")
        data = self.generate_packet(data)
        return self.send_packet(data)

    def get_pgmr_status(self):
        self.ser.flushInput()
        data = defs.GET_PGMR_STATUS_PID.to_bytes(1, "big")
        data = self.generate_packet(data)
        self.send_packet(data)
        read_fail, data = self.read_packet()
        if read_fail:
            self.ser.write(self.NAK_PACKET)
            ret = b''
        else:
            self.ser.write(self.ACK_PACKET)
            #print(f"spi rate: {data[1]}") 
            #print(f"and_read_spi: {data[2]}")
            #print(f"programming enabled: {data[3]}")
            ret = data[3].to_bytes(1,"big")
        return ret


    def send_packet(self, data_to_send: bytes, tries: int=3):
        # its still 1 try bc will fail out on nac
        send_worked = False
        i=0
        while i < tries:
            self.ser.write(data_to_send)
            [read_fail, data] = self.read_packet()
            if not read_fail and data == defs.ACK_PID.to_bytes(1, 'big'):
                send_worked = True
                i = tries
            else:
                print(f"failed on try {i + 1}")
                print(f"read failed {read_fail}")
                print(f"data read in: {data.hex()}")
                if read_fail == defs.TIMEOUT_ERROR_SEB - 0x100:
                    print("timeout_error")
                elif read_fail == defs.CHECKSUM_ERROR - 0x100:
                    print("invalid packet. checksum error")
                assert False, "failed to get ack. Retries not implemented"
            i+=1
        return send_worked
            
    
    def read_packet(self, bytes_to_read: int=0):
        first_byte_in = self.ser.read()
        data_in = first_byte_in
        for _ in range(int.from_bytes(first_byte_in, "big") - 1):
            data_in += self.ser.read()

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

        # things get out of sync if we are looking at old data
        # don't need to do this on the u side bc there is really no buffer
        if read_fail:
            self.ser.flushInput()
        #print(f"read packet | data: {data.hex()} failed: {read_fail}")
        return read_fail, data



    def spi_loopback_test(self, data: bytes):
        # self.ser.flushInput()
        send_failed = 0
        self.and_read_spi(True)
        data = self.generate_packet(b'\x3c' + data)
        send_failed = self.send_packet(data)

        _, data_back = self.read_packet()
        data_back = data_back[1:]
        data = data[2:-1]
        if data == data_back:
            ret = True
        else:
            ret = False
            print(f"spi sent: {data.hex()}")
            print(f"spi read: {data_back.hex()}")
        return ret

    def send_passthrough(self, data: bytes, and_read: bool, override: bool = False):
        # this just slows things down. i think this checking shouwld be sone at a igher lkevel
        #if and_read:
        #    self.and_read_spi(and_read)
        # prog_en = self.get_pgmr_status()

        data_out = b''
        if True:# prog_en or override:
            data = defs.DATA_PID.to_bytes(1, 'big') + data
            data = self.generate_packet(data)
            send_worked = self.send_packet(data)
            #print(f"\nsend_worked {send_worked}")
            data_read = False
            while and_read and not data_read:
                read_failed, data = self.read_packet()
                #print(f"\nread_ok in passthrough: {read_failed}")
                if not read_failed:
                    self.ser.write(self.ACK_PACKET)
                    # print("sending ack")
                    data_out = data[1:]
                    data_read = True #eventually maybe do something here
                else:
                    self.ser.write(self.NAK_PACKET)
                    assert False, "Failed to send. Retries not implemented"
                  
                #print(f"spi_says: {data_out.hex()}")
        else:
            raise ValueError("need prog enabled or override set")
        return data_out


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
        return data


    def check_sum(self, data: bytes):
        total = 0
        for byte in data:
            total += byte
        total = ((~(total) + 1) & 0xff).to_bytes(1, 'big')
        return total
