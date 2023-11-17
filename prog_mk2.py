import serial
import time
import numpy as np
import defs
import mk2_pgmr_utils

class Prog:

    def __init__(self, device='/dev/ttyACM0'):

        self.pac = b'hello'
        self.ser = serial.Serial(device)
        self.ser.baudrate=500000
        self.ser.timeout = 1
        self.hex_struct = []
        self.MAX_PACKET_LEN = defs.MAX_PACKET_LEN - 4;
        self._utils = mk2_pgmr_utils.PGMR_Utils(self.ser)
        
    def program_device(self,
                       filename: str,
                       page_size: int,
                       release_reset:bool=True):
       self.ser.flushInput()
       self._utils.and_read_spi(False)
       if self._utils.enable_programming(True):
           time.sleep(0.1)
           self._utils.send_passthrough(b'\xac\x80\x00\x00', False)
           time.sleep(0.1)
           self.read_hex(filename=filename)
           # self.print_hex_struct()
           self.send_hex_to_device(page_size=page_size)
           if release_reset:
               self._utils.enable_programming(False)


    def send_hex_to_device(self, page_size):
        packet = b''
        self._utils.and_read_spi(False)
        for i, val in enumerate(self.hex_struct):
            word_addrs = val[0] % page_size
            for j in range(2):
                packet += int(0x40 + (j << 3)).to_bytes(1, 'big')
                packet += word_addrs.to_bytes(2, 'big')
                packet += val[j+1].to_bytes(1, 'big')

            # do we have space? or do we need to send a packet?
            if len(packet) >= self.MAX_PACKET_LEN - 15: # if we have space (should be 12)
                self._utils.send_passthrough(packet, False)
                #print(f"sending packet w/o page write: {len(packet)}")
                packet = b''
                
            page_addrs = val[0] & ~(page_size - 1)
            

            # decide if we need to write a page
            if i == len(self.hex_struct) - 1:
                print("end of struct")
                write_page = True
            elif (self.hex_struct[i+1][0] & ~(page_size-1)) != page_addrs:
                # write page if next page address != current address
                write_page = True
            else:
                write_page = False

            if write_page:
                print("writing page: " + str(page_addrs // page_size))
                packet += b'\x4c'
                packet += page_addrs.to_bytes(2, 'big')
                packet += b'\x00'
                self._utils.send_passthrough(packet, False)
                packet = b''
                time.sleep(0.01)

    def prog_test(self, also_read: bool):
        # also_read = True
        self._utils.enable_programming(True)
        time.sleep(0.01)
        x = self._utils.send_passthrough(b'\xac\x80\x00\x00', also_read)
        x = self._utils.send_passthrough(b'\xac\x80\x00\x00', also_read)
        print(x.hex())
        time.sleep(0.01)
        x = self._utils.send_passthrough(b'\x40\x00\x00\x00\x48\x00\x00\x00', also_read)
        print(x.hex())
        x = self._utils.send_passthrough(b'\x4c\x00\x00\x00', also_read)
        print(x.hex())

    def read_hex(self, filename: str):
        self.hex_struct = []
        with open(filename, "r") as fp:
            data_line = fp.readline()
            while data_line != '':
                # print(data_line)
                if data_line[7:9] == '00': 
                    self.add_line_to_hex_struct(data_line)
                data_line = fp.readline()

        idx = np.argsort(list(list(zip(*self.hex_struct))[0]))
        temp_array = [[] for i in self.hex_struct]
        for i, j in enumerate(idx):
            temp_array[i] = self.hex_struct[j]

        self.hex_struct = temp_array
        #self.print_hex_struct()

    def add_line_to_hex_struct(self, data_line: str):
        line_length = int(data_line[1:3], 16) // 2
        addrs = int(data_line[3:7], 16) // 2
        data_only = data_line[9:line_length * 4 + 9]

        if len(data_only) % 4:
            raise NotImplementedError("unsupported hex file. fix me")

        for i in range(0, len(data_only), 4):
          new_low_byte = int(data_only[i:i+2], 16)
          new_high_byte = int(data_only[i+2:i+4], 16)
          new_addrs = addrs + i // 4
          self.hex_struct.append([new_addrs, new_low_byte, new_high_byte])

    def print_hex_struct(self):
        for val in self.hex_struct:
            print("ADDRS: " + format(val[0], '04x'), end=' | ')
            print("LOW: " + format(val[1], '02x'), end=' | ')
            print("HIGH: " + format(val[2], '02x'))

    def send_bytes_s(self, bytes_out):
        print(bytes_out.hex())
        print(bytes_out)

    def read_mem(self,
                 start_addrs: int=0,
                 stop_addrs: int=31,
                 enable_prog: bool=True): # last thing is not active

        data_out = b''      # to enable programming
        j = 0
        bytes_to_send = b''
        for i, addrs in enumerate(range(start_addrs, stop_addrs)):
            bytes_out = addrs.to_bytes(2, "big") + b'\x00'
            bytes_to_send += b'\x20' + bytes_out + b'\x28' + bytes_out
            j += 8
            if j <= self.MAX_PACKET_LEN - 8:
                need_send = False
            else:
                need_send = True
                # print(f"bytes to send: {bytes_to_send}")

            if need_send or addrs == stop_addrs - 1:
                temp_data = self._utils.send_passthrough(bytes_to_send, True)
                # time.sleep(0.1)
                for k, val in enumerate(temp_data):
                    if k % 4 == 3:
                        data_out += val.to_bytes(1, "big")
                j = 0
                bytes_to_send = b''

        # self.print_mem(data_out)
        return data_out

    def print_mem(self, bytes_in: bytes):
        for i, byte in enumerate(bytes_in):
            print(format(byte, '2X'), end='')
            if i % 0x10 == 0x0f:
                print('')

    # checks checks with pgmr if programming is enabled, if not will enable
    def verify_prog(self, filename: str):
       self.ser.flushInput()
       self._utils.and_read_spi(True)
       if self._utils.get_pgmr_status():
           enable_prog = True
       else:
           enable_prog = self._utils.enable_programming(True)

       if enable_prog:
           all_match = True
       else:
           all_match = False
           assert False, "unable to enable programming"

       line_num = 0
       with open(filename, "r") as fp:
           data_line = fp.readline()
           while data_line != '' and all_match:
               if data_line[7:9] == '00':
                   addrs = int(data_line[3:7], 16) // 2
                   num_bytes = int(data_line[1:3], 16) // 2
                   bytes_dut = self.read_mem(start_addrs=addrs,
                       stop_addrs=addrs+num_bytes, enable_prog=False)
                   file_bytes = bytes.fromhex(data_line[9:num_bytes * 4 + 9])
                   if bytes_dut == file_bytes:
                       if line_num % 20 == 0 and line_num > 1:
                           print("line " + str(line_num) + " is a match")
                   else:
                       print("LINE " + str(line_num) + " MISSMATCH!!")
                       print("file : " + file_bytes.hex())
                       print("dut  : " + bytes_dut.hex())            
                       all_match = False
               line_num += 1
               data_line = fp.readline()
       self._utils.enable_programming(False)
       return all_match


    def read_fuse_bits(self):
        self.ser.flushInput()
        self._utils.and_read_spi(True)
        if self._utils.enable_programming(True):
            data = self._utils.send_passthrough(b'\x58\x00\x00\x00', True)
            print(f"lock bits: {hex(data[3])}")
            data = self._utils.send_passthrough(b'\x50\x00\x00\x00', True)
            print(f"fuse bits: {hex(data[3])}")
            data = self._utils.send_passthrough(b'\x58\x08\x00\x00', True)
            print(f"fuse high bits: {hex(data[3])}")
            data = self._utils.send_passthrough(b'\x50\x08\x00\x00', True)
            print(f"fuse extended bits: {hex(data[3])}")
            data = self._utils.send_passthrough(b'\x38\x00\x00\x00', True)
            print(f"calibration bits: {hex(data[3])}")
            self._utils.enable_programming(False)
        else:
            assert False, "sorry friend failed to enable programming"


    def write_fuse_bits(self, byte_in: bytes):
        self.ser.flushInput()
        if self._utils.enable_programming(True):
            data = self._utils.send_passthrough(b'\xac\xa0\x00' + byte_in, True)

        else:
            assert False, "sorry friend failed to enable programming"
