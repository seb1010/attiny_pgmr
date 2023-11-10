import serial
import time



class Prog:

    def __init__(self, device='/dev/ttyACM0'):

        self.pac = b'hello'
        self.ser = serial.Serial(device)
        self.ser.baudrate=9600 # 230400
        self.ser.timeout = 0.1
        self.ser.stopbits = 2
        
    def program_device(self, filename: str, page_size: int, release_reset:bool=True):
       if self.enable_prog():
           self.send_bytes(b'\xac\x80\x00\x00') # clear mem
           time.sleep(0.05)
           with open(filename, "r") as fp:
               data_line = fp.readline()
               while data_line != '':
                   # print(data_line)
                   if data_line[7:9] == '00':
                       self._program_line(data_line, page_size)
                   data_line = fp.readline()
           if release_reset:
               self.set_reset(True) # enabling chip


    def _program_line(self, data_line: str, page_size: int):
        line_length = int(data_line[1:3], 16) // 2
        addrs = int(data_line[3:7], 16) // 2
        data_only = data_line[9:line_length * 4 + 9]
        i = 0
        while i < line_length:
            j = 0
            while j < page_size and i < line_length:
                word_addrs = (addrs + i)  % page_size
                word_addrs_bytes = word_addrs.to_bytes(2, "big")
                data_out = int(data_only[4*i:4*i+2], 16).to_bytes(1, "big")
                time.sleep(0.002)
                self.send_bytes(b'\x40' + word_addrs_bytes + data_out)
                data_out = int(data_only[4*i+2:4*i+4], 16).to_bytes(1, "big")
                time.sleep(0.002)
                self.send_bytes(b'\x48' + word_addrs_bytes + data_out)
                j += 1
                i += 1
            page_addrs = (addrs + i - 1) // page_size * page_size
            #print(page_addrs)
            data_out = b'\x4c' + page_addrs.to_bytes(2, "big") + b'\x00'
            time.sleep(0.01)
            self.send_bytes(data_out)
            time.sleep(0.02)
            #self.send_bytes(b'\x40\x00\x07\x0a')
            #time.sleep(0.1)
            #self.send_bytes(b'\x48\x00\x07\x0b')
            #time.sleep(0.1)
            #self.send_bytes_s(b'\x4C\x00\x00\x00')
            #print("writing page: " + str(page_addrs // 32)) 
           
       
    def send_bytes_s(self, bytes_out):
        print(bytes_out.hex())
        print(bytes_out)

    def read_mem(self, start_addrs: int=0, stop_addrs: int=500, enable_prog: bool=True):
        if not enable_prog:
            pass
        elif(self.enable_prog()): # skip reading memory if we fail
            pass
        else:
            stop_addrs = start_addrs
        data_out = b''      # to enable programming

        for i, addrs in enumerate(range(start_addrs, stop_addrs)):
            bytes_out =  addrs.to_bytes(2, "big") + b'\x00'
            mr_byte = self.send_and_read_byte(b'\x20' + bytes_out)
            time.sleep(0.01)
            data_out += mr_byte
            mr_byte = self.send_and_read_byte(b'\x28' + bytes_out)
            time.sleep(0.01)
            data_out += mr_byte

        # self.print_mem(data_out)
        return data_out

    def print_mem(self, bytes_in: bytes):
        for i, byte in enumerate(bytes_in):
            print(format(byte, '2X'), end='')
            if i % 0x10 == 0x0f:
                print('')
           
    def verify_prog(self, filename: str, enable_prog:bool=True):
       all_match = True
       if enable_prog:
           prog_enabled = self.enable_prog()
       else:
           prog_enabled = True
       if not prog_enabled:
           print("failed to read from dut. ABORTING")
           all_match = False
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
                      # print("line " + str(line_num) + " is a match")
                       pass
                   else:
                       print("LINE " + str(line_num) + " MISSMATCH!!")
                       print("file: " + str(file_bytes))
                       print("dut: " + str(bytes_dut))            
                       all_match = False
               line_num += 1
               data_line = fp.readline()
       self.set_reset(True) # enabling chip
       if all_match:
           print("Program verified. All Correct!")
       return all_match

    def compare_lines(self, dut_data: bytes, file_data: str):
        pass 


    def read_fuse_bytes(self):
        if(self.enable_prog()):
            requests = [
                {"data": b'\x50\x00\x00\x00', "desc": "Fuse Bits"},
                {"data": b'\x58\x08\x00\x00', "desc": "Fuse High Bits"},
                {"data": b'\x50\x08\x00\x00', "desc": "Fuse Extended Bits"}
            ]
            for request in requests:
                print(f"{request['desc']}: ", end='')
                fuse_byte = self.send_and_read_byte(request['data'])
                print(fuse_byte.hex())
        else:
            print("error couldn't talk to device")

    def c_prog_sync(self):
        i = 0
        sync_estab = False
        while True and not sync_estab:
            self.ser.write(b'\x55')
            bb = self.ser.read(size=1)
            old_bb = bb
            while bb != b'':
                old_bb = bb
                bb = self.ser.read(size=1)
            i += 1
            if old_bb == b'\x69':
                sync_estab = True
            else:
                print("failed to sync with " + str(i) + " tries")
      
        return sync_estab
                 
            
            
    def enable_prog(self) -> bool:
        time.sleep(0.05)
        self.set_reset(True)
        time.sleep(0.05)
        self.set_reset(False)
        time.sleep(0.05)
 
        bytes_out = b'\xac\x53\x00'
        mr_byte = self.send_and_read_byte(bytes_out)
        self.send_bytes(b'\x00')
        #self.send_bytes(b'\x00')
        if mr_byte == b'\x53':
            ret = True
            print("Programming Enabled!")
            time.sleep(0.1)
        else:
            ret = False
            print("Failed to Enable Programming")

        return ret   


    def say_hi(self, data: bytes = b'a'):
        self.ser.write(data)


#class Utils:
#    def __init__():
#        pass

    def send_bytes(self, data_out: bytes, and_read: bool = False):
        """ send between 1 and 32 bytes inclusive """
        if len(data_out) < 1 or len(data_out) > 32:
            print("data length not in range")
            ret = -1
        else:
            header = len(data_out) | 0xE0
            if and_read:
                header = header & ~0x20
            packet = header.to_bytes(1, "big") + data_out
          #  for p in packet:
          #      self.ser.write(p.to_bytes(1, "big"))
          #      time.sleep(100E-6)
            self.ser.write(packet)
            ret = 0
            # print(packet)
        return ret


    def send_and_read_byte(self, data_out: bytes, read_num: int = 1):
        """sends between 1 and 32 bytes and reads the last one"""
        status = self.send_bytes(data_out, and_read=True)
        if status == 0:
            bytes_back = self.ser.read(read_num)
            if len(bytes_back) != read_num:
                print("error reading bytes. timeout reached")
        else:
             bytes_back = b''

        return bytes_back

    def set_reset(self, set_high: bool=True):
        data_out = (set_high << 3) + 0x05
        byte_out = data_out.to_bytes(1, "big")
        self.ser.write(byte_out)

