
import struct

class UBX(object):
    def __init__(self):
        self.state = 'IDLE'
        self.data = ""
        self.id = 0
        self.len = 0

    def update(self):
        pass

    def receive(self):
        print "UBX packet %02x %02x len %d" % (self.type, self.id, self.len)
        if self.type == 0x01 and self.id == 0x02:
            (itow, lon, lat, height, hs1, hacc, vacc) = struct.unpack("IiiiiII", self.data)
            print "NAV POSLLH: %u %i %i %i %i %u %u" % (itow, lon, lat, height, hs1, hacc, vacc)

    def parse(self, data):
        if self.state == 'IDLE':
            if data == 0xb5:
                self.state = 'START'
        elif self.state == 'START':
            if data == 0x62:
                self.state = 'SYNC'
            else:
                self.state = 'IDLE'
        elif self.state == 'SYNC':
            self.data = ""
            self.type = data
            self.state = 'TYPE'
        elif self.state == 'TYPE':
            self.id = data
            self.state = 'LEN1'
        elif self.state == 'LEN1':
            self.len = data
            self.state = 'LEN2'
        elif self.state == 'LEN2':
            self.len += data << 8
            if not self.len:
                self.state = 'IDLE'
            self.state = 'RECV'
        elif self.state == 'RECV':
            self.data += chr(data)
            if len(self.data) == self.len:
                self.state = 'CHECK'
        elif self.state == 'CHECK':
            self.receive()
            self.state = 'IDLE'





