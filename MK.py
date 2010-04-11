
import os
import termios

_zero = ord('=')

def board_name(addr):
    if addr == "a":
        return "<any>"
    if addr == "b":
        return "FlightCtrl"
    if addr == "c":
        return "NaviCtrl"
    if addr == "d":
        return "MK3Mag"
    return "<unknown>"


def calc_crc(data):
    counter = ord('#')
    for char in data:
        byte = ord(char)
        counter += byte
    counter %= 4096

    return chr( _zero + counter/64 ) + chr( _zero + counter%64 )


def decode(code):
    data = []
    while len(code) >= 4:
        a = ord(code[0]) - _zero
        b = ord(code[1]) - _zero
        c = ord(code[2]) - _zero
        d = ord(code[3]) - _zero

        x = ((a & 0x3f) << 2) | ((b & 0xf0) >> 4)
        y = ((b & 0x0f) << 4) | ((c & 0xfc) >> 2)
        z = ((c & 0x03) << 6) |  (d & 0x3f)
        if x > 127: x -= 256
        if y > 127: y -= 256
        if z > 127: z -= 256

        data += [x, y, z]
        code = code[4:]
    return data

def encode(data):
    code = ""
    while len(data):
        while len(data)<3: data.append(0)
        (x, y, z) = data[0:3]
        if x < 0: x += 256
        if y < 0: y += 256
        if z < 0: z += 256

        a = (x & 0xfc) >> 2
        b = ((x & 0x03) << 4) | ((y & 0xf0) >> 4)
        c = ((y & 0x0f) << 2) | ((z & 0xc0) >> 6)
        d = z & 0x3f

        code += chr( _zero + a )
        code += chr( _zero + b )
        code += chr( _zero + c )
        code += chr( _zero + d )
        data = data[3:]

    return code


def decode_str(data):
    s = ""
    for c in data:
        if not c: break
        s += chr(c)
    return s.strip()

def encode_str(s, l):
    data = []
    for c in s:
        data.append( ord(c) )
        l -= 1
    while l:
        data.append( 0 )
        l -= 1
    return data

class MK(object):
    pass

def send_packet(link, addr, id, contents):
    code = encode(contents)
    packet = addr + id + code
    crc = calc_crc(packet)
    link.write('#')
    link.write(packet)
    link.write(crc)
    link.write('\r')

def recv_packet(link, packet):
    crc = calc_crc(packet[:-2])
    if crc != packet[-2:]:
        print "CRC!", crc, packet
        return

    addr = packet[0]
    id = packet[1]
    data = decode(packet[2:-2])


    if id == 'w':
        #print "HeadingRequest", data
        pass

    elif id == 'A':
        index = data[0]
        label = decode_str(data[1:])
        print "Label", index, '"%s"' % label
        send_packet(link, 'a', 'a', [ index+1 ])

    elif id == 'M':
        if data[0] == 1:
            print "Mixer write OK"
        else:
            print "Mixer write ERROR"

    elif id == 'N':
        rev = data[0]
        label = decode_str(data[1:13])
        print "Mixer", rev, label
        if rev != 1:
            raise "unkown mixer revision"
        table = data[13:]
        print "Motor", "Gas", "Nick", "Roll", "Yaw"
        motor = 1
        while table[0]:
            print motor, table[0], table[1], table[2], table[3]
            table = table[4:]
            motor += 1

    elif id == 'Q':
        print "Settings", data

    elif id == 'V':
        print "Version", "%s %d.%d%s" % ( board_name(addr),
                data[0], data[1], chr( ord('a')+ data[4]) )
        print "Serial", data[2], data[3]

    else:
        print "unknown", addr, id, data


def connect(device):
    link = open(device, "r+b")
    fd = link.fileno()

    tca = termios.tcgetattr( link.fileno() )
    tca[4] = termios.B57600
    tca[5] = termios.B57600
    termios.tcsetattr( link.fileno(), termios.TCSAFLUSH, tca )

    return link

def receive(link):
    state = 0
    line = ""
    while True:
        data = link.read( 1 )
        if state == 0:
            if data == '#':
                state = 1
            elif data == '\r':
                print "line in:", line
                line = ""
            else:
                line += data
        elif state == 1:
            if data == '\r':
                recv_packet(link, line)
                line = ""
                state = 0
            else:
                line += data



#    data = link.read( 64 )
#    print "in:", data
#    print "-->", decode(data[2:-3])

#for line in link:
#    print line

