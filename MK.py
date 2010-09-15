# -*- coding: utf-8 -*-

import asyncore
import os
import select
import struct
import termios
import time

_zero = ord('=')


settings = [
    "Channel_Gas", "Channel_Gier", "Channel_Nick", "Channel_Roll",
    "POTI1", "POTI2", "POTI3", "POTI4", "POTI5", "POTI6", "POTI7", "POTI8",
    "GlobalConfig",            # 0x01=Höhenregler aktiv,0x02=Kompass aktiv, 0x04=GPS aktiv, 0x08=Heading Hold aktiv
    "Hoehe_MinGas",            # Wert : 0-100
    "Luftdruck_D",             # Wert : 0-250
    "MaxHoehe",                # Wert : 0-32
    "Hoehe_P",                 # Wert : 0-32
    "Hoehe_Verstaerkung",      # Wert : 0-50
    "Hoehe_ACC_Wirkung",       # Wert : 0-250
    "Hoehe_HoverBand",         # Wert : 0-250
    "Hoehe_GPS_Z",             # Wert : 0-250
    "Hoehe_StickNeutralPoint", # Wert : 0-250
    "Stick_P",                 # Wert : 1-6
    "Stick_D",                 # Wert : 0-64
    "Gier_P",                  # Wert : 1-20
    "Gas_Min",                 # Wert : 0-32
    "Gas_Max",                 # Wert : 33-250
    "GyroAccFaktor",           # Wert : 1-64
    "KompassWirkung",          # Wert : 0-32
    "Gyro_P",                  # Wert : 10-250
    "Gyro_I",                  # Wert : 0-250
    "Gyro_D",                  # Wert : 0-250
    "Gyro_Gier_P",             # Wert : 10-250
    "Gyro_Gier_I",             # Wert : 0-250
    "UnterspannungsWarnung",   # Wert : 0-250
    "NotGas",                  # Wert : 0-250     //Gaswert bei Empängsverlust
    "NotGasZeit",              # Wert : 0-250     // Zeitbis auf NotGas geschaltet wird, wg. Rx-Problemen
    "Receiver",                # 0= Summensignal, 1= Spektrum, 2 =Jeti, 3=ACT DSL, 4=ACT S3D
    "I_Faktor",                # Wert : 0-250
    "UserParam1",              # Wert : 0-250
    "UserParam2",              # Wert : 0-250
    "UserParam3",              # Wert : 0-250
    "UserParam4",              # Wert : 0-250
    "ServoNickControl",        # Wert : 0-250     // Stellung des Servos
    "ServoNickComp",           # Wert : 0-250     // Einfluss Gyro/Servo
    "ServoNickMin",            # Wert : 0-250     // Anschlag
    "ServoNickMax",            # Wert : 0-250     // Anschlag
    "ServoRollControl",        # Wert : 0-250     // Stellung des Servos
    "ServoRollComp",           # Wert : 0-250
    "ServoRollMin",            # Wert : 0-250
    "ServoRollMax",            # Wert : 0-250
    "ServoNickRefresh",        # Speed of the Servo
    "Servo3",                  # Value or mapping of the Servo Output
    "Servo4",                  # Value or mapping of the Servo Output
    "Servo5",                  # Value or mapping of the Servo Output
    "LoopGasLimit",            # Wert: 0-250  max. Gas während Looping
    "LoopThreshold",           # Wert: 0-250  Schwelle für Stickausschlag
    "LoopHysterese",           # Wert: 0-250  Hysterese für Stickausschlag
    "AchsKopplung1",           # Wert: 0-250  Faktor, mit dem Gier die Achsen Roll und Nick koppelt (NickRollMitkopplung)
    "AchsKopplung2",           # Wert: 0-250  Faktor, mit dem Nick und Roll verkoppelt werden
    "CouplingYawCorrection",   # Wert: 0-250  Faktor, mit dem Nick und Roll verkoppelt werden
    "WinkelUmschlagNick",      # Wert: 0-250  180°-Punkt
    "WinkelUmschlagRoll",      # Wert: 0-250  180°-Punkt
    "GyroAccAbgleich",         # 1/k  (Koppel_ACC_Wirkung)
    "Driftkomp",
    "DynamicStability",
    "UserParam5",              # Wert : 0-250
    "UserParam6",              # Wert : 0-250
    "UserParam7",              # Wert : 0-250
    "UserParam8",              # Wert : 0-250
    "J16Bitmask",              # for the J16 Output
    "J16Timing",               # for the J16 Output
    "J17Bitmask",              # for the J17 Output
    "J17Timing",               # for the J17 Output
    "WARN_J16_Bitmask",        # for the J16 Output
    "WARN_J17_Bitmask",        # for the J17 Output
    "NaviGpsModeControl",      # Parameters for the Naviboard
    "NaviGpsGain",
    "NaviGpsP",
    "NaviGpsI",
    "NaviGpsD",
    "NaviGpsPLimit",
    "NaviGpsILimit",
    "NaviGpsDLimit",
    "NaviGpsACC",
    "NaviGpsMinSat",
    "NaviStickThreshold",
    "NaviWindCorrection",
    "NaviSpeedCompensation",
    "NaviOperatingRadius",
    "NaviAngleLimitation",
    "NaviPH_LoginTime",
    "ExternalControl",        # for serial Control
    "BitConfig",          # (war Loop-Cfg) Bitcodiert: 0x01=oben, 0x02=unten, 0x04=links, 0x08=rechts / wird getrennt behandelt
    "ServoCompInvert",    # 0x01 = Nick, 0x02 = Roll   0 oder 1  // WICHTIG!!! am Ende lassen
    "ExtraConfig",        # bitcodiert
#   char Name[12];
    ]

def calc_crc(data):
    counter = ord('#')
    for char in data:
        byte = ord(char)
        counter += byte
    counter %= 4096

    return chr( _zero + counter/64 ) + chr( _zero + counter%64 )

triplet = struct.Struct('BBB')

def decode(code):
    data = ""
    while len(code) >= 4:
        a = ord(code[0]) - _zero
        b = ord(code[1]) - _zero
        c = ord(code[2]) - _zero
        d = ord(code[3]) - _zero

        x = ((a & 0x3f) << 2) | ((b & 0xf0) >> 4)
        y = ((b & 0x0f) << 4) | ((c & 0xfc) >> 2)
        z = ((c & 0x03) << 6) |  (d & 0x3f)

        data += triplet.pack(x, y, z)
        code = code[4:]
    return data

def encode(data):
    code = ""
    while len(data):
        while len(data)<3: data += chr(0)
        (x, y, z) = triplet.unpack(data[:3])

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



class Board(object):
    def __init__(self):
        self.redirect = None


class FlightCtrl(Board):
    def __init__(self):
        Board.__init__(self)
        self.addr = 'b'
        self.name = "FlightCtrl"
        self.redirect = 0

class NaviCtrl(Board):
    def __init__(self):
        Board.__init__(self)
        self.addr = 'c'
        self.name = "NaviCtrl"

class MK3Mag(Board):
    def __init__(self):
        Board.__init__(self)
        self.addr = 'd'
        self.name = "MK3Mag"
        self.redirect = 1

class MKGPS(Board):
    def __init__(self):
        Board.__init__(self)
        self.name = "MKGPS"
        self.redirect = 2



class Command(object):
    """
    A command which can be sent to the Mikrokopter.
    """
    def __init__(self, board, id, reply_id, data):
        """
        Create a new Command.
        You have to specify the destination board and the ID code of the command
        and the reply along with the payload data.
        """
        self.board = board
        self.id = id
        self.tries = 3
        packet = board.addr + id + encode(data)
        self.frame = '#' + packet + calc_crc(packet) + '\r'
        self.reply_id = reply_id
        self.reply = None

    def get_frame(self):
        """
        Get the encoded data frame which is sent to the MikroKopter over the
        serial line.
        """
        return self.frame

    def parse_reply(self, reply):
        """
        Decode the reply message.
        """
        return reply

class CmdRedirect(Command):
    """
    The command to redirect the serial line from NaviCtrl to one of the
    other boards.
    It has no reply.
    """
    def __init__(self, proxy, target):
        data = struct.pack('B', target.redirect)
        Command.__init__(self, proxy, 'u', None, data)

class CmdVersion(Command):
    """
    Obtain version information.
    Result is returned as a string (e.g. "0.78f").
    """
    def __init__(self, board):
        Command.__init__(self, board, 'v', 'V', "")

    def parse_reply(self, reply):
        major, minor, patch = struct.unpack("BBxxBx6x", reply)
        return "%d.%d%s" % ( major, minor, chr( ord('a') + patch) )

class CmdErrorMsg(Command):
    """
    Obtain error message from NaviCtrl.
    """
    def __init__(self, board):
        Command.__init__(self, board, 'e', 'E', "")

class CmdAnalogLabel(Command):
    """
    Obtain the label of an analog debug signal.
    Result is returned as string.
    """
    def __init__(self, board, index):
        Command.__init__(self, board, 'a', 'A', struct.pack('B', index))

    def parse_reply(self, reply):
        index, name = struct.unpack("B 16s x", reply)
        return name

class CmdGetDebug(Command):
    """
    Obtain analog debug signals.
    Result is returned as a list of integers.
    First is a unsigned with 16 bits, followed by 32 signed integers.
    Labels of the 32 analog values can be obtained with CmdAnalogLabel.
    """
    def __init__(self, board, interval):
        Command.__init__(self, board, 'd', 'D', struct.pack('B', interval))
        if not interval:
            self.reply_id = None

    def parse_reply(self, reply):
        return struct.unpack("<H32h", reply)

class CmdGetSettings(Command):
    """
    Obtain settings.
    Use index 0xff to get the current configuration.
    Result is returned as a tuple (set, version, setting).
    set is the currently active configuration set, setting is a map
    with the actual configuration.
    """
    def __init__(self, board, index):
        Command.__init__(self, board, 'q', 'Q', struct.pack('B', index))

    def parse_reply(self, reply):
        set, version = struct.unpack("2B", reply[0:2])
        reply = reply[2:]
        setting = {}
        for name in settings:
            setting[name], = struct.unpack('B', reply[0])
            reply = reply[1:]
        return set, version, setting

class CmdSetSettings(Command):
    """
    Write a new configuration.
    Parameters are the same as the reply of CmdGetSettings
    """
    def __init__(self, board, set, version, setting):
        data = struct.pack("2B", set, version)
        for name in settings:
            data += struct.pack("B", setting[name])
        Command.__init__(self, board, 's', 'S', data)

    def parse_reply(self, reply):
        set, = struct.unpack("Bxx", reply)
        return set

class CmdGetChannels(Command):
    def __init__(self, board):
        Command.__init__(self, board, 'p', 'P', "")

    def parse_reply(self, reply):
        return struct.unpack("<8h", reply[:16])


class CmdSetMotorMixer(Command):
    def __init__(self, board, name, data):
        payload = struct.pack("B 12s", 1, name)
        for d in data:
            payload += struct.pack("b", d)
        Command.__init__(self, board, 'm', 'M', payload)

    def parse_reply(self, reply):
        status = struct.unpack("B 2x", reply)
        return status


class CmdGetMotorMixer(Command):
    def __init__(self, board):
        Command.__init__(self, board, 'n', 'N', "")

    def parse_reply(self, reply):
        rev, name = struct.unpack("B 12s", reply[:13])
        if rev != 1:
            raise "Unknown mixer revision"
        reply = reply[13:]
        data = []
        while len(reply) >= 4:
            data += struct.unpack("4b", reply[:4])
        return name, data

class CmdReset(Command):
    def __init__(self, board):
        Command.__init__(self, board, 'R', None, "")


class MK(object):
    def __init__(self, name):
        self.name = name
        self.debug = False

        self.device = None
        self.reconnect()

        self.fc = FlightCtrl()
        self.nc = NaviCtrl()
        self.gps = MKGPS()
        self.mk3mag = MK3Mag()

    def reconnect(self):
        if self.device:
            self.device.close()
            if self.debug:
                print "reconnecting"

        self.device = open(self.name, "r+b")
        fd = self.device.fileno()
        self.selected = None
        self.line = ""

        tca = termios.tcgetattr( fd )
        tca[4] = termios.B57600
        tca[5] = termios.B57600
        termios.tcsetattr( fd, termios.TCSAFLUSH, tca )

    def recv_bytes(self, bytes):
        return os.read(self.device.fileno(), bytes)

    def recv_timeout(self, bytes, timeout):
        r, w, e = select.select([self.device.fileno()], [], [], timeout)
        if len(r):
            return self.recv_bytes(bytes)
        else:
            return ""

    def recv_data(self, bytes, timeout):
        until = time.time() + timeout
        data = ""
        while len(data) < bytes and timeout > 0:
            received = self.recv_timeout(bytes - len(data), timeout)
            data += received
            if self.debug:
                print "received", len(received)
            if not received: break
            timeout = until - time.time()
        return data

    def recv_packet(self, board, id, timeout):

        until = time.time() + timeout

        received = ""
        while timeout > 0:
            if not received:
                received = self.recv_timeout(4096, timeout)
                if not received: return None
                if self.debug:
                    print "received", len(received)
                timeout = until - time.time()

            c = received[0]
            received = received[1:]

            if c != '\r':
                self.line += c
                continue

            if len(self.line) == 0: continue

            if self.line[0] != '#':
                if self.debug:
                    print "line in:", self.line
                self.line = ""
                continue

            packet = self.line[1:]
            self.line = ""
            crc = calc_crc(packet[:-2])
            if crc != packet[-2:]:
                if self.debug:
                    print "CRC error", crc, packet
                continue

            r_addr = packet[0]
            r_id = packet[1]
            r_data = decode(packet[2:-2])

            if r_addr == board.addr and r_id == id:
                return r_data
            else:
                if self.debug and r_addr == 'd' and r_id == 'w':
                    continue # ignore heading request
                if self.debug:
                    print "unexpected packet", r_addr, r_id, r_data
        return None

    def send_bytes(self, data):
        return os.write(self.device.fileno(), data)

    def send_data(self, data, wait):
        wrote = self.send_bytes(data)
        if self.debug:
            print "sent", wrote

        if wait:
            time.sleep(wait)


    def select_board(self, board):
        if board == self.selected: return
        #print "select_board"

        # cancel existing redirections
        escape = "#" + chr(0x1b) + chr(0x1b)
        escape += chr(0x55) + chr(0xaa) + chr(0) + '\r'
        self.send_data(escape, 0.1)

        if board.redirect != None:
            redirect = Command(self.nc, 'u', None, struct.pack('B', board.redirect))
            redirect = CmdRedirect(self.nc, board)
            self.send_data(redirect.get_frame(), 0.1)

        self.selected = board


    def send_cmd(self, cmd):
        self.select_board(cmd.board)
        self.line = ""

        if not cmd.reply_id:
            self.send_data(cmd.get_frame(), 0)
            return None

        reply = None
        tries = cmd.tries
        while not reply and tries:
            self.send_data(cmd.get_frame(), 0)
            reply = self.recv_packet(cmd.board, cmd.reply_id, 0.5)
            tries -= 1

        if reply:
            return cmd.parse_reply(reply)
        else:
            return None

    def recv_cmd(self, cmd):
        reply = self.recv_packet(cmd.board, cmd.reply_id, 0.5)
        if reply:
            return cmd.parse_reply(reply)
        else:
            return None


