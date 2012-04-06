# -*- coding: utf-8 -*-

import asyncore
import os
import select
import struct
import termios
import time

_zero = ord('=')


class Setting(object):
    def __init__(self, name):
        self.name = name
        self.value = None
        self.struct = struct.Struct( "B" )

    def assign(self, value):
        self.value = value

    def __str__(self):
        return str(self.value)

    def pack(self):
        return self.struct.pack(self.value)

    def unpack(self, data):
        self.value, = self.struct.unpack(data)


class AnalogSetting(Setting):
    def __init__(self, name, min=0, max=255):
        Setting.__init__(self, name)
        self.min = min
        self.max = max

    def assign(self, value):
        value = int(value)
        if value < self.min: raise ValueError(
                "invalid value for %s: %d is lower than the limit (%d)!" % (
                    self.name, value, self.min) )
        if value > self.max: raise ValueError(
                "invalid value for %s: %d is higher than the limit (%d)!" % (
                    self.name, value, self.max) )
        self.value = value

class AnalogPotiSetting(AnalogSetting):
    def assign(self, value):
        if len(value) == 5 and value[0:5]=="POTI":
            value = 255 - int(value[4:5]) - 1
        else:
            AnalogSetting.assign(self, value)

    def __str__(self):
        if self.value < 248:
            return str(self.value)
        else:
            return "POTI" + str(255 - self.value + 1)


class BinarySetting(Setting):
    def assign(self, value):
        self.value = int(value)

    def __str__(self):
        return bin(self.value)


class ChoiceSetting(BinarySetting):
    def __init__(self, name, choices):
        BinarySetting.__init__(self, name)
        self.choices = choices
        self.reverse = {}
        for k,v in choices.iteritems():
            self.reverse[v] = k

    def assign(self, value):
        self.value = self.reverse[ value ]

    def __str__(self):
        return self.choices[ self.value ]


class LabeledSetting(BinarySetting):
    def __init__(self, name, labels):
        BinarySetting.__init__(self, name)
        self.labels = labels

    def __str__(self):
        mask = 1
        index = 0

        while self.value >= mask:
            if self.value & mask:
                print self.labels[index]



class StringSetting(Setting):
    def __init__(self, name, length):
        Setting.__init__(self, name)
        self.struct = struct.Struct( str(length) + "s" )

    def unpack(self, data):
        Setting.unpack(self, data)
        # end string at the first \0 byte:
        self.value = self.value[: self.value.find('\0') ]


class Configuration(object):
    def __init__(self, version):
        self.version = version
        self.setting = {}
        self.structure = Configuration.settings(version)
        for s in self.structure:
            if s.name == "Name":
                self.name = str(s)
            else:
                self.setting[ s.name ] = s

    def set(self, key, value):
        self.setting[key].assign(value)

    def value(self, key):
        self.setting[key].value

    def get(self, key):
        str( self.setting[key] )

    def pack(self):
        data = ""
        for s in self.structure:
            data += s.pack()
        return data

    def unpack(self, data):
        for s in self.structure:
            s.unpack( data[: s.struct.size ] )
            data = data[ s.struct.size :]

    @classmethod
    def settings(cls, version):
        if version < 82: raise ValueError("unsupported version")
        if version > 85: raise ValueError("unsupported version")
        list = [
            AnalogSetting("Channel_Gas", max=15),
            AnalogSetting("Channel_Gier", max=15),
            AnalogSetting("Channel_Nick", max=15),
            AnalogSetting("Channel_Roll", max=15),
            AnalogSetting("POTI1", max=15), AnalogSetting("POTI2", max=15),
            AnalogSetting("POTI3", max=15), AnalogSetting("POTI4", max=15),
            AnalogSetting("POTI5", max=15), AnalogSetting("POTI6", max=15),
            AnalogSetting("POTI7", max=15), AnalogSetting("POTI8", max=15),
            BinarySetting("GlobalConfig"), # LabeledSetting( "GlobalConfig", ["AIRPRESS_SENSOR", "HEIGHT_SWITCH", "HEADING_HOLD", "COMPASS_ACTIVE", "COMPASS_FIX", "GPS_ACTIVE", "AXIS_COUPLING_ACTIVE", "ROTARY_RATE_LIMITER"]),
            AnalogSetting("Hoehe_MinGas", max=100),
            AnalogPotiSetting("Luftdruck_D"),
            AnalogPotiSetting("MaxHoehe", max=32),
            AnalogPotiSetting("Hoehe_P", max=32),
            AnalogSetting("Hoehe_Verstaerkung", max=50),
            AnalogPotiSetting("Hoehe_ACC_Wirkung"),
            AnalogSetting("Hoehe_HoverBand"),
            AnalogPotiSetting("Hoehe_GPS_Z"),
            AnalogSetting("Hoehe_StickNeutralPoint"),
            AnalogSetting("Stick_P", min=1, max=6),
            AnalogSetting("Stick_D", max=64),
            AnalogSetting("Gier_P", min=1, max=20),
            AnalogSetting("Gas_Min", max=32),
            AnalogSetting("Gas_Max", min=33),
            AnalogSetting("GyroAccFaktor", min=1, max=64),
            AnalogPotiSetting("KompassWirkung", max=32),
            AnalogPotiSetting("Gyro_P", min=10),
            AnalogPotiSetting("Gyro_I"),
            AnalogPotiSetting("Gyro_D"),
            AnalogPotiSetting("Gyro_Gier_P", min=10),
            AnalogPotiSetting("Gyro_Gier_I")
        ]
        if version >= 84:
            list += [ AnalogSetting("Gyro_Stability", max=16) ]
        list += [
            AnalogSetting("UnterspannungsWarnung"),
            AnalogSetting("NotGas"),      # Gaswert bei Empängsverlust
            AnalogSetting("NotGasZeit"),  # Zeitbis auf NotGas geschaltet wird, wg. Rx-Problemen
            ChoiceSetting("Receiver", {
                0: "PPM", 1: "SPEKTRUM", 2: "SPEKTRUM_HI_RES", 3: "SPEKTRUM_LO_RES",
                4: "JETI", 5: "ACT_DSL"
            }),
            AnalogPotiSetting("I_Faktor"),
            AnalogPotiSetting("UserParam1"),
            AnalogPotiSetting("UserParam2"),
            AnalogPotiSetting("UserParam3"),
            AnalogPotiSetting("UserParam4"),
            AnalogPotiSetting("ServoNickControl"), # Stellung des Servos
            AnalogSetting("ServoNickComp"),        # Einfluss Gyro/Servo
            AnalogSetting("ServoNickMin"),
            AnalogSetting("ServoNickMax"),
            AnalogPotiSetting("ServoRollControl"), # Stellung des Servos
            AnalogSetting("ServoRollComp"),
            AnalogSetting("ServoRollMin"),
            AnalogSetting("ServoRollMax"),
            AnalogSetting("ServoNickRefresh")      # Speed of the Servo
        ]
        if version >= 85:
            list += [
                AnalogSetting("ServoManualControlSpeed"),
                AnalogSetting("CamOrientation")
            ]
        list += [
            AnalogPotiSetting("Servo3"),   # Value or mapping of the Servo Output
            AnalogPotiSetting("Servo4"),   # Value or mapping of the Servo Output
            AnalogPotiSetting("Servo5"),   # Value or mapping of the Servo Output
            AnalogPotiSetting("LoopGasLimit"),     # max. Gas während Looping
            AnalogSetting("LoopThreshold"),        # Schwelle für Stickausschlag
            AnalogSetting("LoopHysterese"),        # Hysterese für Stickausschlag
            AnalogPotiSetting("AchsKopplung1"),    # Faktor, mit dem Gier die Achsen Roll und Nick koppelt (NickRollMitkopplung)
            AnalogPotiSetting("AchsKopplung2"),           # Faktor, mit dem Nick und Roll verkoppelt werden
            AnalogPotiSetting("CouplingYawCorrection"),   # Faktor, mit dem Nick und Roll verkoppelt werden
            AnalogSetting("WinkelUmschlagNick"),   # 180°-Punkt
            AnalogSetting("WinkelUmschlagRoll"),   # 180°-Punkt
            AnalogSetting("GyroAccAbgleich"),      # 1/k  (Koppel_ACC_Wirkung)
            AnalogSetting("Driftkomp"),
            AnalogPotiSetting("DynamicStability"),
            AnalogPotiSetting("UserParam5"),
            AnalogPotiSetting("UserParam6"),
            AnalogPotiSetting("UserParam7"),
            AnalogPotiSetting("UserParam8"),
            BinarySetting("J16Bitmask"),
            AnalogPotiSetting("J16Timing"),
            BinarySetting("J17Bitmask"),
            AnalogPotiSetting("J17Timing"),
            BinarySetting("WARN_J16_Bitmask"),
            BinarySetting("WARN_J17_Bitmask"),
            AnalogPotiSetting("NaviGpsModeControl"),   # Parameters for the Naviboard
            AnalogPotiSetting("NaviGpsGain"),
            AnalogPotiSetting("NaviGpsP"),
            AnalogPotiSetting("NaviGpsI"),
            AnalogPotiSetting("NaviGpsD"),
            AnalogSetting("NaviGpsPLimit"),
            AnalogSetting("NaviGpsILimit"),
            AnalogSetting("NaviGpsDLimit"),
            AnalogPotiSetting("NaviGpsACC"),
            AnalogSetting("NaviGpsMinSat"),
            AnalogSetting("NaviStickThreshold"),
            AnalogPotiSetting("NaviWindCorrection"),
            AnalogPotiSetting("NaviSpeedCompensation"),
            AnalogPotiSetting("NaviOperatingRadius"),
            AnalogPotiSetting("NaviAngleLimitation"),
            AnalogSetting("NaviPH_LoginTime"),
            AnalogPotiSetting("ExternalControl")   # for serial Control
        ]
        if version >= 84:
            list += [
                 AnalogSetting("OrientationAngle"), # Where is the front-direction?
                 AnalogPotiSetting("OrientationModeControl") # switch for CareFree
            ]
        if version >= 85:
            list += [
                 AnalogSetting("MotorSafetySwitch", max=15)
            ]
        list += [
            BinarySetting("BitConfig"), # LabeledSetting("BitConfig", ["LOOP_OBEN", "LOOP_UNTEN", "LOOP_LINKS", "LOOP_RECHTS", "MOTOR_BLINK", "MOTOR_OFF_LED1", "MOTOR_OFF_LED2"])
            BinarySetting("ServoCompInvert"),      # 0x01 = Nick, 0x02 = Roll
            BinarySetting("ExtraConfig"),          # LabeledSetting("ExtraConfig", ["HEIGHT_LIMIT", "VARIO_BEEP", "SENSITIVE_RC", "3_3V_REFERENCE"])
            StringSetting("Name", 12)
        ]
        return list

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
        self.name = "Flight-Ctrl"
        self.redirect = 0

class NaviCtrl(Board):
    def __init__(self):
        Board.__init__(self)
        self.addr = 'c'
        self.name = "Navi-Ctrl"

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
        config = Configuration(version)
        config.unpack( reply )
        return set, version, config

class CmdSetSettings(Command):
    """
    Write a new configuration.
    Parameters are the same as the reply of CmdGetSettings
    """
    def __init__(self, board, set, version, config):
        data = struct.pack("2B", set, version)
        data += config.pack()
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

class CmdMotorTest(Command):
    def __init__(self, board, list):
        while len(list) < 16: list += [0]
        payload = struct.pack("16B", *list)
        Command.__init__(self, board, 't', 'T', payload)

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
        if self.debug:
            print "selecting", board.name

        # cancel existing redirections
        escape = "#" + chr(0x1b) + chr(0x1b)
        escape += chr(0x55) + chr(0xaa) + chr(0) + '\r'
        self.send_data(escape, 0.1)

        if board.redirect != None:
            redirect = Command(self.nc, 'u', None, struct.pack('B', board.redirect))
            redirect = CmdRedirect(self.nc, board)
            self.send_data(redirect.get_frame(), 0.1)

        self.selected = board
        # put the command processor in defined state:
        self.send_data('\r', 0)
        # read any pending data
        self.recv_data(1024, 0.2)


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


