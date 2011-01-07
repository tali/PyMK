#!/usr/bin/python
# Test suite for IntelHex class.

import array
from cStringIO import StringIO
import os
import sys
import tempfile
import unittest

import intelhex
from intelhex import IntelHex


##
# Data for tests

hex8 = '''\
:1004E300CFF0FBE2FDF220FF20F2E120E2FBE6F396
:1004F3000A00FDE0E1E2E3B4E4E5BAE6E7B3BFE80E
:10050300E9EAEBECEDEEEFF0F1F2F3F4F5F6F7F8E0
:10051300F9FCFEFF00C0C1C2C3A5C4C5AAC6C7B2C9
:10052300AFC8C9CACBCCCDCECFD0D1D2D3D4D5D6F8
:07053300D7D8D9DCDEDF00A0
:10053A0078227C007D007BFF7A0479F57E007F2398
:10054A0012042F78457C007D007BFF7A0579187E9E
:10055A00007F2212042F759850438920758DDDD2B1
:10056A008ED2996390017BFF7A0479E31200658049
:01057A00FE82
:030000000205A254
:0C05A200787FE4F6D8FD75817A02053AF6
:10035F00E709F608DFFA8046E709F208DFFA803E80
:10036F0088828C83E709F0A3DFFA8032E309F6086D
:10037F00DFFA8078E309F208DFFA807088828C83D5
:10038F00E309F0A3DFFA806489828A83E0A3F60889
:10039F00DFFA805889828A83E0A3F208DFFA804C63
:1003AF0080D280FA80C680D4806980F2803380103A
:1003BF0080A680EA809A80A880DA80E280CA8033A3
:1003CF0089828A83ECFAE493A3C8C582C8CCC5831B
:1003DF00CCF0A3C8C582C8CCC583CCDFE9DEE780EB
:1003EF000D89828A83E493A3F608DFF9ECFAA9F06A
:1003FF00EDFB2289828A83ECFAE0A3C8C582C8CCC0
:10040F00C583CCF0A3C8C582C8CCC583CCDFEADED8
:10041F00E880DB89828A83E493A3F208DFF980CC3A
:10042F0088F0EF60010E4E60C388F0ED2402B40433
:10043F000050B9F582EB2402B4040050AF232345DA
:06044F0082239003AF734D
:10000300E576246AF8E60576227867300702786A8F
:10001300E475F0011204AD0204552000EB7F2ED2EB
:10002300008018EF540F2490D43440D4FF30040BD5
:10003300EF24BFB41A0050032461FFE57760021573
:1000430077057AE57A7002057930070D7867E475EC
:10005300F0011204ADEF02049B02057B7403D20787
:100063008003E4C207F5768B678A688969E4F577CC
:10007300F579F57AE57760077F2012003E80F57504
:1000830078FFC201C200C202C203C205C206C2088F
:1000930012000CFF700D3007057F0012004FAF7A7E
:1000A300AE7922B4255FC2D5C20412000CFF24D05E
:1000B300B40A00501A75F00A787730D50508B6FFF0
:1000C3000106C6A426F620D5047002D20380D924E3
:1000D300CFB41A00EF5004C2E5D20402024FD2019A
:1000E30080C6D20080C0D20280BCD2D580BAD205ED
:1000F30080B47F2012003E2002077401B5770040D0
:10010300F1120003FF12003E020077D208D20680EC
:1001130095120003FB120003FA120003F94A4B7015
:100123000679207A037BFF20022EE577602A7E0082
:100133008E8275830012046E60060EEE657870F091
:10014300C2D5EBC0E0EAC0E0E9C0E0EE120296D00F
:10015300E0F9D0E0FAD0E0FB120455FF60AAEBC04F
:10016300E0EAC0E0E9C0E012003ED0E02401F9D0AB
:10017300E03400FAD0E0FBE5780460DCD578D98080
:10018300877BFF7A027992D202809C791080027970
:1001930008C206C2088008D2D5790A8004790AC247
:1001A300D5E578047002F578E4FAFDFEFF1200034A
:1001B300FC7B08200113120003FD7B1030000A12A0
:1001C3000003FE120003FF7B20EC3382D592D5504F
:1001D30013C3E43000069FFFE49EFEE42001039D69
:1001E300FDE49CFCE4CBF8C201EC700CCFCECDCC8B
:1001F300E824F8F870F38017C3EF33FFEE33FEED16
:1002030033FDEC33FCEB33FB994002FB0FD8E9EBF6
:10021300300105F8D0E0C448B201C0E00AEC4D4E0D
:100223004F78207B0070C2EAB5780040BCC0E01272
:100233000298D0F0D0E0200104C4C0E0C4B201C0F1
:10024300F0120027D0F0D5F0EB0200771204BD01C5
:100253001453018E5800E54C00E14201924F019A7C
:0F02630044019A4900FA4301A0550184460184E1
:100272004501844703405000E92D00ED2E01102B6B
:1002820000F123010E2003292A00A94800000108D9
:100292003F3F3F00790AA2D5200314300509B91067
:1002A200020404B9080104A2D52006025001042068
:1002B20002689202B577005034C0E07F2030031903
:1002C2007F30A20272067205500F1202EFC202C202
:1002D20006C205C2087F30800F300503E9C0E01274
:1002E200003E300503D0E0F9D0E0B577CC300517F9
:1002F2007F30B9100C12003E7F583004077F78809F
:1003020003B9080312003E3002057F2D02003E7F32
:10031200202008F87F2B2006F322920280CF286E3D
:10032200756C6C2900D2011200033001F8C2017809
:100332007730D50108F60200A92D50434958120022
:10034200032403B405004001E490033B9312002F01
:0D035200743A12002FD20375770402018E59
:10045500BB010689828A83E0225002E722BBFE02A5
:09046500E32289828A83E49322D8
:10046E00BB010CE58229F582E5833AF583E0225043
:10047E0006E92582F8E622BBFE06E92582F8E2228D
:0D048E00E58229F582E5833AF583E49322A7
:10049B00BB010689828A83F0225002F722BBFE0140
:0204AB00F3223A
:1004AD00FAE6FB0808E6F925F0F618E6CA3AF62250
:1004BD00D083D082F8E4937012740193700DA3A3CE
:1004CD0093F8740193F5828883E4737402936860E2
:0604DD00EFA3A3A380DFE2
:10057B00EFB40A07740D120586740A309811A89906
:10058B00B8130CC2983098FDA899C298B811F630E0
:07059B0099FDC299F59922B8
:00000001FF
'''
bin8 = array.array('B',[2, 5, 162, 229, 118, 36, 106, 248, 230, 5, 118, 34,
                        120, 103, 48, 7, 2, 120, 106, 228, 117, 240, 1, 18,
                        4, 173, 2, 4, 85, 32, 0, 235, 127, 46, 210, 0, 128,
                        24, 239, 84, 15, 36, 144, 212, 52, 64, 212, 255, 48,
                        4, 11, 239, 36, 191, 180, 26, 0, 80, 3, 36, 97, 255,
                        229, 119, 96, 2, 21, 119, 5, 122, 229, 122, 112, 2,
                        5, 121, 48, 7, 13, 120, 103, 228, 117, 240, 1, 18,
                        4, 173, 239, 2, 4, 155, 2, 5, 123, 116, 3, 210, 7,
                        128, 3, 228, 194, 7, 245, 118, 139, 103, 138, 104,
                        137, 105, 228, 245, 119, 245, 121, 245, 122, 229,
                        119, 96, 7, 127, 32, 18, 0, 62, 128, 245, 117, 120,
                        255, 194, 1, 194, 0, 194, 2, 194, 3, 194, 5, 194, 6,
                        194, 8, 18, 0, 12, 255, 112, 13, 48, 7, 5, 127, 0,
                        18, 0, 79, 175, 122, 174, 121, 34, 180, 37, 95, 194,
                        213, 194, 4, 18, 0, 12, 255, 36, 208, 180, 10, 0, 80,
                        26, 117, 240, 10, 120, 119, 48, 213, 5, 8, 182, 255,
                        1, 6, 198, 164, 38, 246, 32, 213, 4, 112, 2, 210, 3,
                        128, 217, 36, 207, 180, 26, 0, 239, 80, 4, 194, 229,
                        210, 4, 2, 2, 79, 210, 1, 128, 198, 210, 0, 128, 192,
                        210, 2, 128, 188, 210, 213, 128, 186, 210, 5, 128,
                        180, 127, 32, 18, 0, 62, 32, 2, 7, 116, 1, 181, 119,
                        0, 64, 241, 18, 0, 3, 255, 18, 0, 62, 2, 0, 119, 210,
                        8, 210, 6, 128, 149, 18, 0, 3, 251, 18, 0, 3, 250,
                        18, 0, 3, 249, 74, 75, 112, 6, 121, 32, 122, 3, 123,
                        255, 32, 2, 46, 229, 119, 96, 42, 126, 0, 142, 130,
                        117, 131, 0, 18, 4, 110, 96, 6, 14, 238, 101, 120,
                        112, 240, 194, 213, 235, 192, 224, 234, 192, 224,
                        233, 192, 224, 238, 18, 2, 150, 208, 224, 249, 208,
                        224, 250, 208, 224, 251, 18, 4, 85, 255, 96, 170,
                        235, 192, 224, 234, 192, 224, 233, 192, 224, 18, 0,
                        62, 208, 224, 36, 1, 249, 208, 224, 52, 0, 250, 208,
                        224, 251, 229, 120, 4, 96, 220, 213, 120, 217, 128,
                        135, 123, 255, 122, 2, 121, 146, 210, 2, 128, 156,
                        121, 16, 128, 2, 121, 8, 194, 6, 194, 8, 128, 8, 210,
                        213, 121, 10, 128, 4, 121, 10, 194, 213, 229, 120, 4,
                        112, 2, 245, 120, 228, 250, 253, 254, 255, 18, 0, 3,
                        252, 123, 8, 32, 1, 19, 18, 0, 3, 253, 123, 16, 48,
                        0, 10, 18, 0, 3, 254, 18, 0, 3, 255, 123, 32, 236,
                        51, 130, 213, 146, 213, 80, 19, 195, 228, 48, 0, 6,
                        159, 255, 228, 158, 254, 228, 32, 1, 3, 157, 253,
                        228, 156, 252, 228, 203, 248, 194, 1, 236, 112, 12,
                        207, 206, 205, 204, 232, 36, 248, 248, 112, 243, 128,
                        23, 195, 239, 51, 255, 238, 51, 254, 237, 51, 253,
                        236, 51, 252, 235, 51, 251, 153, 64, 2, 251, 15, 216,
                        233, 235, 48, 1, 5, 248, 208, 224, 196, 72, 178, 1,
                        192, 224, 10, 236, 77, 78, 79, 120, 32, 123, 0, 112,
                        194, 234, 181, 120, 0, 64, 188, 192, 224, 18, 2, 152,
                        208, 240, 208, 224, 32, 1, 4, 196, 192, 224, 196,
                        178, 1, 192, 240, 18, 0, 39, 208, 240, 213, 240, 235,
                        2, 0, 119, 18, 4, 189, 1, 20, 83, 1, 142, 88, 0, 229,
                        76, 0, 225, 66, 1, 146, 79, 1, 154, 68, 1, 154, 73,
                        0, 250, 67, 1, 160, 85, 1, 132, 70, 1, 132, 69, 1,
                        132, 71, 3, 64, 80, 0, 233, 45, 0, 237, 46, 1, 16,
                        43, 0, 241, 35, 1, 14, 32, 3, 41, 42, 0, 169, 72, 0,
                        0, 1, 8, 63, 63, 63, 0, 121, 10, 162, 213, 32, 3, 20,
                        48, 5, 9, 185, 16, 2, 4, 4, 185, 8, 1, 4, 162, 213,
                        32, 6, 2, 80, 1, 4, 32, 2, 104, 146, 2, 181, 119, 0,
                        80, 52, 192, 224, 127, 32, 48, 3, 25, 127, 48, 162,
                        2, 114, 6, 114, 5, 80, 15, 18, 2, 239, 194, 2, 194,
                        6, 194, 5, 194, 8, 127, 48, 128, 15, 48, 5, 3, 233,
                        192, 224, 18, 0, 62, 48, 5, 3, 208, 224, 249, 208,
                        224, 181, 119, 204, 48, 5, 23, 127, 48, 185, 16, 12,
                        18, 0, 62, 127, 88, 48, 4, 7, 127, 120, 128, 3, 185,
                        8, 3, 18, 0, 62, 48, 2, 5, 127, 45, 2, 0, 62, 127,
                        32, 32, 8, 248, 127, 43, 32, 6, 243, 34, 146, 2, 128,
                        207, 40, 110, 117, 108, 108, 41, 0, 210, 1, 18, 0, 3,
                        48, 1, 248, 194, 1, 120, 119, 48, 213, 1, 8, 246, 2,
                        0, 169, 45, 80, 67, 73, 88, 18, 0, 3, 36, 3, 180, 5,
                        0, 64, 1, 228, 144, 3, 59, 147, 18, 0, 47, 116, 58,
                        18, 0, 47, 210, 3, 117, 119, 4, 2, 1, 142, 231, 9,
                        246, 8, 223, 250, 128, 70, 231, 9, 242, 8, 223, 250,
                        128, 62, 136, 130, 140, 131, 231, 9, 240, 163, 223,
                        250, 128, 50, 227, 9, 246, 8, 223, 250, 128, 120,
                        227, 9, 242, 8, 223, 250, 128, 112, 136, 130, 140,
                        131, 227, 9, 240, 163, 223, 250, 128, 100, 137,
                        130, 138, 131, 224, 163, 246, 8, 223, 250, 128, 88,
                        137, 130, 138, 131, 224, 163, 242, 8, 223, 250, 128,
                        76, 128, 210, 128, 250, 128, 198, 128, 212, 128, 105,
                        128, 242, 128, 51, 128, 16, 128, 166, 128, 234, 128,
                        154, 128, 168, 128, 218, 128, 226, 128, 202, 128, 51,
                        137, 130, 138, 131, 236, 250, 228, 147, 163, 200,
                        197, 130, 200, 204, 197, 131, 204, 240, 163, 200,
                        197, 130, 200, 204, 197, 131, 204, 223, 233, 222,
                        231, 128, 13, 137, 130, 138, 131, 228, 147, 163, 246,
                        8, 223, 249, 236, 250, 169, 240, 237, 251, 34, 137,
                        130, 138, 131, 236, 250, 224, 163, 200, 197, 130,
                        200, 204, 197, 131, 204, 240, 163, 200, 197, 130,
                        200, 204, 197, 131, 204, 223, 234, 222, 232, 128,
                        219, 137, 130, 138, 131, 228, 147, 163, 242, 8,
                        223, 249, 128, 204, 136, 240, 239, 96, 1, 14, 78,
                        96, 195, 136, 240, 237, 36, 2, 180, 4, 0, 80, 185,
                        245, 130, 235, 36, 2, 180, 4, 0, 80, 175, 35, 35,
                        69, 130, 35, 144, 3, 175, 115, 187, 1, 6, 137, 130,
                        138, 131, 224, 34, 80, 2, 231, 34, 187, 254, 2, 227,
                        34, 137, 130, 138, 131, 228, 147, 34, 187, 1, 12,
                        229, 130, 41, 245, 130, 229, 131, 58, 245, 131, 224,
                        34, 80, 6, 233, 37, 130, 248, 230, 34, 187, 254, 6,
                        233, 37, 130, 248, 226, 34, 229, 130, 41, 245, 130,
                        229, 131, 58, 245, 131, 228, 147, 34, 187, 1, 6,
                        137, 130, 138, 131, 240, 34, 80, 2, 247, 34, 187,
                        254, 1, 243, 34, 250, 230, 251, 8, 8, 230, 249, 37,
                        240, 246, 24, 230, 202, 58, 246, 34, 208, 131, 208,
                        130, 248, 228, 147, 112, 18, 116, 1, 147, 112, 13,
                        163, 163, 147, 248, 116, 1, 147, 245, 130, 136,
                        131, 228, 115, 116, 2, 147, 104, 96, 239, 163, 163,
                        163, 128, 223, 207, 240, 251, 226, 253, 242, 32,
                        255, 32, 242, 225, 32, 226, 251, 230, 243, 10, 0,
                        253, 224, 225, 226, 227, 180, 228, 229, 186, 230,
                        231, 179, 191, 232, 233, 234, 235, 236, 237, 238,
                        239, 240, 241, 242, 243, 244, 245, 246, 247, 248,
                        249, 252, 254, 255, 0, 192, 193, 194, 195, 165, 196,
                        197, 170, 198, 199, 178, 175, 200, 201, 202, 203,
                        204, 205, 206, 207, 208, 209, 210, 211, 212, 213,
                        214, 215, 216, 217, 220, 222, 223, 0, 120, 34, 124,
                        0, 125, 0, 123, 255, 122, 4, 121, 245, 126, 0, 127,
                        35, 18, 4, 47, 120, 69, 124, 0, 125, 0, 123, 255,
                        122, 5, 121, 24, 126, 0, 127, 34, 18, 4, 47, 117,
                        152, 80, 67, 137, 32, 117, 141, 221, 210, 142, 210,
                        153, 99, 144, 1, 123, 255, 122, 4, 121, 227, 18, 0,
                        101, 128, 254, 239, 180, 10, 7, 116, 13, 18, 5, 134,
                        116, 10, 48, 152, 17, 168, 153, 184, 19, 12, 194,
                        152, 48, 152, 253, 168, 153, 194, 152, 184, 17,
                        246, 48, 153, 253, 194, 153, 245, 153, 34, 120, 127,
                        228, 246, 216, 253, 117, 129, 122, 2, 5, 58])


hex16 = """:020000040000FA
:10000000000083120313072055301820042883169C
:10001000031340309900181598168312031318160D
:1000200098170800831203138C1E14281A0808005E
:0C003000831203130C1E1A28990008000C
:00000001FF
"""
bin16 = array.array('H', [0x0000, 0x1283, 0x1303, 0x2007,
                          0x3055, 0x2018, 0x2804, 0x1683,
                          0x1303, 0x3040, 0x0099, 0x1518,
                          0x1698, 0x1283, 0x1303, 0x1618,
                          0x1798, 0x0008, 0x1283, 0x1303,
                          0x1E8C, 0x2814, 0x081A, 0x0008,
                          0x1283, 0x1303, 0x1E0C, 0x281A,
                          0x0099, 0x0008, 0x3FFF, 0x3FFF])


hex64k = """:020000040000FA
:0100000001FE
:020000040001F9
:0100000002FD
:00000001FF
"""
data64k = {0: 1, 0x10000: 2}


hex_rectype3 = """:0400000312345678E5
:0100000001FE
:00000001FF
"""
data_rectype3 = {0: 1}
start_addr_rectype3 = {'CS': 0x1234, 'IP': 0x5678}


hex_rectype5 = """:0400000512345678E3
:0100000002FD
:00000001FF
"""
data_rectype5 = {0: 2}
start_addr_rectype5 = {'EIP': 0x12345678}


##
# Test cases

class TestIntelHex(unittest.TestCase):

    def setUp(self):
        self.f = StringIO(hex8)

    def tearDown(self):
        self.f.close()

    def test_readfile(self):
        ih = intelhex.IntelHex(self.f)
        self.assert_(ih.readfile(), "readfile return error: %s" % ih.Error)
        self.assertEqual(ih.AddrOverlap, None, "Address overlapping: %r" % ih.AddrOverlap)
        # test internal buffer
        self.assertNotEqual({}, ih._buf, "Internal buffer is empty")

    def test_tobinstr(self):
        ih = intelhex.IntelHex(self.f)
        ih.readfile()
        s1 = ih.tobinstr()
        s2 = bin8.tostring()
        self.assertEqual(s2, s1, "data not equal\n%s\n\n%s" % (s1, s2))

    def test_tobinfile(self):
        ih = intelhex.IntelHex(self.f)
        ih.readfile()
        sio = StringIO()
        ih.tobinfile(sio)
        s1 = sio.getvalue()
        sio.close()
        s2 = bin8.tostring()
        self.assertEqual(s2, s1, "data not equal\n%s\n\n%s" % (s1, s2))


class TestIntelHexStartingAddressRecords(unittest.TestCase):

    def _test_read(self, hexstr, data, start_addr):
        sio = StringIO(hexstr)
        ih = IntelHex(sio)
        self.assert_(ih.readfile(), "readfile return error: %s" % ih.Error)
        sio.close()
        # test data
        self.assertEqual(data, ih._buf,
                         "Internal buffer: %r != %r" %
                         (data, ih._buf))
        self.assertEqual(start_addr, ih.start_addr,
                         "Start address: %r != %r" %
                         (start_addr, ih.start_addr))

    def test_read_rectype3(self):
        self._test_read(hex_rectype3, data_rectype3, start_addr_rectype3)

    def test_read_rectype5(self):
        self._test_read(hex_rectype5, data_rectype5, start_addr_rectype5)

    def _test_write(self, hexstr, data, start_addr, write_start_addr=True):
        # prepare
        ih = IntelHex(None)
        ih._buf = data
        ih.start_addr = start_addr
        # write
        sio = StringIO()
        self.assertTrue(ih.writefile(sio, write_start_addr))
        s = sio.getvalue()
        sio.close()
        # check
        self.assertEquals(hexstr, s, """Written data is incorrect
Should be:
%s

Written:
%s
""" % (hexstr, s))

    def _test_dont_write(self, hexstr, data, start_addr):
        expected = ''.join(hexstr.splitlines(True)[1:])
        self._test_write(expected, data, start_addr, False)

    def test_write_rectype3(self):
        self._test_write(hex_rectype3, data_rectype3, start_addr_rectype3)

    def test_dont_write_rectype3(self):
        self._test_dont_write(hex_rectype3, data_rectype3, start_addr_rectype3)

    def test_write_rectype5(self):
        self._test_write(hex_rectype5, data_rectype5, start_addr_rectype5)

    def test_dont_write_rectype5(self):
        self._test_dont_write(hex_rectype5, data_rectype5, start_addr_rectype5)


class TestIntelHex_big_files(unittest.TestCase):
    """Test that data bigger than 64K read/write correctly"""

    def setUp(self):
        self.f = StringIO(hex64k)

    def tearDown(self):
        self.f.close()

    def test_readfile(self):
        ih = intelhex.IntelHex(self.f)
        ih.readfile()
        for addr, byte in data64k.items():
            readed = ih[addr]
            self.assertEquals(byte, readed, "data not equal at addr %X (%X != %X)" % (addr, byte, readed))

    def test_writefile(self):
        ih = intelhex.IntelHex(self.f)
        ih.readfile()
        sio = StringIO()
        self.assertTrue(ih.writefile(sio))
        s = sio.getvalue()
        sio.close()
        self.assertEquals(hex64k, s, """Written data is incorrect
Should be:
%s

Written:
%s
""" % (hex64k, s))


class TestIntelHex16bit(unittest.TestCase):

    def setUp(self):
        self.f = StringIO(hex16)

    def tearDown(self):
        self.f.close()

    def test_readfile(self):
        ih = intelhex.IntelHex16bit(self.f)
        self.assert_(ih.readfile(), "readfile return error: %s" % ih.Error)
        self.assertEqual(ih.AddrOverlap, None, "Address overlapping: %r" % ih.AddrOverlap)

    def test_minaddr(self):
        ih = intelhex.IntelHex16bit(self.f)
        ih.readfile()
        addr = ih.minaddr()
        self.assertEqual(0, addr, 'Error in detection of minaddr (0 != 0x%x)' % addr)

    def test_maxaddr(self):
        ih = intelhex.IntelHex16bit(self.f)
        ih.readfile()
        addr = ih.maxaddr()
        self.assertEqual(0x001D, addr, 'Error in detection of maxaddr (0x001D != 0x%x)' % addr)

    def test_getitem(self):
        ih = intelhex.IntelHex16bit(self.f)
        ih.readfile()
        ih.padding = 0x3FFF
        for addr, word in enumerate(bin16):
            self.assertEqual(word, ih[addr],
                             'Data mismatch at address 0x%x (0x%x != 0x%x)' % (addr, word, ih[addr]))

    def test_writefile(self):
        ih = intelhex.IntelHex16bit(self.f)
        ih.readfile()

        sio = StringIO()
        ih.writefile(sio)
        s = sio.getvalue()
        sio.close()

        fin = StringIO(s)
        ih2 = intelhex.IntelHex16bit(fin)
        ih2.readfile()

        self.assertEqual(ih.tobinstr(), ih2.tobinstr(), "Writed hex file does not equal with original")

    def test_setitem(self):
        ih = intelhex.IntelHex16bit(self.f)
        ih.readfile()

        old = ih[0]
        ih[0] = old ^ 0xFFFF

        self.assertNotEqual(old, ih[0],
                            "Setting new value to internal buffer failed")
#/class TestIntelHex16bit


##
# MAIN
if __name__ == '__main__':
    unittest.main()
