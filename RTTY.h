#ifndef __RTTY_H__
#define __RTTY_H__

#include <Arduino.h>
#include <SoftwareSerial.h>

#include "config.h"
#include "debugutils.h"
#include "hardwareutils.h"

#include <string.h>
#include <util/crc16.h>


class RTTY
{
    private:
        SoftwareSerial NTX2EN;
        
        uint16_t gps_CRC16_checksum (char*);
        char datastring[22];
        
        //void RTTY_SetChannel();
        
        //void RTTY_SetChannel();
     	void rtty_txbit(int);
     	void rtty_txbyte (char);
     	void rtty_txstring (char*);
    
    public:
        RTTY ();
        ~RTTY() {};
        void Transmit_RTTY_GPS();
        void Transmit_RTTY_NO_GPS();
        void RTTY_SetChannel();
        void RTTY_Setup();
        void check_RTTY();   
};


 /*************************************************NTX2 CHANNEL LIST*************************************************
00 = .050000 | 20 = .150000 | 40 = .250000 | 60 = .350000 | 80 = .450000 | A0 = .550000 | C0 = .650000 | E0 = .750000
01 = .053125 | 21 = .153125 | 41 = .253125 | 61 = .353125 | 81 = .453125 | A1 = .553125 | C1 = .653125 | E1 = .753125
02 = .056250 | 22 = .156250 | 42 = .256250 | 62 = .356250 | 82 = .456250 | A2 = .556250 | C2 = .656250 | E2 = .756250
03 = .059375 | 23 = .159375 | 43 = .259375 | 63 = .359375 | 83 = .459375 | A3 = .559375 | C3 = .659375 | E3 = .759375
04 = .062500 | 24 = .162500 | 44 = .262500 | 64 = .362500 | 84 = .462500 | A4 = .562500 | C4 = .662500 | E4 = .762500
05 = .065625 | 25 = .165625 | 45 = .265625 | 65 = .365625 | 85 = .465625 | A5 = .565625 | C5 = .665625 | E5 = .765625
06 = .068750 | 26 = .168750 | 46 = .268750 | 66 = .368750 | 86 = .468750 | A6 = .568750 | C6 = .668750 | E6 = .768750
07 = .071875 | 27 = .171875 | 47 = .271875 | 67 = .371875 | 87 = .471875 | A7 = .571875 | C7 = .671875 | E7 = .771875
08 = .075000 | 28 = .175000 | 48 = .275000 | 68 = .375000 | 88 = .475000 | A8 = .575000 | C8 = .675000 | E8 = .775000
09 = .078125 | 29 = .178125 | 49 = .278125 | 69 = .378125 | 89 = .478125 | A9 = .578125 | C9 = .678125 | E9 = .778125
0A = .081250 | 2A = .181250 | 4A = .281250 | 6A = .381250 | 8A = .481250 | AA = .581250 | CA = .681250 | EA = .781250
0B = .084375 | 2B = .184375 | 4B = .284375 | 6B = .384375 | 8B = .484375 | AB = .584375 | CB = .684375 | EB = -
0C = .087500 | 2C = .187500 | 4C = .287500 | 6C = .387500 | 8C = .487500 | AC = .587500 | CC = .687500 | EC = -
0D = .090625 | 2D = .190625 | 4D = .290625 | 6D = .390625 | 8D = .490625 | AD = .590625 | CD = .690625 | ED = -
0E = .093750 | 2E = .193750 | 4E = .293750 | 6E = .393750 | 8E = .493750 | AE = .593750 | CE = .693750 | EE = -
0F = .096875 | 2F = .196875 | 4F = .296875 | 6F = .396875 | 8F = .496875 | AF = .596875 | CF = .696875 | EF = -
10 = .100000 | 30 = .200000 | 50 = .300000 | 70 = .400000 | 90 = .500000 | B0 = .600000 | D0 = .700000 | F0 = -
11 = .103125 | 31 = .203125 | 51 = .303125 | 71 = .403125 | 91 = .503125 | B1 = .603125 | D1 = .703125 | F1 = -
12 = .106250 | 32 = .206250 | 52 = .306250 | 72 = .406250 | 92 = .506250 | B2 = .606250 | D2 = .706250 | F2 = -
13 = .109375 | 33 = .209375 | 53 = .309375 | 73 = .409375 | 93 = .509375 | B3 = .609375 | D3 = .709375 | F3 = -
14 = .112500 | 34 = .212500 | 54 = .312500 | 74 = .412500 | 94 = .512500 | B4 = .612500 | D4 = .712500 | F4 = -
15 = .115625 | 35 = .215625 | 55 = .315625 | 75 = .415625 | 95 = .515625 | B5 = .615625 | D5 = .715625 | F5 = -
16 = .118750 | 36 = .218750 | 56 = .318750 | 76 = .418750 | 96 = .518750 | B6 = .618750 | D6 = .718750 | F6 = -
17 = .121875 | 37 = .221875 | 57 = .321875 | 77 = .421875 | 97 = .521875 | B7 = .621875 | D7 = .721875 | F7 = -
18 = .125000 | 38 = .225000 | 58 = .325000 | 78 = .425000 | 98 = .525000 | B8 = .625000 | D8 = .725000 | F8 = -
19 = .128125 | 39 = .228125 | 59 = .328125 | 79 = .428125 | 99 = .528125 | B9 = .628125 | D9 = .728125 | F9 = -
1A = .131250 | 3A = .231250 | 5A = .331250 | 7A = .431250 | 9A = .531250 | BA = .631250 | DA = .731250 | FA = -
1B = .134375 | 3B = .234375 | 5B = .334375 | 7B = .434375 | 9B = .534375 | BB = .634375 | DB = .734375 | FB = -
1C = .137500 | 3C = .237500 | 5C = .337500 | 7C = .437500 | 9C = .537500 | BC = .637500 | DC = .737500 | FC = -
1D = .140625 | 3D = .240625 | 5D = .340625 | 7D = .440625 | 9D = .540625 | BD = .640625 | DD = .740625 | FD = -
1E = .143750 | 3E = .243750 | 5E = .343750 | 7E = .443750 | 9E = .543750 | BE = .643750 | DE = .743750 | FE = -
1F = .146875 | 3F = .246875 | 5F = .346875 | 7F = .446875 | 9F = .546875 | BF = .646875 | DF = .746875 | FF = -
*********************************************************************************************************************/

#endif
