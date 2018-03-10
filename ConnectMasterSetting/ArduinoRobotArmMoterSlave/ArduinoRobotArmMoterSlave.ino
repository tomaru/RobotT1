// Set Goal Position
//  Move position to the full range
//  Since software serial is selected, dynamixel should be use at 57600bps or less.
//  1.select [SOFT]
//  2.compile & download
extern "C" {
#include "CalcCRC.h"
}

#include <avr/wdt.h> //WatchDogTimer

#include <dxlib.h>
#include <SoftwareSerial.h>

#define BUFF_MAX 255
char bufferSerial[BUFF_MAX];
int gindex;

#define MOTER_NUM 3
static int gMoterID[MOTER_NUM] = { 4, 3, 2};
static int16_t defMaxPos[MOTER_NUM] = { 900, 800, 950 };//  0～1023
static int16_t defMinPos[MOTER_NUM] = { 100, 200, 50  };

DXLIB dxif (true); // select software serial

#define DEBUG

#ifdef DEBUG    //Macros are usually in all capital letters.
#define DebugSerialPrint(...)    Serial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
#define DebugSerialPrintln(...)  Serial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#else
#define DebugSerialPrint(...)     //now defines a blank line
#define DebugSerialPrintln(...)   //now defines a blank line
#endif
void setup() {
  dxif.begin (57143);
  Serial.begin(115200);

  DebugSerialPrintln("Moter Control Started");
}

void loop() {
  char ch;
  char rcv;
  int moter;
  int OK;
  int16_t pos;
  //UARTから読み込み
  if (Serial.available()) {
    ch = Serial.read();
    OK = inputSerial(ch, bufferSerial, &gindex);

    if ( OK ) {
      char *sendbuffer;
      sendbuffer = bufferSerial;
      OK = DecodeArmController(&moter, sendbuffer, gindex, &pos);
      switch ( OK ) {
        // 取得成功
        case 1 :
          MoveMoter( moter, pos);
          gindex = 0;
          break;
        // 取得失敗
        case -1 :
          gindex = 0;
          break;
        // 電文未完成
        default :
          break;
      }
    }
  }

}

int inputSerial(char ch, char *lbufferSerial, int *inoutindex) {
  lbufferSerial[*inoutindex] = ch;
  if ( lbufferSerial[*inoutindex] == '\r' || lbufferSerial[*inoutindex] == '\n') {
    goto END;
  }
  (*inoutindex) = (*inoutindex) + 1;
  //バッファ以上の場合は中断
  if (*inoutindex >= BUFF_MAX) {
#ifdef DEBUG
    Serial.println("Break");
#endif
    *inoutindex = 0;
    goto END;
  }
  if ( lbufferSerial[*inoutindex - 1] == ';' ) {
#ifdef DEBUG
    Serial.println("=== CmdDetected ===");
    Serial.println(lbufferSerial);
    Serial.println(*inoutindex);
#endif

    unsigned short crc = crc16(0, (unsigned char*)lbufferSerial, *inoutindex - 5);
    char temp[100];
    sprintf( temp, "%02x%02x", (crc >> 8), (crc & 0xff) );
#ifdef DEBUG
    Serial.println(temp);
#endif

    if ( (temp[0] == lbufferSerial[*inoutindex - 5]) &&
         (temp[1] == lbufferSerial[*inoutindex - 4]) &&
         (temp[2] == lbufferSerial[*inoutindex - 3]) &&
         (temp[3] == lbufferSerial[*inoutindex - 2])) {
      lbufferSerial[*inoutindex - 5] = '\0';
#ifdef DEBUG
      Serial.println("Suceess");
      Serial.println(lbufferSerial);
#endif
      return 1;

    }

    *inoutindex = 0;
#ifdef DEBUG
    Serial.println("Fail");
#endif
    return 0;
  }
END:
  return 0;
}

int DecodeArmController(int *moter, char *input, int lindex, int *pos)
{
  *pos = 0;
#ifdef DEBUG
  Serial.println("=== DecodeArmMasterTerm ===");
  Serial.println(lindex);
  Serial.println(*input);
#endif
  if ( lindex >= 3) {
    *moter = (input)[0] - '0';
    *pos = atoi(&(input[1]));
    return 1;
  } else {
    return 0;
  }
}
int MoveMoter( int moterID, int16_t movepos)
{
  int ii;
  for ( ii = 0; (ii < (sizeof(gMoterID)/sizeof(gMoterID[0]))) && moterID != gMoterID[ii]; ii++) {
    ;
  }
  if ( ii != (sizeof(gMoterID) / sizeof(gMoterID[0]))) {
    if ( movepos > defMaxPos[ii] ) {
      DebugSerialPrintln(F("defMaxPos"));
      movepos = defMaxPos[ii];
    } else if ( movepos < defMinPos[ii] ) {
      DebugSerialPrintln(F("defMinPos"));
      movepos = defMinPos[ii];
    }
  }
  dxif.WriteWordData (moterID, 30, movepos, NULL);
  Serial.print(moterID);
  Serial.print(movepos);
  Serial.println(' ');
  return 1;
}
