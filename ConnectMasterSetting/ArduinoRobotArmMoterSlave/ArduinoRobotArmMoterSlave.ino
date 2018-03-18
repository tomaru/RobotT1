// Set Goal Position
//  Move position to the full range
//  Since software serial is selected, dynamixel should be use at 57600bps or less.
//  1.select [SOFT]
//  2.compile & download
//#define DEBUG_COMMAND
//#define DEBUG
//#define DEBUG_SETTING

//############  HardwareSerial   ############
#define DebugSerial       Serial1
#define MoterPOSSerial  Serial
#define MoterPOSSerialEvent serialEvent

#ifdef DEBUG    //Macros are usually in all capital letters.
#define DebugSerialPrint(...)    Serial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
#define DebugSerialPrintln(...)  Serial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#else
#define DebugSerialPrint(...)     //now defines a blank line
#define DebugSerialPrintln(...)   //now defines a blank line
#endif


#include "Ring.h"
#include "Command.h"
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

//############  MoterMasterCommand   ############
#define CMD_NUM 1

typedef struct Moter_t
{
  byte moterID;
  int16_t pos;
} MoterPOS;

typedef struct ComPOS_t
{
  CommandHead head;
  MoterPOS moter;
} ComPOS;

CommandHead gComHead[CMD_NUM] = {
  //  { 0x01, 0x01, 0x01, 0x0003}, // XYZ DelataCommand
  { 0x02, 0x01, 0x01, 0x0003}  // X or Y or Z POS
};

ComPOS gpos = { { 0x02, 0x01, 0x01, 0x0003}, { 2, 800} };

void MoterPOSSerialEvent() {
  byte ch;
  //UARTから読み込み
  while (MoterPOSSerial.available()) {
    ch = MoterPOSSerial.read();
    // 取得した電文(1byte)をリングバッファに詰める
    RingWrite(ch);
#ifdef DEBUG_SETTING
    RingPrint();
#endif
  }
}

int MoveMoter( int moterID, int16_t movepos)
{
  int ii;
  for ( ii = 0; (ii < (sizeof(gMoterID) / sizeof(gMoterID[0]))) && moterID != gMoterID[ii]; ii++) {
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

  DebugSerialPrint(moterID);
  DebugSerialPrint("   ");
  DebugSerialPrintln(movepos);

  return 1;
}
void setup() {
  dxif.begin (57143);
  DebugSerial.begin(115200);
  MoterPOSSerial.begin(115200);

  RingInit();

  DebugSerialPrintln("Moter Control Started");
}
void getPos() {
  int get_num;
  byte bufferSerialSetting[RING_BUF];
  do {
    get_num = getCommand(bufferSerialSetting, gComHead, CMD_NUM);
    if ( 1 <= get_num ) {
      memcpy( &gpos, bufferSerialSetting, sizeof(ComPOS) );
      MoveMoter( gpos.moter.moterID, gpos.moter.pos);
      DebugSerialPrint(gpos.moter.moterID);
      DebugSerialPrint("   ");
      DebugSerialPrintln(gpos.moter.pos);
    } else if ( -1 == get_num ) {
      MoterPOSSerial.println("F");
    }
  } while ( 1 <= get_num  );
}

void loop() {
  //UARTから読み込み
  getPos();
  /*
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
  */

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
    MoterPOSSerial.println("F");
#ifdef DEBUG
    Serial.println("Fail");
#endif
    return 0;
  }
END:
  return 0;
}

