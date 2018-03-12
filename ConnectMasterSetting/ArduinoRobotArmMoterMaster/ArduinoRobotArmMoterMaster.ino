// Set Goal Position
//  Move position to the full range
//  Since software serial is selected, dynamixel should be use at 57600bps or less.
//  1.select [SOFT]
//  2.compile & download

#define DEBUG
//#define DEBUG_SETTING

#include <HardwareSerial.h>
#include "Event.h"
#include "EEPROMConfig.h"
#include "Ring.h"
extern "C" {
#include "CalcCRC.h"
}
#include <avr/wdt.h> //WatchDogTimer

#include <dxlib.h>
//#include <SoftwareSerial.h>
#include <PS3USB.h>
//#include <LiquidCrystal.h>

#define REG_MOTER_MAX_POS 8
#define REG_MOTER_MIN_POS 6
#define MOTER_NUM 3

static int gMoterID[MOTER_NUM] = { 4, 3, 2};

int16_t gMaxPos[MOTER_NUM];
int16_t gMinPos[MOTER_NUM];
static int16_t defMaxPos[MOTER_NUM] = { 1000, 800, 1010 };//  0～1023
static int16_t defMinPos[MOTER_NUM] = { 10,   200, 0    };
int16_t gpos[MOTER_NUM] = {800, 800, 800 };
int dir = 10;

DXLIB dxif (true); // select software serial

//############  PS3 Controller   ############

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
/* You can create the instance of the class in two ways */
PS3USB PS3(&Usb); // This will just create the instance
//PS3USB PS3(&Usb,0x00,0x15,0x83,0x3D,0x0A,0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

bool printAngle;
uint8_t state = 0;

//############  HardwareSerial   ############
#define DebugSerial       Serial
#define MoterSlaveSerial  Serial1
#define MoterMasterSerial Serial2

//############  MasterSetting   ############
#define BUFF_MAX 255
#define DEF_DELTA 100
int gindexsetting;

//############  EEPROM   ############
configuration conf_ram;

#define USE_PS3
//#define USE_DX
#define POS_RANGE1 137
#define POS_RANGE2 117

//############  MoterMasterCommand   ############
#define HEADER_LEN 5
#define CRC_SIZE 2

typedef struct COMMAND_HEAD_t
{
  //byte == unsigned char
  unsigned char main;
  unsigned char sub;
  unsigned char ver;
  unsigned short len;
} CommandHead;

CommandHead gComHead[2] = {
  { 0x01, 0x01, 0x01, 0x0003}// XYZ DelataCommand
};

typedef struct XYZ_t
{
  byte x;
  byte y;
  byte z;
} XYZ;

typedef struct ComXYZ_t
{
  CommandHead head;
  XYZ pos;
} ComXYZ;

ComXYZ gxyz = { { 0x01, 0x01, 0x01, 0x0003}, { 100, 100, 100} };

//############  EventManager   ############
EventManager evtManager;

struct SettingLisnerTask : public EventTask
{
  using EventTask::execute;

  int getSetting()
  {
    byte ch;
    byte rcv;
    int moter;
    int OK;
    int16_t pos;
    byte bufferSerialSetting[RING_BUF];
    //UARTから読み込み
    if (MoterMasterSerial.available()) {
      ch = MoterMasterSerial.read();
      OK = inputSerialSetting(ch, bufferSerialSetting, &gindexsetting);

      if ( OK ) {
        memcpy( &gxyz , bufferSerialSetting, sizeof(ComXYZ));
      } else {
        ;//        MoterMasterSerial.end();
      }
    } else {
      ;//      MoterMasterSerial.end();
    } else {
        DebugSerial.println(" NOT MoterMasterSerial.available");
    }
  }

  int inputSerialSetting(byte ch, byte *lbufferSerial, int *inoutindex) {
    int ii;
    unsigned short len = 0;
    byte lCmd[RING_BUF];// 本当は電文の最大長 + alpha を確保する。ので間違いではない
    int fail = 1;// デフォルトは電文取得に失敗

    // 取得した電文(1byte)をリングバッファに詰める
    RingWrite(ch);
    // なんとなく取得した電文数を記録する。多分、全体でみると間違っているのでTODO
    (*inoutindex)++;

    if ( HEADER_LEN <= RingSize() ) {
      RingGetCommand(lCmd, HEADER_LEN );
    }

    // ヘッダ部の読み込み
    while (RingSize() >= (HEADER_LEN) ) {
      for ( ii = 0; ii < (sizeof(gComHead) / sizeof(gComHead[0])); ii++ ) {
        if (gComHead[ii].main == lCmd[0]) {
          if (gComHead[ii].sub == lCmd[1]) {
            if (gComHead[ii].ver == lCmd[2]) {
              len = *((unsigned short*)(&lCmd[3]));
              //DebugSerial.print("len = "); DebugSerial.println(len);
              if ( gComHead[ii].len != len ) {
                len = 0;
                // レングス不正のため次の位置に更新（2byteのためこことforのあとので次にする）
                RingReadPosAdd(1);
                // ヘッダ取得に失敗したことを記録する
                fail = 1;
              } else {
                fail = 0;
                break;
              }
            }
          }
        }
      }
      if (fail == 0) {
        // 成功したらwhileを抜ける
        break;
      } else {
        // ヘッダ取得失敗のためリングバッファを1byte次にする
        RingReadPosAdd(1);
      }
    }
#ifdef DEBUG_SETTING
    //RingPrint();
#endif

    //DebugSerial.print("Size = "); DebugSerial.println(RingSize());
    // 取得に成功しており、かつ、リングバッファがヘッダサイズ+ヘッダに書かれたデータ部のレングス+CRCサイズ以上か
    if ( fail == 0 && RingSize() >= (HEADER_LEN + len + CRC_SIZE) ) {
#ifdef DEBUG_SETTING
      DebugSerial.println("============");
#endif
      memset(lCmd, 0x00, RING_BUF);
      RingGetCommand(lCmd, HEADER_LEN + len + CRC_SIZE  );
#ifdef DEBUG_SETTING
      RingPrint2(lCmd, HEADER_LEN + len + CRC_SIZE);//DebugCode
#endif
      // 取得したデータからCRC計算
      unsigned short crc = crc16(0, (unsigned char*)lCmd, HEADER_LEN + len);
      // 通信で取得したCRC
      unsigned short settingcrc = *((unsigned short*)(&lCmd[HEADER_LEN + len]));
#ifdef DEBUG_SETTING
      DebugSerial.println(crc);
      DebugSerial.println(settingcrc);
#endif
      if ( crc == settingcrc ) {
        // 電文の取得に成功した
#ifdef DEBUG_SETTING
        DebugSerial.println("Success");
#endif
        // 次の位置に更新
        RingReadPosAdd((HEADER_LEN + len + CRC_SIZE));
        return 1;
      } else {
        // 電文の取得に失敗したので
        // 次の位置に更新
        RingReadPosAdd(1);
#ifdef DEBUG_SETTING
        DebugSerial.println("Fail");
#endif
      }
    }
END:
    return 0;
  }

  void execute(Event evt)
  {
    getSetting();
  }
} SettingLisnerTask;

//############  DebugCode   ############
#ifdef DEBUG    //Macros are usually in all capital letters.
#define DebugSerialPrint(...)    DebugSerial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
#define DebugSerialPrintln(...)  DebugSerial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#else
#define DebugSerialPrint(...)     //now defines a blank line
#define DebugSerialPrintln(...)   //now defines a blank line
#endif

//############  setup   ############
void setup() {
  DebugSerial.begin(115200);// Debug
  MoterSlaveSerial.begin(115200);// Connect MoterSlave
  MoterMasterSerial.begin(115200);// Connect Setting
  if (!DebugSerial) {
    DebugSerial.end();
  }
  if (!MoterSlaveSerial) {
    DebugSerial.println("MoterSlaveSerial.end()");
    MoterSlaveSerial.end();
  }
  if (!MoterMasterSerial) {
    DebugSerial.println("MoterMasterSerial.end()");
    MoterMasterSerial.end();
  }

  RingInit();

#ifdef DEBUG
  DebugSerial.println("=======CONFIG GET RESULT START =======");
  DebugSerial.println(conf_ram.delta_x);
  DebugSerial.println(conf_ram.delta_y);
  DebugSerial.println(conf_ram.delta_z);
  DebugSerial.println(conf_ram.crc);
  DebugSerial.println("=======GET RESULT END=======");
#endif
#ifdef USE_DX
  dxif.begin (57143);
#endif

#ifdef USE_PS3
  if (Usb.Init() == -1) {
    DebugSerialPrintln("OSC did not start");
    // don't do anything more:
    wdt_enable(WDTO_1S);   // 1秒のウォッチドッグタイマーをセット
    return;
  }
#endif

#ifdef USE_DX
  int ii;
  // Get max/min position limit
  for ( ii = 0; ii < MOTER_NUM; ii++) {
    dxif.ReadWordData (gMoterID[ii], REG_MOTER_MAX_POS, (uint16_t *)&gMaxPos[ii], NULL);
    DebugSerialPrint("REG_MOTER_MAX_POS  ");
    DebugSerialPrintln(gMaxPos[ii]);
    dxif.ReadWordData (gMoterID[ii], REG_MOTER_MIN_POS, (uint16_t *)&gMinPos[ii], NULL);
    DebugSerialPrint("REG_MOTER_MIN_POS  ");
    DebugSerialPrintln(gMinPos[ii]);
  }
#endif
  DebugSerialPrintln("Moter Control Started");

  evtManager.subscribe(Subscriber("event.keepAlive", &SettingLisnerTask));
  Event keepAlive = Event("event.keepAlive");
  evtManager.triggerInterval(TimedTask(500, keepAlive));
}

void loop() {

  evtManager.tick();

  if (!DebugSerial) {
    DebugSerial.end();
  }
  if (!MoterSlaveSerial) {
    DebugSerial.println("MoterSlaveSerial.end()");
    MoterSlaveSerial.end();
  }
  if (!MoterMasterSerial) {
    DebugSerial.println("MoterMasterSerial.end()");
    MoterMasterSerial.end();
  }

#ifdef USE_PS3
  Usb.Task();

  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    int16_t movex = PS3.getAnalogHat(LeftHatX);

    if (movex > POS_RANGE1 || movex < POS_RANGE2)
    {

      DebugSerialPrint(F("LeftHatX: "));
      DebugSerialPrint(LeftHatX);

      if ( movex > POS_RANGE1 ) {
        // 1はゼロ除算対策
        // モーターの配置の都合上、-1をかけて反対にする
        gpos[0] = gpos[0] +  -1 * (movex - POS_RANGE1 + 1) / 25 - (gxyz.pos.x - DEF_DELTA);
      } else {
        gpos[0] = gpos[0] +  -1 * (movex - POS_RANGE1 + 1) / 25 + (gxyz.pos.x - DEF_DELTA);
      }
      MoveMoter( 0, gMoterID[0], &(gpos[0]));
    }

    if ( PS3.getAnalogHat(LeftHatY) > POS_RANGE1 || PS3.getAnalogHat(LeftHatY) < POS_RANGE2 )
    {
      int16_t movey = PS3.getAnalogHat(LeftHatY);
      DebugSerialPrint(F("\tLeftHatY: "));
      DebugSerialPrint(movey);
      if ( movey > POS_RANGE1 ) {
        // 1はゼロ除算対策
        gpos[1] = gpos[1] + (movey - POS_RANGE1 + 1) / 25 + (gxyz.pos.y - DEF_DELTA);
      } else {
        gpos[1] = gpos[1] + (movey - POS_RANGE2 + 1) / 25 - (gxyz.pos.y - DEF_DELTA);
      }
      MoveMoter( 1, gMoterID[1], &(gpos[1]));

    }
    /*
      if( PS3.getAnalogHat(RightHatX) > 137 || PS3.getAnalogHat(RightHatX) < 117)
      {
      DebugSerialPrint(F("\tRightHatX: "));
      DebugSerialPrint(RightX);
      }
    */

    if ( PS3.getAnalogHat(RightHatY) > POS_RANGE1 || PS3.getAnalogHat(RightHatY) < POS_RANGE2)
    {
      int16_t movez = PS3.getAnalogHat(RightHatY);
      DebugSerialPrint(F("\tRightHatY: "));
      DebugSerialPrintln(movez);

      if ( movez > POS_RANGE1 ) {
        // 1はゼロ除算対策
        gpos[2] = gpos[2] + (movez - POS_RANGE1 + 1) / 25 + (gxyz.pos.z - DEF_DELTA);
      } else {
        gpos[2] = gpos[2] + (movez - POS_RANGE2 + 1) / 25 - (gxyz.pos.z - DEF_DELTA);
      }
      MoveMoter( 2, gMoterID[2], &(gpos[2]));
    }
  } else if (PS3.PS3MoveConnected) { // One can only set the color of the bulb, set the rumble, set and get the bluetooth address and calibrate the magnetometer via USB
    DebugSerialPrintln("PS3MoveConnected");
    if (state == 0) {
      PS3.moveSetRumble(0);
      PS3.moveSetBulb(Off);
    } else if (state == 1) {
      PS3.moveSetRumble(75);
      PS3.moveSetBulb(Red);
    } else if (state == 2) {
      PS3.moveSetRumble(125);
      PS3.moveSetBulb(Green);
    } else if (state == 3) {
      PS3.moveSetRumble(150);
      PS3.moveSetBulb(Blue);
    } else if (state == 4) {
      PS3.moveSetRumble(175);
      PS3.moveSetBulb(Yellow);
    } else if (state == 5) {
      PS3.moveSetRumble(200);
      PS3.moveSetBulb(Lightblue);
    } else if (state == 6) {
      PS3.moveSetRumble(225);
      PS3.moveSetBulb(Purple);
    } else if (state == 7) {
      PS3.moveSetRumble(250);
      PS3.moveSetBulb(White);
    }

    state++;
    if (state > 7)
      state = 0;
    delay(1000);
  }
#endif
  delay(30);
}

void DebugLogXYZ( int x, int y, int z)
{
  DebugSerialPrint(F("X: "));
  DebugSerialPrint(x);
  DebugSerialPrint(F("\t y: "));
  DebugSerialPrint(y);
  DebugSerialPrint(F("\t z: "));
  DebugSerialPrintln(z);
}

int MoveMoter( int index, int moterID, int16_t *movepos)
{
  if ( *movepos > defMaxPos[index] ) {
    DebugSerialPrintln(F("defMaxPos"));
    *movepos = defMaxPos[index];
  } else if ( *movepos < defMinPos[index] ) {
    DebugSerialPrintln(F("defMinPos"));
    *movepos = defMinPos[index];
  }
  // Set goal position
  DebugSerialPrint(F("moterID: "));
  DebugSerialPrintln(moterID);
  DebugSerialPrint("movepos");
  DebugSerialPrintln(*movepos);
#ifdef USE_DX
  dxif.WriteWordData (moterID, 30, *movepos, NULL);
#endif
  char buf[255];
  sprintf(buf, "%d%d", moterID, *movepos);
  int n = strlen(buf);
  unsigned short crc = crc16(0, (unsigned char*)buf, n);
  // この電文形式はすべてASCIIなのでIDEのシリアルモニターで電文を確認できます
  sprintf(buf, "%s%02x%02x", buf, (crc >> 8), (crc & 0xff));
  if (MoterSlaveSerial.available()) {
    MoterSlaveSerial.print(buf);
    MoterSlaveSerial.print(';');
  }

  if (DebugSerial.available()) {
    DebugSerial.print(buf);
    DebugSerial.print(';');
  }
  return 1;
}

