//#include <AltSoftSerial.h>

// Set Goal Position
//  Move position to the full range
//  Since software serial is selected, dynamixel should be use at 57600bps or less.
//  1.select [SOFT]
//  2.compile & download

//#define DEBUG
//#define DEBUG_SETTING
//#define DEBUG_COMMAND

#include <HardwareSerial.h>
//############  HardwareSerial   ############
#define DebugSerial       Serial
#define POSSerial  Serial1
#define POSSerialEvent serialEvent1
#define SettingSerial Serial2
#define SettingSerialEvent serialEvent2

#include "EEPROMConfig.h"
#include "Ring.h"
#include "Event.h"
#include <avr/wdt.h> //WatchDogTimer

#include <PS3USB.h>

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

byte gPosSendBuff[16];

//############  One Run   ############
unsigned long startMs;
#define ONE_RUN_TIMEOUT 5000

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
//############  DebugCode   ############
#ifdef DEBUG    //Macros are usually in all capital letters.
#define DebugSerialPrint(...)    DebugSerial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
#define DebugSerialPrintln(...)  DebugSerial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#else
#define DebugSerialPrint(...)     //now defines a blank line
#define DebugSerialPrintln(...)   //now defines a blank line
#endif

//############  MasterSetting   ############
#define BUFF_MAX 255
#define DEF_DELTA 100
int gindexsetting;

//############  EEPROM   ############
configuration conf_ram = conf_def;

#define USE_PS3
//#define USE_DX
#define POS_RANGE1 137
#define POS_RANGE2 117

//############  MoterMasterCommand   ############
#define CMD_NUM 2

CommandHead gComHead[CMD_NUM] = {
  { 0x01, 0x01, 0x01, 0x0003}, // XYZ DelataCommand
  { 0x02, 0x01, 0x01, 0x0009}  // (X , Y , Z) POS
};

typedef struct XYZ_t
{
  byte x;
  byte y;
  byte z;
} XYZ;

typedef struct Moter_t
{
  byte moterID;
  byte pos;
} MoterPOS;

typedef struct ComPOS_t
{
  CommandHead head;
  XYZ pos;
} ComXYZ;

ComXYZ gxyz = { { 0x01, 0x01, 0x01, 0x0003}, { 100, 100, 100} };
ComXYZ gxyz_def = { { 0x01, 0x01, 0x01, 0x0003}, { 100, 100, 100} };

// SingletonHolderを使う際のお決まりtypedef
typedef tmlib::SingletonHolder<MyRingBuffer> RingHolder;
// 海外のコードでよくみかけるdefineやexternで使うthe...形式(ちょっとオシャレ?)
#define theRing RingHolder::getInstance()

//############  EventManager   ############
EventManager evtManager;
int gSettingTaskID;
int KillFlg = 0;
int Onekill = 0;

//############  EventManager   SettingLisnerTask   ############
struct PosSenerTask : public EventTask
{
  using EventTask::execute;

  int MoveMoter( int *index, int indexsize)
  {
    int ii;
    int bufii = 5;
    int moterID;
    int16_t movepos;
    byte buf[10];

    buf[0] = gComHead[1].main;
    buf[1] = gComHead[1].sub;
    buf[2] = gComHead[1].ver;
    buf[3] = (gComHead[1].len) & 0xff;
    buf[4] = (gComHead[1].len) >> 8;
    //buf[5] = moterID;
    //buf[6] = (*movepos) & 0xff;
    //buf[7] = (*movepos) >> 8;


    for ( ii = 0; ii < indexsize; ii++) {
      moterID = gMoterID[ (index[ii]) ];
      movepos = gpos[ (index[ii]) ];
      if ( movepos > defMaxPos[(index[ii])] ) {
        DebugSerialPrintln(F("defMaxPos"));
        movepos = defMaxPos[(index[ii])];
      } else if ( movepos < defMinPos[(index[ii])] ) {
        DebugSerialPrintln(F("defMinPos"));
        movepos = defMinPos[(index[ii])];
      }
      // Set goal position
      DebugSerialPrint(F("moterID: "));
      DebugSerialPrintln(moterID);
      DebugSerialPrint("movepos:  ");
      DebugSerialPrintln(movepos);
      buf[bufii++] = moterID;
      buf[bufii++] = movepos & 0xff;
      buf[bufii++] = movepos >> 8;
    }
#if 0
    char buf[255];
    sprintf(buf, "%d%d", moterID, *movepos);
    int n = strlen(buf);
    unsigned short crc = crc16(0, (unsigned char*)buf, n);
    // この電文形式はすべてASCIIなのでIDEのシリアルモニターで電文を確認できます
    sprintf(buf, "%s%02x%02x", buf, (crc >> 8), (crc & 0xff));
    if (POSSerial.availableForWrite()) {
      POSSerial.print(buf);
      POSSerial.print(';');
    } else {
      DebugSerial.println('  NOT  POSSerial.available');
    }

    if (DebugSerial.availableForWrite()) {
      DebugSerial.print("====POSSerial===");
      DebugSerial.print(buf);
      DebugSerial.println(';');
    }
#endif

    unsigned short conf_def_crc = crc16(0, (unsigned char*)buf, 5 + gComHead[1].len);
    buf[bufii++] = (conf_def_crc & 0xff);
    buf[bufii++] = (conf_def_crc >> 8);

    memcpy( gPosSendBuff, buf, bufii);

    if (POSSerial.availableForWrite()) {
      POSSerial.write(buf, bufii);
    }
    return 1;
  }

  void execute(Event evt) {
    if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
      int16_t movex = PS3.getAnalogHat(LeftHatX);
      int16_t movey = PS3.getAnalogHat(LeftHatY);
      int16_t movez = PS3.getAnalogHat(RightHatY);

      if (movex > POS_RANGE1 || movex < POS_RANGE2)
      {

        DebugSerialPrint(F("LeftHatX: "));
        DebugSerialPrint(LeftHatX);

        if ( movex > POS_RANGE1 ) {
          // 1はゼロ除算対策
          // モーターの配置の都合上、-1をかけて反対にする
          gpos[0] = gpos[0] - 1 /*+  -1 * (movex - POS_RANGE1 + 1) / 25*/ - (gxyz.pos.x - DEF_DELTA);
        } else {
          gpos[0] = gpos[0] + 1/*+  -1 * (movex - POS_RANGE1 + 1) / 25*/ + (gxyz.pos.x - DEF_DELTA);
        }
      }

      if (movey > POS_RANGE1 || movey < POS_RANGE2 )
      {
        DebugSerialPrint(F("\tLeftHatY: "));
        DebugSerialPrint(movey);
        if ( movey > POS_RANGE1 ) {
          // 1はゼロ除算対策
          gpos[1] = gpos[1] + 1/*+ (movey - POS_RANGE1 + 1) / 25*/ + (gxyz.pos.y - DEF_DELTA);
        } else {
          gpos[1] = gpos[1] - 1/*+ (movey - POS_RANGE2 + 1) / 25*/ - (gxyz.pos.y - DEF_DELTA);
        }
      }
      /*
        if( PS3.getAnalogHat(RightHatX) > 137 || PS3.getAnalogHat(RightHatX) < 117)
        {
        DebugSerialPrint(F("\tRightHatX: "));
        DebugSerialPrint(RightX);
        }
      */

      if ( movez > POS_RANGE1 || movez < POS_RANGE2)
      {
        DebugSerialPrint(F("\tRightHatY: "));
        DebugSerialPrintln(movez);

        if ( movez > POS_RANGE1 ) {
          // 1はゼロ除算対策
          gpos[2] = gpos[2] + 1/*+ (movez - POS_RANGE1 + 1) / 25*/ + (gxyz.pos.z - DEF_DELTA);
        } else {
          gpos[2] = gpos[2] - 1/*+ (movez - POS_RANGE2 + 1) / 25*/ - (gxyz.pos.z - DEF_DELTA);
        }
      }
      if ( (movex > POS_RANGE1 || movex < POS_RANGE2) ||
           (movey > POS_RANGE1 || movey < POS_RANGE2) || 
           ( movez > POS_RANGE1 || movez < POS_RANGE2) ) {
        int index[3] = {0, 1, 2};
        int indexsize = 3;
        MoveMoter( index, indexsize);
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

  }
} PosSenerTask;

//############  EventManager   SettingLisnerTask   ############

struct SettingLisnerTask : public EventTask
{
  using EventTask::execute;

  void execute(Event evt) {
    String extra = evt.extra;
#ifdef DEBUG_COMMAND
    DebugSerial.println("SettingLisnerTask execute ");
#endif
    if (extra == "Stop")
    {
    } else {
      int get_num;
      byte bufferSerialSetting[RING_BUF];
      //UARTから読み込み

      do {
        get_num = theRing.getCommand(bufferSerialSetting, gComHead, CMD_NUM);
        if ( 1 <= get_num ) {
          gxyz = *(ComXYZ*)(bufferSerialSetting);
          conf_ram.delta_x = gxyz.pos.x;
          conf_ram.delta_y = gxyz.pos.y;
          conf_ram.delta_z = gxyz.pos.z;
          write_config(conf_ram);
          KillFlg = 1;
        } else if ( -1 == get_num ) {
          ;//SettingSerial.println("F");
        }
      } while ( 1 <= get_num  );
    }
  }
} SettingLisnerTask;

void POSSerialEvent() {
  byte ch;
  //UARTから読み込み
  while (POSSerial.available()) {
    ch = POSSerial.read();

    if (ch == 'F') {
      ReSend();
    }
  }
}

void SettingSerialEvent() {
  byte ch;
  //UARTから読み込み
  while (SettingSerial.available()) {
    ch = SettingSerial.read();

    theRing.RingWrite(ch);

#ifdef DEBUG_SETTING
    theRing.RingPrint();
#endif
  }
}

//############  setup   ############
void setup() {
#ifdef DEBUG
  DebugSerial.begin(115200);// Debug
#endif
  POSSerial.begin(115200);// Connect MoterSlave
  SettingSerial.begin(115200);// Connect Setting

  // シングルトンなRingBufferクラスを生成
  RingHolder::create();
  theRing.Init();
  gxyz = gxyz_def;

  conf_ram = read_config();
  gxyz.pos.x = conf_ram.delta_x;
  gxyz.pos.y = conf_ram.delta_y;
  gxyz.pos.z = conf_ram.delta_z;

#ifdef DEBUG
  DebugSerial.println("=======CONFIG GET RESULT START =======");
  DebugSerial.println(conf_ram.delta_x);
  DebugSerial.println(conf_ram.delta_y);
  DebugSerial.println(conf_ram.delta_z);
  DebugSerial.println(conf_ram.crc);
  DebugSerial.println("=======GET RESULT END=======");
#endif
#ifdef USE_PS3
  if (Usb.Init() == -1) {
    DebugSerialPrintln("OSC did not start");
    // don't do anything more:
    wdt_enable(WDTO_1S);   // 1秒のウォッチドッグタイマーをセット
    return;
  }
#endif

  DebugSerialPrintln("Moter Control Started");

  evtManager.subscribe(Subscriber("event.SettingLisner", &SettingLisnerTask));
  //Event SettingLisner = Event("event.SettingLisner");
  Event SettingEvent = Event("event.SettingLisner", "Start");
  gSettingTaskID = evtManager.triggerInterval(TimedTask(500, SettingEvent));
  //evtManager.trigger(SettingEvent);

  evtManager.subscribe(Subscriber("event.PosSend", &PosSenerTask));
  Event poSsenderTsk = Event("event.PosSend");
  evtManager.triggerInterval(TimedTask(120, poSsenderTsk));

  startMs  = millis();
}

void loop() {

  evtManager.tick();

  unsigned long currentMs = millis();
  unsigned long difference = currentMs - startMs;
  if ( (Onekill == 0) && ((KillFlg == 1) || (difference >= ONE_RUN_TIMEOUT ))) {
    SettingSerial.end();// Stop Setting
    evtManager.kill(gSettingTaskID);
    KillFlg = 0;
    Onekill = 1;
  }

  if (!DebugSerial) {
    DebugSerial.end();
  }
  if (!POSSerial) {
    POSSerial.end();
  }
  if (!SettingSerial) {
    SettingSerial.end();
  }

  Usb.Task();

}

void ReSend()
{
  if (POSSerial.availableForWrite())
  {
#ifdef DEBUG
    DebugSerial.println("Resend");
#endif
    POSSerial.write(gPosSendBuff, 10);
  }
}




