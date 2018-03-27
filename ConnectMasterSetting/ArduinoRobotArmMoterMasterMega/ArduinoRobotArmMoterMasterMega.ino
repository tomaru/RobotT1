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
static int16_t defMaxPos[MOTER_NUM] = { 1000, 800, 1000 };//  0～1023
static int16_t defMinPos[MOTER_NUM] = { 10,   200, 10    };
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
#define CMD_NUM 1

CommandHead gComHead[CMD_NUM] = {
  { 0x01, 0x01, 0x01, 0x0011}  // Setting
};

CommandHead com_xyz = 
  { 0x02, 0x01, 0x01, 0x0009}; // (X , Y , Z) POS

CommandHead com_speedxyz = 
  { 0x02, 0x02, 0x01, 0x0009};  // XYZ SpeedCommand
 
typedef struct Moter_t
{
  byte moterID;
  byte pos;
} MoterPOS;

typedef struct ComPOS_t
{
  CommandHead head;
  unsigned short x;
  unsigned short y;
  unsigned short z;
  byte delta_x;
  byte delta_y;
  byte delta_z;
  unsigned short speed_x;
  unsigned short speed_y;
  unsigned short speed_z;
  byte ltime;
  byte mode;
} ComXYZ;

ComXYZ gxyz     = { { 0x01, 0x01, 0x01, 0x000b}, 500,500,500,110, 110, 110, 50,50,50,120,0 };
ComXYZ gxyz_def = { { 0x01, 0x01, 0x01, 0x000b}, 500,500,500,110, 110, 110, 50,50,50,120,0 };

// SingletonHolderを使う際のお決まりtypedef
typedef tmlib::SingletonHolder<MyRingBuffer> RingHolder;
// 海外のコードでよくみかけるdefineやexternで使うthe...形式(ちょっとオシャレ?)
#define theRing RingHolder::getInstance()

//############  EventManager   ############
EventManager evtManager;
int gSettingTaskID;
int KillFlg = 0;
int Onekill = 0;

int SendSettingSpeed()
{
    int bufii = 5;
    
    byte buf[5 + com_speedxyz.len + 2];
    buf[0] = com_speedxyz.main;
    buf[1] = com_speedxyz.sub;
    buf[2] = com_speedxyz.ver;
    buf[3] = (com_speedxyz.len) & 0xff;
    buf[4] = (com_speedxyz.len) >> 8;
    buf[bufii++] = gMoterID[2];
    buf[bufii++] = gxyz.speed_x & 0xff;
    buf[bufii++] = gxyz.speed_x >> 8;
    buf[bufii++] = gMoterID[1];
    buf[bufii++] = gxyz.speed_y & 0xff;
    buf[bufii++] = gxyz.speed_y >> 8;
    buf[bufii++] = gMoterID[0];
    buf[bufii++] = gxyz.speed_z & 0xff;
    buf[bufii++] = gxyz.speed_z >> 8;
    unsigned short conf_def_crc = crc16(0, (unsigned char*)buf, 5 + com_speedxyz.len);
    buf[bufii++] = (conf_def_crc & 0xff);
    buf[bufii++] = (conf_def_crc >> 8);
    
    if (POSSerial.availableForWrite()) {
      POSSerial.write(buf, bufii);
    }
      DebugSerialPrint(F("moterID: "));
      DebugSerialPrint(gMoterID[0]);
      DebugSerialPrint(F("   Speed: "));
      DebugSerialPrintln(gxyz.speed_x);
      DebugSerialPrint(F("moterID: "));
      DebugSerialPrint(gMoterID[1]);
      DebugSerialPrint(F("   Speed: "));
      DebugSerialPrintln(gxyz.speed_y);
      DebugSerialPrint(F("moterID: "));
      DebugSerialPrint(gMoterID[2]);
      DebugSerialPrint(F("   Speed: "));
      DebugSerialPrintln(gxyz.speed_z);
    return 1;
    
}
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

    buf[0] = com_xyz.main;
    buf[1] = com_xyz.sub;
    buf[2] = com_xyz.ver;
    buf[3] = (com_xyz.len) & 0xff;
    buf[4] = (com_xyz.len) >> 8;
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
#ifdef DEBUG_POS
      DebugSerialPrint(F("moterID: "));
      DebugSerialPrintln(moterID);
      DebugSerialPrint("movepos:  ");
      DebugSerialPrintln(movepos);
#endif
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

    unsigned short conf_def_crc = crc16(0, (unsigned char*)buf, 5 + com_xyz.len);
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

#ifdef DEBUG_POS
        DebugSerialPrint(F("LeftHatX: "));
        DebugSerialPrint(LeftHatX);
#endif

        if ( movex > POS_RANGE1 ) {
          // 1はゼロ除算対策
          // モーターの配置の都合上、-1をかけて反対にする
          gpos[0] = gpos[0] - 1 /*+  -1 * (movex - POS_RANGE1 + 1) / 25*/ - (gxyz.delta_x - DEF_DELTA);
        } else {
          gpos[0] = gpos[0] + 1 /*+  -1 * (movex - POS_RANGE1 + 1) / 25*/ + (gxyz.delta_x - DEF_DELTA);
        }
      }

      if (movey > POS_RANGE1 || movey < POS_RANGE2 )
      {
#ifdef DEBUG_POS
        DebugSerialPrint(F("\tLeftHatY: "));
        DebugSerialPrint(movey);
#endif
        if ( movey > POS_RANGE1 ) {
          // 1はゼロ除算対策
          gpos[1] = gpos[1] + 1 /*+ (movey - POS_RANGE1 + 1) / 25*/ + (gxyz.delta_y - DEF_DELTA);
        } else {
          gpos[1] = gpos[1] - 1 /*+ (movey - POS_RANGE2 + 1) / 25*/ - (gxyz.delta_y - DEF_DELTA);
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
#ifdef DEBUG_POS
        DebugSerialPrint(F("\tRightHatY: "));
        DebugSerialPrintln(movez);
#endif

        if ( movez > POS_RANGE1 ) {
          // 1はゼロ除算対策
          gpos[2] = gpos[2] + 1/*+ (movez - POS_RANGE1 + 1) / 25*/ + (gxyz.delta_z - DEF_DELTA);
        } else {
          gpos[2] = gpos[2] - 1/*+ (movez - POS_RANGE2 + 1) / 25*/ - (gxyz.delta_z - DEF_DELTA);
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
#ifdef DEBUG
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
          conf_ram.x = gxyz.x;
          conf_ram.y = gxyz.y;
          conf_ram.z = gxyz.z;
#ifdef DEBUG
    DebugSerial.print("conf_ram.x  "); DebugSerial.println(conf_ram.x);
    DebugSerial.print("conf_ram.y  "); DebugSerial.println(conf_ram.y);
    DebugSerial.print("conf_ram.z  "); DebugSerial.println(conf_ram.z);
#endif
          conf_ram.delta_x = gxyz.delta_x;
          conf_ram.delta_y = gxyz.delta_y;
          conf_ram.delta_z = gxyz.delta_z;
#ifdef DEBUG
    DebugSerial.print("conf_ram.delta_x  "); DebugSerial.println(conf_ram.delta_x);
    DebugSerial.print("conf_ram.delta_y  "); DebugSerial.println(conf_ram.delta_y);
    DebugSerial.print("conf_ram.delta_z  "); DebugSerial.println(conf_ram.delta_z);
#endif
          conf_ram.speed_x = gxyz.speed_x;
          conf_ram.speed_y = gxyz.speed_y;
          conf_ram.speed_z = gxyz.speed_z;
#ifdef DEBUG
    DebugSerial.print("gxyz.speed_x  "); DebugSerial.println(gxyz.speed_x);
    DebugSerial.print("gxyz.speed_y  "); DebugSerial.println(gxyz.speed_y);
    DebugSerial.print("gxyz.speed_z  "); DebugSerial.println(gxyz.speed_z);
#endif
          conf_ram.ltime = gxyz.ltime;
          conf_ram.mode = gxyz.mode;
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
  gxyz.x = conf_ram.x;
  gxyz.y = conf_ram.y;
  gxyz.z = conf_ram.z;
  gxyz.delta_x = conf_ram.delta_x;
  gxyz.delta_y = conf_ram.delta_y;
  gxyz.delta_z = conf_ram.delta_z;
  gxyz.speed_x = conf_ram.speed_x;
  gxyz.speed_y = conf_ram.speed_y;
  gxyz.speed_z = conf_ram.speed_z;
  gxyz.ltime = conf_ram.ltime;
  gxyz.mode = conf_ram.mode;
    
  SendSettingSpeed();

#ifdef DEBUG
  DebugSerial.println("=======CONFIG GET RESULT START =======");
  DebugSerial.println(conf_ram.x);
  DebugSerial.println(conf_ram.y);
  DebugSerial.println(conf_ram.z);
  DebugSerial.println(conf_ram.delta_x);
  DebugSerial.println(conf_ram.delta_y);
  DebugSerial.println(conf_ram.delta_z);
  DebugSerial.println(conf_ram.crc);
  DebugSerial.println("=======GET RESULT END=======");
#endif
  gpos[0] = gxyz.x;
  gpos[1] = gxyz.y;
  gpos[2] = gxyz.z;

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
  evtManager.triggerInterval(TimedTask(gxyz.ltime, poSsenderTsk));

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




