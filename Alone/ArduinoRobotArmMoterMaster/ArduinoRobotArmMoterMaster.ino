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
#include <PS3USB.h>

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

//#define DEBUG

#define USE_PS3
//#define USE_DX
#define POS_RANGE1 137
#define POS_RANGE2 117

#ifdef DEBUG    //Macros are usually in all capital letters.
#define DebugSerialPrint(...)    Serial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
#define DebugSerialPrintln(...)  Serial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#else
#define DebugSerialPrint(...)     //now defines a blank line
#define DebugSerialPrintln(...)   //now defines a blank line
#endif
void setup() {
#ifdef USE_DX
  dxif.begin (57143);
#endif
  Serial.begin(115200);
  Serial3.begin(115200);

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
  for( ii=0; ii < MOTER_NUM; ii++) {
    dxif.ReadWordData (gMoterID[ii], REG_MOTER_MAX_POS, (uint16_t *)&gMaxPos[ii], NULL);
    DebugSerialPrint("REG_MOTER_MAX_POS  ");
    DebugSerialPrintln(gMaxPos[ii]);
    dxif.ReadWordData (gMoterID[ii], REG_MOTER_MIN_POS, (uint16_t *)&gMinPos[ii], NULL);
    DebugSerialPrint("REG_MOTER_MIN_POS  ");
    DebugSerialPrintln(gMinPos[ii]);
  }
#endif
  DebugSerialPrintln("Moter Control Started");
}

void loop() {
#ifdef USE_PS3
  Usb.Task();
  
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    int16_t movex = PS3.getAnalogHat(LeftHatX);
    
    if (movex > POS_RANGE1 || movex < POS_RANGE2)
    {
      
      DebugSerialPrint(F("LeftHatX: "));
      DebugSerialPrint(LeftHatX);
      
      if( movex > POS_RANGE1 ) {
        gpos[0] = gpos[0] + (movex-POS_RANGE1+1)/25;
      } else {
        gpos[0] = gpos[0] + (movex-POS_RANGE1+1)/25;
      }
      MoveMoter( 0, gMoterID[0], &(gpos[0]));
    }
    
    if( PS3.getAnalogHat(LeftHatY) > POS_RANGE1 || PS3.getAnalogHat(LeftHatY) < POS_RANGE2 )
    {
      int16_t movey = PS3.getAnalogHat(LeftHatY);
      DebugSerialPrint(F("\tLeftHatY: "));
      DebugSerialPrint(movey);
      if( movey > POS_RANGE1 ) {
        gpos[1] = gpos[1] + (movey-POS_RANGE1+1) / 25;
      } else {
        gpos[1] = gpos[1] + (movey-POS_RANGE2+1) / 25;
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
    
    if( PS3.getAnalogHat(RightHatY) > POS_RANGE1 || PS3.getAnalogHat(RightHatY) < POS_RANGE2)
    {
      int16_t movez = PS3.getAnalogHat(RightHatY);
      DebugSerialPrint(F("\tRightHatY: "));
      DebugSerialPrintln(movez);  
      
      if( movez > POS_RANGE1 ) {
        gpos[2] = gpos[2] + (movez-POS_RANGE1+1) / 25;
      } else {
        gpos[2] = gpos[2] + (movez-POS_RANGE2+1) / 25;
      }
      MoveMoter( 2, gMoterID[2], &(gpos[2]));
    }
  }else if (PS3.PS3MoveConnected) { // One can only set the color of the bulb, set the rumble, set and get the bluetooth address and calibrate the magnetometer via USB
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

int MoveMoter( int index,int moterID, int16_t *movepos)
{
    if( *movepos > defMaxPos[index] ) {
      DebugSerialPrintln(F("defMaxPos"));
      *movepos = defMaxPos[index];
    } else if( *movepos < defMinPos[index] ) {
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

    sprintf(buf, "%s%02x%02x", buf, (crc >> 8), (crc & 0xff));
    Serial.print(buf);
    Serial.print(';');
   
    return 1;
}
