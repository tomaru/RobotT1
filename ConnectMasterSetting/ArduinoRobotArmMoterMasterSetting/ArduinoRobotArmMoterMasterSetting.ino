// Set Goal Position
//  Move position to the full range
//  Since software serial is selected, dynamixel should be use at 57600bps or less.
//  1.select [SOFT]
//  2.compile & download
//#include "EEPROMAnything.h"

//#define DEBUG
#include "Event.h"
#include "EEPROMConfig.h"
extern "C" {
#include "CalcCRC.h"
}
#include <avr/wdt.h> //WatchDogTimer

#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

//############  MoterMasterCommand   ############
#define HEADER_LEN 5

typedef struct COMMAND_HEAD_t
{
  //byte == unsigned char
  unsigned char main;
  unsigned char sub;
  unsigned char ver;
  unsigned short len;
} CommandHead;

static const CommandHead gcomhead[1] = {
  { 0x01, 0x01, 0x01, 0x0003}
};

//############  LCD   ############
#define BUTTON_RIGHT  0
#define BUTTON_UP     1
#define BUTTON_DOWN   2
#define BUTTON_LEFT   3
#define BUTTON_SELECT 4

const int NUM_KEYS = 5;
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
int oldkey = -1;
int diskey = BUTTON_SELECT;
int adc_key_val[NUM_KEYS] = {70, 240, 420, 620, 880}; // キーの閾値 (個体によって値は異なる)

//############  Config Button   ############
#define SELECT_X 0
#define SELECT_Y 1
#define SELECT_Z 2
int gButtonState = 0; // 0 = x,  1 = y, 2 = z

//############  EEPROM   ############
configuration conf_ram;

//############  EventManager   ############
EventManager evtManager;

struct SendTask : public EventTask
{
  using EventTask::execute;

  void execute(Event evt)
  {
    byte buf[10];

    buf[0] = gcomhead[0].main;
    buf[1] = gcomhead[0].sub;
    buf[2] = 1;//Version
    buf[3] = (gcomhead[0].len) & 0xff;
    buf[4] = (gcomhead[0].len) >> 8;
    buf[5] = conf_ram.delta_x;
    buf[6] = conf_ram.delta_y;
    buf[7] = conf_ram.delta_z;

    unsigned short conf_def_crc = crc16(0, (unsigned char*)buf, HEADER_LEN + 3);
    buf[8] = (conf_def_crc & 0xff);
    buf[9] = (conf_def_crc >> 8);
    Serial3.write(buf, 10);
  }
} SendTask;

#ifdef DEBUG    //Macros are usually in all capital letters.
#define DebugSerialPrint(...)    Serial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
#define DebugSerialPrintln(...)  Serial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#else
#define DebugSerialPrint(...)     //now defines a blank line
#define DebugSerialPrintln(...)   //now defines a blank line
#endif

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);

  evtManager.subscribe(Subscriber("event.keepAlive", &SendTask));
  Event keepAlive = Event("event.keepAlive");
  evtManager.triggerInterval(TimedTask(1000, keepAlive));

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);           //カーソルの移動
  lcd.print("ReadSetting");

  conf_ram = read_config();
#ifdef DEBUG
  Serial.println("=======CONFIG GET RESULT START =======");
  Serial.println(conf_ram.delta_x);
  Serial.println(conf_ram.delta_y);
  Serial.println(conf_ram.delta_z);
  Serial.println(conf_ram.crc);
  Serial.println("=======GET RESULT END=======");
#endif

  lcd.clear();
  lcd.setCursor(0, 0);// 液晶の 行数、列数 を設定
  lcd.print("  StartSetting");
  lcd.setCursor(0, 1);// 液晶の 行数、列数 を設定
  lcd.print("     Master");
  delay(1000);
}

void loop() {
  Display();
  evtManager.tick();

  delay(30);

}

void Display()
{
  int adc_key_in;
  int key = -1;
  int trigger = 0;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("x=" + String(conf_ram.delta_x) + "y=" + String(conf_ram.delta_y) + "z=" + String(conf_ram.delta_z));

  adc_key_in = analogRead(0);
  key = get_key(adc_key_in);
  if (key != oldkey)
  {
    delay(30);
    adc_key_in = analogRead(0);
    key = get_key(adc_key_in);
    if (key != oldkey)
    {
      oldkey = key;
      if ( key >= 0 ) {
        diskey = key;
        trigger = 1;
      }
    }
  }
  lcd.setCursor(0, 1);
  switch (diskey) {
    case BUTTON_SELECT:
      switch ( gButtonState ) {
        case SELECT_X:
          lcd.print("Sellect X");
          break;
        case SELECT_Y:
          lcd.print("Sellect Y");
          break;
        case SELECT_Z:
          lcd.print("Sellect Z");
          break;
      }
      break;
    case BUTTON_UP:
      lcd.print("UP ");
      switch ( gButtonState ) {
        case SELECT_X:
          if (trigger == 1) {
            conf_ram.delta_x++;
            write_config(conf_ram);
          }
          lcd.print("X=" + String(conf_ram.delta_x));
          break;
        case SELECT_Y:
          if (trigger == 1) {
            conf_ram.delta_y++;
            write_config(conf_ram);
          }
          lcd.print("Y=" + String(conf_ram.delta_y));
          break;
        case SELECT_Z:
          if (trigger == 1) {
            conf_ram.delta_z++;
            write_config(conf_ram);
          }
          lcd.print("Z=" + String(conf_ram.delta_z));
          break;
      }
      break;
    case BUTTON_DOWN:
      lcd.print("DOWN ");
      switch ( gButtonState ) {
        case SELECT_X:
          if (trigger == 1) {
            conf_ram.delta_x--;
            write_config(conf_ram);
          }
          lcd.print("X=" + String(conf_ram.delta_x));
          break;
        case SELECT_Y:
          if (trigger == 1) {
            conf_ram.delta_y--;
            write_config(conf_ram);
          }
          lcd.print("Y=" + String(conf_ram.delta_y));
          break;
        case SELECT_Z:
          if (trigger == 1) {
            conf_ram.delta_z--;
            write_config(conf_ram);
          }
          lcd.print("Z=" + String(conf_ram.delta_z));
          break;
      }

      break;
    case BUTTON_LEFT:
      lcd.print("LEFT ");
      switch ( gButtonState ) {
        case SELECT_X:
          if (trigger == 1) {
            gButtonState = SELECT_Z;
          }
          lcd.print("Select X");
          break;
        case SELECT_Y:
          if (trigger == 1) {
            gButtonState = SELECT_X;
          }
          lcd.print("Select Y");
          break;
        case SELECT_Z:
          if (trigger == 1) {
            gButtonState = SELECT_Y;
          }
          lcd.print("Select Z");
          break;
      }
      break;
    case BUTTON_RIGHT:
      lcd.print("RIGHT ");
      switch ( gButtonState ) {
        case SELECT_X:
          if (trigger == 1) {
            gButtonState = SELECT_Y;
          }
          lcd.print("Select X");
          break;
        case SELECT_Y:
          if (trigger == 1) {
            gButtonState = SELECT_Z;
          }
          lcd.print("Select Y");
          break;
        case SELECT_Z:
          if (trigger == 1) {
            gButtonState = SELECT_X;
          }
          lcd.print("Select Z");
          break;
      }
      break;
  }
}

int get_key(unsigned int input)
{
  int k;
  for (k = 0; k < NUM_KEYS; k++)
    if (input < adc_key_val[k])
      return k;
  if (k >= NUM_KEYS)
    k = -1;
  return k;
}

void utf_del_uni(char *s) {
  byte i = 0;
  byte j = 0;
  while (s[i] != '\0') {
    if ((byte)s[i] == 0xEF) {
      if ((byte)s[i + 1] == 0xBE) s[i + 2] += 0x40;
      i += 2;
    }
    s[j] = s[i];
    i++;
    j++;
  }
  s[j] = '\0';
}
