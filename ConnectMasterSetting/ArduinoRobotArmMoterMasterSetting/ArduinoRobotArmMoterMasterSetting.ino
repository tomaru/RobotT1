// Set Goal Position
//  Move position to the full range
//  Since software serial is selected, dynamixel should be use at 57600bps or less.
//  1.select [SOFT]
//  2.compile & download
//#include "EEPROMAnything.h"
#define DEBUG_CONFIG
#define DEBUG_Setting
#define DEBUG
#include "Event.h"
#include "EEPROMConfig.h"
extern "C" {
#include "CalcCRC.h"
}
#include <avr/wdt.h> //WatchDogTimer

#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

//############  MoterMasterCommand   ############
#define HEADER_LEN 6

typedef struct COMMAND_HEAD_t
{
  //byte == unsigned char
  unsigned char main;
  unsigned char sub;
  unsigned char ver;
  unsigned short len;
} CommandHead;

static const CommandHead gcomhead[1] = {
  { 0x01, 0x01, 0x01, 0x000e}
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


//############  PageInfo   ############
typedef struct page_info
{
  byte page_menu_no;
  byte page_menu_cnt;
  byte page_menu_current;
  int (*pFun[3])(int up_down);
} PAGE_INFO;

int mnt_pos_x(int up_down);
int mnt_pos_y(int up_down);
int mnt_pos_z(int up_down);
int mnt_delta_x(int up_down);
int mnt_delta_y(int up_down);
int mnt_delta_z(int up_down);
int mnt_speed_x(int up_down);
int mnt_speed_y(int up_down);
int mnt_speed_z(int up_down);
int mnt_time(int x);
int mnt_mode(int x);

PAGE_INFO gpages[] = {
  {
    1, //byte page_menu_no; 1,2,3
    3, //byte page_menu_cnt;
    0, //int page_menu_current;
    {mnt_pos_x, mnt_pos_y, mnt_pos_z}, //int (*pFun[3])(int);
  },
  {
    4, //byte page_menu_no; 4,5,6
    3, //byte page_menu_cnt;
    0, //int page_menu_current;
    {mnt_delta_x, mnt_delta_y, mnt_delta_z}, //int (*pFun[3])(int);
  },
  {
    7, //byte page_menu_no;7,8,9
    3, //byte page_menu_cnt;
    0, //int page_menu_default;
    {mnt_speed_x, mnt_speed_y, mnt_speed_z}, //int (*pFun[3])(int);
  },
  {
    10, //byte page_menu_no;
    1, //byte page_menu_cnt;
    0, //int page_menu_default;
    {mnt_time, NULL, NULL}, //int (*pFun[3])(int);
  },
  {
    11, //byte page_menu_no;
    1, //byte page_menu_cnt;
    0, //int page_menu_default;
    {mnt_mode, NULL, NULL}, //int (*pFun[3])(int);
  }
};

char gPageTxt[][17] = {
  "                ", // [0]
  "POS X           ", // [1]
  "POS Y           ", // [2]
  "POS Z           ", // [3]
  "Delta X         ", // [4]
  "Delta Y         ", // [5]
  "Delta Z         ", // [6]
  "SPPED X         ", // [7]
  "SPPED Y         ", // [8]
  "SPPED Z         ", // [9]
  "TIME            ", // [10]
  "MODE            ", // [11]
};

int pages_index = 0;

struct SendTask : public EventTask
{
  using EventTask::execute;

  void execute(Event evt)
  {
    byte buf[30];
    int ii=0;
    buf[ii++] = gcomhead[0].main;
    buf[ii++] = gcomhead[0].sub;
    buf[ii++] = 1;//Version
    buf[ii++] = (gcomhead[0].len) & 0xff;
    buf[ii++] = (gcomhead[0].len) >> 8;
    buf[ii++] = 0xFF;
    buf[ii++] = conf_ram.x & 0xff;
    buf[ii++] = conf_ram.x >> 8;
    buf[ii++] = conf_ram.y & 0xff;
    buf[ii++] = conf_ram.y >> 8;
    buf[ii++] = conf_ram.z & 0xff;
    buf[ii++] = conf_ram.z >> 8;
    buf[ii++] = conf_ram.delta_x;
    buf[ii++] = conf_ram.delta_y;
    buf[ii++] = conf_ram.delta_z;
    buf[ii++] = conf_ram.speed_x;
    buf[ii++] = conf_ram.speed_y;
    buf[ii++] = conf_ram.speed_z;
    buf[ii++] = conf_ram.ltime;
    buf[ii++] = conf_ram.mode;

    unsigned short conf_def_crc = crc16(0, (unsigned char*)buf, HEADER_LEN + gcomhead[0].len);
    buf[ii++] = (conf_def_crc & 0xff);
    buf[ii++] = (conf_def_crc >> 8);
    Serial3.write(buf, ii);
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
#ifdef DEBUG_Setting
  Serial.println("=======CONFIG GET RESULT START =======");
  Serial.println(conf_ram.ver);
  Serial.println(conf_ram.x);
  Serial.println(conf_ram.y);
  Serial.println(conf_ram.z);
  Serial.println(conf_ram.delta_x);
  Serial.println(conf_ram.delta_y);
  Serial.println(conf_ram.delta_z);
  Serial.println(conf_ram.speed_x);
  Serial.println(conf_ram.speed_y);
  Serial.println(conf_ram.speed_z);
  Serial.println(conf_ram.ltime);
  Serial.println(conf_ram.mode);
  Serial.println(conf_ram.crc);
  Serial.println("=======GET RESULT END=======");
#endif

  lcd.clear();
  lcd.setCursor(0, 0);// 液晶の 行数、列数 を設定
  lcd.print("  StartSetting");
  lcd.setCursor(0, 1);// 液晶の 行数、列数 を設定
  lcd.print("     Master");
  delay(1000);
  MyMenuControl(gpages, sizeof(gpages) / sizeof(gpages[0]));
}

void loop() {
  Display();
  evtManager.tick();

  delay(30);

}

int mnt_pos_x(int up_down)
{
  conf_ram.x = conf_ram.x + up_down*50;
  write_config(conf_ram);
  return conf_ram.x;
}

int mnt_pos_y(int up_down)
{
  conf_ram.y = conf_ram.y + up_down*50;
  write_config(conf_ram);
  return conf_ram.y;
}

int mnt_pos_z(int up_down)
{
  conf_ram.z = conf_ram.z + up_down*50;
  write_config(conf_ram);
  return conf_ram.z;
}

int mnt_delta_x(int up_down)
{
  conf_ram.delta_x = conf_ram.delta_x + up_down;
  write_config(conf_ram);
  return conf_ram.delta_x;
}

int mnt_delta_y(int up_down)
{
  conf_ram.delta_y = conf_ram.delta_y + up_down;
  write_config(conf_ram);
  return conf_ram.delta_y;
}

int mnt_delta_z(int up_down)
{
  conf_ram.delta_z = conf_ram.delta_z + up_down;
  write_config(conf_ram);
  return conf_ram.delta_z;
}

int mnt_speed_x(int up_down)
{
  conf_ram.speed_x = conf_ram.speed_x + up_down;
  write_config(conf_ram);
  return conf_ram.speed_x;
}

int mnt_speed_y(int up_down)
{
  conf_ram.speed_y = conf_ram.speed_y + up_down;
  write_config(conf_ram);
  return conf_ram.speed_y;
}

int mnt_speed_z(int up_down)
{
  conf_ram.speed_z = conf_ram.speed_z + up_down;
  write_config(conf_ram);
  return conf_ram.speed_z;
}

int mnt_time(int up_down)
{
  conf_ram.ltime = conf_ram.ltime + up_down;
  write_config(conf_ram);
  return conf_ram.ltime;
}

int mnt_mode(int up_down)
{
  conf_ram.mode = (conf_ram.mode + up_down) % 2;
  write_config(conf_ram);
  return conf_ram.mode;
}

void MyMenuControl(PAGE_INFO *pages, unsigned short page_cnt)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print( gPageTxt[ (pages[pages_index].page_menu_no + pages[pages_index].page_menu_current) ] );
  lcd.setCursor(0, 1);
  int (*pMenuFunc)(int) = 0;
  pMenuFunc = pages[pages_index].pFun[ pages[ pages_index ].page_menu_current ];
  int to_str = pMenuFunc(0);
  lcd.print( String(to_str) );
}

int MyMenuControl_NextPage( PAGE_INFO *pages, unsigned short page_cnt)
{
  pages_index = (pages_index + 1) % page_cnt;
  // ページ切り替えで、最初に戻したい場合に以下を有効化する
  //pages[ pages_index ].page_menu_current = 0;
}

int MyMenuControl_Right( PAGE_INFO *pages, unsigned short page_cnt)
{
  pages[ pages_index ].page_menu_current = ( pages[ pages_index ].page_menu_current + 1 ) % pages[ pages_index ].page_menu_cnt;
}

int MyMenuControl_Left( PAGE_INFO *pages, unsigned short page_cnt)
{
  if ( pages[ pages_index ].page_menu_current == 0 ) {
    pages[ pages_index ].page_menu_current =  pages[ pages_index ].page_menu_cnt - 1;
  } else {
    pages[ pages_index ].page_menu_current = ( pages[ pages_index ].page_menu_current - 1 ) % pages[ pages_index ].page_menu_cnt;
  }
}

int MyMenuControl_Up( PAGE_INFO *pages, unsigned short page_cnt)
{
  int (*pMenuFunc)(int) = 0;
  pMenuFunc = pages[ pages_index ].pFun[ pages[ pages_index ].page_menu_current ];
  pMenuFunc(1);
}

int MyMenuControl_Down( PAGE_INFO *pages, unsigned short page_cnt)
{
  int (*pMenuFunc)(int) = 0;
  pMenuFunc = pages[ pages_index ].pFun[ pages[ pages_index ].page_menu_current ];
  pMenuFunc(-1);
}

void Display()
{
  int adc_key_in;
  int key = -1;
  int trigger = 0;

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

  if ( 1 == trigger ) {
    switch (diskey) {
      case BUTTON_SELECT:
        MyMenuControl_NextPage(gpages, sizeof(gpages) / sizeof(gpages[0]));
        break;
      case BUTTON_UP:
        MyMenuControl_Up(gpages, sizeof(gpages) / sizeof(gpages[0]));
        break;
      case BUTTON_DOWN:
        MyMenuControl_Down(gpages, sizeof(gpages) / sizeof(gpages[0]));
        break;
      case BUTTON_LEFT:
        MyMenuControl_Left(gpages, sizeof(gpages) / sizeof(gpages[0]));
        break;
      case BUTTON_RIGHT:
        MyMenuControl_Right(gpages, sizeof(gpages) / sizeof(gpages[0]));
        break;
    }
    MyMenuControl(gpages, sizeof(gpages) / sizeof(gpages[0]));
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
