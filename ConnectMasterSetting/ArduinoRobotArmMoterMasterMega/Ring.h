#ifndef Ring_h
#define Ring_h

#include "Arduino.h"
extern "C" {
#include "CalcCRC.h"
}
#include "Singleton.h"

//#define DEBUG_COMMAND
#define DebugSerial       Serial

//############  RingBuffer   ############
#define HEADER_LEN 6
#define CRC_SIZE 2
#define RING_BUF 256 // 2のべき乗である必要がある

typedef struct COMMAND_HEAD_t
{
  //byte == unsigned char
  unsigned char main;
  unsigned char sub;
  unsigned char ver;
  unsigned short len;
  byte dummy;
} CommandHead;

class MyRingBuffer
{
  public:
    // シングルトンのテンプレート引数に自身を指定したものだけに生成をゆるす
    friend tmlib::SingletonHolder<MyRingBuffer>;

   void Init(void) {
      // 初期値を設定する
      // 読み込み位置
      read_pos = &data[0];
      // 書き込み位置
      write_pos = &data[0];
      // 確保した配列の最初を指す
      start_pos = &data[0];
      // 確保した配列の最後を指す
      end_pos = &data[RING_BUF - 1];
      // これでマスクすると剰余を求めなくて済む
      // ただしこれで配列要素を指定できるのは、リングバッファが2^nの制限があるため
      mask = RING_BUF - 1;
      // 読み込んだbyte数は0(byte)
      m_len = 0;
    };
  public:
    byte *read_pos;
    byte *write_pos;
    byte *start_pos;
    byte *end_pos;
    int m_len;
    byte data[RING_BUF];
    unsigned int mask = (RING_BUF - 1);
  public:
    void RingInit();
    void RingPrint();
    void RingPrint2(byte *out, int len);
    int RingGet(byte (&out)[RING_BUF], int maxlen);
    int RingGet(byte *lbufferSerial, CommandHead *head, int cmd_num);
    int RingSize();
    int RingWrite(byte data);
    int RingReadPosAdd(int add);
    int getCommand(byte *lbufferSerial, CommandHead *head, int cmd_num);
    byte* nextpos(byte *);
} ;

#endif
