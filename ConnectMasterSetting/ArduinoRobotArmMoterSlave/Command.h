#ifndef Command_h
#define Command_h

extern "C" {
#include "CalcCRC.h"
}

//############  Command   ############
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


int getCommand(byte *lbufferSerial, CommandHead *head, int cmd_num)
{
  int ii;
  unsigned short len = 0;
  byte lCmd[RING_BUF];// 本当は電文の最大長 + alpha を確保する。ので間違いではない
  int fail = 1;// デフォルトは電文取得に失敗
  // 最小の電文サイズを超えたら電文を確認するようにしないと無駄な確認が多くなり制御に支障が出ます
  if ( HEADER_LEN + 3 + 2 <= RingSize() ) {
    RingGetCommand(lCmd, HEADER_LEN );
  } else {
    return 0;
  }
#ifdef DEBUG_COMMAND
  RingPrint();
#endif

  // ヘッダ部の読み込み
  while (RingSize() >= (HEADER_LEN) ) {
#ifdef DEBUG_SETTING
    RingPrint2(lCmd, HEADER_LEN);
#endif
    for ( ii = 0; ii < cmd_num; ii++ ) {
      if (head[ii].main == lCmd[0]) {
        if (head[ii].sub == lCmd[1]) {
          if (head[ii].ver == lCmd[2]) {
            len = *((unsigned short*)(&lCmd[3]));
#ifdef DEBUG_SETTING
            DebugSerial.print("len = "); DebugSerial.println(len);
#endif
            if ( head[ii].len != len ) {
              len = 0;
              // レングス不正のため次の位置に更新（2byteのためこことforのあとので次にする）
              RingReadPosAdd(1);
              // ヘッダ取得に失敗したことを記録する
              fail = 1;
            } else {
              fail = 0;
              break;
            }
          } else {
#ifdef DEBUG_COMMAND
            DebugSerial.print(".ver = "); DebugSerial.println(lCmd[2]);
#endif
          }
        } else {
#ifdef DEBUG_COMMAND
          DebugSerial.print(".sub = "); DebugSerial.println(lCmd[1]);
#endif
        }
      } else {
#ifdef DEBUG_COMMAND
        DebugSerial.print(".main = "); DebugSerial.println(lCmd[0]);
#endif
      }
    }
    if (fail == 0) {
      // 成功したらwhileを抜ける
      break;
    } else {
      // ヘッダ取得失敗のためリングバッファを1byte次にする
      RingReadPosAdd(1);
      RingGetCommand(lCmd, HEADER_LEN );
    }
  }

#ifdef DEBUG_SETTING
  DebugSerial.print("Size = "); DebugSerial.println(RingSize());
#endif
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
      memcpy(lbufferSerial, lCmd, HEADER_LEN + len + CRC_SIZE);
#ifdef DEBUG_COMMAND
      DebugSerial.println("Success");
#endif
#ifdef DEBUG_COMMAND
      RingPrint2(lbufferSerial, HEADER_LEN + len + CRC_SIZE);
#endif
      // 次の位置に更新
      RingReadPosAdd((HEADER_LEN + len + CRC_SIZE));
      return (HEADER_LEN + len + CRC_SIZE);
    } else {
      // 電文の取得に失敗したので
      // 次の位置に更新
      RingReadPosAdd(1);
#ifdef DEBUG_COMMAND
      DebugSerial.println("Fail");
#endif
      return -1;
    }
  }
END:
  return 0;
}

#endif
