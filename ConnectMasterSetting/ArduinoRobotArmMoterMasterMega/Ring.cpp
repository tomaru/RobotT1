#include "Ring.h"
// Debug用にリングバッファの状態を出力する

// 次の要素位置を特定する。こっちのほうが下より正確
//#define nextpos(a)  (byte *)(ring.start_pos + ((unsigned int)(((a - ring.start_pos == 0 ? sizeof(byte) : (byte)(a - ring.start_pos))/sizeof(byte))+1 ) & (unsigned int)(ring.mask)))
#define nextpos(a)  (byte *)(start_pos + ((unsigned int)(((a - start_pos == 0 ? sizeof(byte) : (byte)(a - start_pos))/sizeof(byte))+1 ) & (unsigned int)(mask)))
// 上を端折ったやつ。
//#define nextpos(a)  (byte *)(ring.start_pos + ((unsigned int)((a - ring.start_pos) + 1) & (unsigned int)(ring.mask)))

/*
  byte*  RingBuffer::nextpos(byte *a) {
  return (byte *)(start_pos + ((unsigned int)(((a - start_pos == 0 ? sizeof(byte) : (byte)(a - start_pos)) / sizeof(byte)) + 1 ) & (unsigned int)(mask)));
  }
*/

void MyRingBuffer::RingPrint()
{
  int ii;
  char printbuf[255];
  for (ii = 0; ii < RING_BUF ; ii++ ) {
    sprintf( printbuf, "%02x ", data[ii] );
#ifdef DEBUG
    DebugSerial.print(printbuf);
#endif
  }
#ifdef DEBUG
  DebugSerial.println("");
#endif
}

// 引数の配列を出力する（リングバッファ用）
void MyRingBuffer::RingPrint2(byte *out, int locallen)
{
  int ii;
  int jj;
  int ones = 0;
  char printbuf[255];
  for (ii = 0; ii < RING_BUF && ii < locallen ; ii++ ) {
    sprintf( printbuf, "%02x ", *(out + ii) );
#ifdef DEBUG
    DebugSerial.print(printbuf);
#endif
  }
#ifdef DEBUG
  DebugSerial.println("");
#endif
}

// 引数のoutにリングバッファから取り出した文字列を設定する
int MyRingBuffer::RingGet(byte (&out)[RING_BUF], int maxlen)
{
  unsigned int ii;
  unsigned int jj = 0;
  int ones = 0;
  int locallen = 0;
  char printbuf[255];

  for (ii = 0; (ii < RING_BUF) && (ii < maxlen) && ((read_pos + ii) != write_pos); ii++ ) {
    if ( ((read_pos + ii) > (end_pos)) &&  (ones == 0) ) {
      ones = 1;
      // 4つ{0,1,2,3}で、要素1の入力なら、iiは2になっている
      jj = ii;
    }
    if (ones == 1) {
      out[ii] = *(start_pos + (ii - jj & mask)) ;
    } else {
      out[ii] = *(read_pos + ii) ;
    }
    locallen++;
  }
  return locallen;
}

int MyRingBuffer::RingSize()
{
  return m_len;
}

// リングバッファに引数のdataを一番後ろに設定する
int MyRingBuffer::RingWrite(byte data)
{
  *(write_pos) = data;
  write_pos = nextpos(write_pos) ;// Next Posision
  m_len++;
  return 1;
}

// リングバッファの読み込み位置をaddの分だけ加算する
int MyRingBuffer::RingReadPosAdd(int add)
{
  read_pos = nextpos(read_pos);// Next Posision
  m_len--;
  return 1;
}
int MyRingBuffer::getCommand(byte *lbufferSerial, CommandHead *head, int headnum)
{
  int ii;
  unsigned short locallen = 0;
  byte lCmd[RING_BUF];// 本当は電文の最大長 + alpha を確保する。ので間違いではない
  int fail = 1;// デフォルトは電文取得に失敗
  int getlen = 0;

  if ( HEADER_LEN <= RingSize() ) {
    getlen = RingGet(lCmd, HEADER_LEN );
  } else {
#ifdef DEBUG_COMMAND
    DebugSerial.println("MIN HEADER_LEN");
#endif
    return 0;
  }

#ifdef DEBUG_COMMAND
  RingPrint();
#endif
#ifdef DEBUG_COMMAND
  RingPrint2(lCmd, getlen);//DebugCode
#endif

  // ヘッダ部の読み込み
  while (RingSize() >= (HEADER_LEN) ) {
    for ( ii = 0; ii < headnum; ii++ ) {
      if (head[ii].main == lCmd[0]) {
        if (head[ii].sub == lCmd[1]) {
          if (head[ii].ver == lCmd[2]) {
            locallen = *((unsigned short*)(&lCmd[3]));
            if ( head[ii].len != locallen ) {
              locallen = 0;
              // レングス不正のため次の位置に更新（2byteのためこことforのあとので次にする）
#ifdef DEBUG_COMMAND
              DebugSerial.print(".len = "); DebugSerial.println(locallen);
#endif
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
      getlen = RingGet(lCmd, HEADER_LEN );
#ifdef DEBUG_COMMAND
      RingPrint2(lCmd, getlen);//DebugCode
#endif
    }
  }

#ifdef DEBUG_COMMAND
  DebugSerial.print("Size = "); DebugSerial.println(RingSize());
#endif
  // 取得に成功しており、かつ、リングバッファがヘッダサイズ+ヘッダに書かれたデータ部のレングス+CRCサイズ以上か
  if ( fail == 0 && RingSize() >= (HEADER_LEN + locallen + CRC_SIZE) ) {
#ifdef DEBUG_SETTING
    DebugSerial.println("============");
#endif
    memset(lCmd, 0x00, RING_BUF);
    RingGet(lCmd, HEADER_LEN + locallen + CRC_SIZE );
#ifdef DEBUG_SETTING
    RingPrint2(lCmd, HEADER_LEN + locallen + CRC_SIZE);//DebugCode
#endif
    // 取得したデータからCRC計算
    unsigned short crc = crc16(0, (unsigned char*)lCmd, HEADER_LEN + locallen);
    // 通信で取得したCRC
    unsigned short settingcrc = *((unsigned short*)(&lCmd[HEADER_LEN + locallen]));
#ifdef DEBUG_COMMAND
    DebugSerial.println(crc);
    DebugSerial.println(settingcrc);
#endif
    if ( crc == settingcrc ) {
      // 電文の取得に成功した
      memcpy(lbufferSerial, lCmd, HEADER_LEN + locallen + CRC_SIZE);
#ifdef DEBUG_COMMAND
      DebugSerial.println("Success");
#endif
      // 次の位置に更新
      RingReadPosAdd((HEADER_LEN + locallen + CRC_SIZE));
      return (HEADER_LEN + locallen + CRC_SIZE);
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

