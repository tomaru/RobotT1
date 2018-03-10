#ifndef Ring_h
#define Ring_h

//############  RingBuffer   ############
#define RING_BUF 256 // 2のべき乗である必要がある
typedef struct st_ring
{
  byte *read_pos;
  byte *write_pos;
  byte *start_pos;
  byte *end_pos;
  int len;
  byte data[RING_BUF];
  unsigned int mask = (RING_BUF - 1);
} RingBuffer;

// リングバッファの実態
RingBuffer ring;

// 次の要素位置を特定する。こっちのほうが下より正確
#define nextpos(a)  (byte *)(ring.start_pos + ((unsigned int)(((a - ring.start_pos == 0 ? sizeof(byte) : (byte)(a - ring.start_pos))/sizeof(byte))+1 ) & (unsigned int)(ring.mask)))
// 上を端折ったやつ。
//#define nextpos(a)  (byte *)(ring.start_pos + ((unsigned int)((a - ring.start_pos) + 1) & (unsigned int)(ring.mask)))

// リングバッファの初期化関数
void RingInit()
{
  // 初期値を設定する
  // 読み込み位置
  ring.read_pos = &ring.data[0];
  // 書き込み位置
  ring.write_pos = &ring.data[0];
  // 確保した配列の最初を指す
  ring.start_pos = &ring.data[0];
  // 確保した配列の最後を指す
  ring.end_pos = &ring.data[RING_BUF-1];
  // これでマスクすると剰余を求めなくて済む
  // ただしこれで配列要素を指定できるのは、リングバッファが2^nの制限があるため
  ring.mask = RING_BUF - 1;
  // 読み込んだbyte数は0(byte)
  ring.len = 0;
}

// Debug用にリングバッファの状態を出力する
void RingPrint()
{
  int ii;
  char printbuf[255];
  for(ii=0; ii<RING_BUF ; ii++ ){
    sprintf( printbuf, "%02x ", ring.data[ii] );
    Serial.print(printbuf);
  }
  Serial.println("");
}

// 引数の配列を出力する（リングバッファ用）
void RingPrint2(byte *out, int len)
{
  int ii;
  int jj;
  int ones=0;
  char printbuf[255];
  for(ii=0; ii<RING_BUF && ii < len ; ii++ ){
    sprintf( printbuf, "%02x ", *(out+ii) );
    Serial.print(printbuf);
  }
  Serial.println("");
}

// 引数のoutにリングバッファから取り出した文字列を設定する
int RingGetCommand(byte (&out)[RING_BUF], int maxlen)
{
  unsigned int ii;
  unsigned int jj=0;
  int ones=0;
  int len=0;
  char printbuf[255];

  for(ii=0; (ii < RING_BUF) && (ii < maxlen) && ((ring.read_pos+ii) != ring.write_pos); ii++ ){
    if( ((ring.read_pos+ii) > (ring.end_pos)) &&  (ones == 0) ) {
      ones = 1;
      // 4つ{0,1,2,3}で、要素1の入力なら、iiは2になっている
      jj = ii;
    }
    if(ones == 1){
      (out[ii]) = *(ring.start_pos + (ii-jj & ring.mask)) ;
    }else{
      (out[ii]) = *(ring.read_pos+ii) ;
    }
    len++;
  }
  return len;
}

int RingSize()
{
  return ring.len;
}

// リングバッファに引数のdataを一番後ろに設定する
int RingWrite(byte data)
{
  *(ring.write_pos) = data;
  ring.write_pos = nextpos(ring.write_pos) ;// Next Posision
  ring.len++;
  return 1;
}

// リングバッファの読み込み位置をaddの分だけ加算する
int RingReadPosAdd(int add)
{
  ring.read_pos = nextpos(ring.read_pos);// Next Posision
  ring.len--;
  return 1;
}

#endif
