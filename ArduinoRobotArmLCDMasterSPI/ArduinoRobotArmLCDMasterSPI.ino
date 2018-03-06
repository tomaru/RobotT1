/*
  SPIマスタ
  SS   - gSSPin = { 5, 4}
  MOSI - Pin11
  MISO - Pin12
  SCK  - Pin13
*/
//#define DEBUG

#include <SPI.h>

#define SSPin 10

String TextCode[2] = {{ "Call Slave" }, {"OK?"}};
int callcount = 0;
#define BUFF_MAX 255
#define TERM_MAX 2
char bufferSerial[BUFF_MAX];

SPISettings settingA(500000, MSBFIRST, SPI_MODE2);

int gindex;
// gSSPin と gsettings の配列要素はそれぞれが対応している
int gSSPin[TERM_MAX] = {5, 4};
SPISettings gsettings[TERM_MAX] = {settingA, settingA};

void setup() {
  Serial.begin (115200);
#ifdef DEBUG
  Serial.println("Master");
#endif
  initSS(); //SSピンを出力設定

  //SPI.setBitOrder(MSBFIRST);  //最上位ビット(MSB)から送信
  //SPI.setClockDivider(SPI_CLOCK_DIV128);  //通信速度をデフォルト4->32
  //SPI.setDataMode(SPI_MODE2);   //アイドル5Vで0V→5Vの変化で送信する
  SPI.begin();  //開始

  gindex = 0;
}

void loop() {
  char ch;
  char rcv;
  int term;
  int OK;
  callcount = callcount + 1;
  //UARTから読み込み
  if (Serial.available()) {
    ch = Serial.read();
    OK = inputSerial(ch, bufferSerial, &gindex);

    if ( OK ) {
      char *sendbuffer;
      sendbuffer = bufferSerial;
      OK = DecodeArmMasterTerm(&term, &sendbuffer, gindex);
      switch ( OK ) {
        // 取得成功
        case 1 :
          SendSPI( term, sendbuffer);
          gindex = 0;
          break;
        // 取得失敗
        case -1 :
          gindex = 0;
          break;
        // 電文未完成
        default :
          break;
      }
    }
  }
}

void SendSPI( int term, char *sendbuffer)
{
  //データがあれば送信
  //String outputstr;
  // column(2桁) row(1桁)　表示文字列1 終端文字(;)
  //outputstr =  sendbuffer + String(callcount) + ';';
  //outputstr.toCharArray(sendbuffer, BUFF_MAX);
#ifdef DEBUG
  Serial.println("=== SendSPI sendBuffer ===");
  Serial.println(sendbuffer);
  Serial.println("=== SendSPI term ===");
  Serial.println(term);
#endif

  int ii = 0;
  //このSlaveをセレクトする
  //（マスタ通信は有効です）
  selectSSstart(term);
  delay(100);// 信号の安定化が必要
  for ( ii = 0; sendbuffer[ii] != '\0'; ii++) {
    //送信
    SPI.transfer(sendbuffer[ii]);
    //delay(50);
  }
  selectSSend(term);
  delay(100);// 信号の安定化が必要
}

int inputSerial(char ch, char *lbufferSerial, int *inoutindex) {
  lbufferSerial[*inoutindex] = ch;
  if ( lbufferSerial[*inoutindex] == '\r' || lbufferSerial[*inoutindex] == '\n' || lbufferSerial[*inoutindex] == '\0') {
    goto END;
  }
  (*inoutindex) = (*inoutindex) + 1;
#ifdef DEBUG
  Serial.print(lbufferSerial);
  Serial.print("   ");
  Serial.println(*inoutindex);
#endif
  //バッファ以上の場合は中断
  if (*inoutindex >= BUFF_MAX) {
#ifdef DEBUG
    Serial.println("Break");
#endif
    *inoutindex = 0;
    goto END;
  }
  if ( lbufferSerial[*inoutindex - 1] == ';' ) {
    //終端文字を足す
    lbufferSerial[*inoutindex] = '\0';
#ifdef DEBUG
    Serial.println("=== CmdDetected ===");
    Serial.println(lbufferSerial);
    Serial.println(*inoutindex);
#endif
    return 1;
  }
END:
  return 0;
}

int DecodeArmMasterTerm(int *term, char **input, int lindex)
{
#ifdef DEBUG
    Serial.println("=== DecodeArmMasterTerm ===");
    Serial.println(lindex);
    Serial.println(*input);
#endif
  if ( lindex >= 2) {
    *term = (*input)[0] - '0';
    *input = &((*input)[1]);
    if ( (*term >= 0) && (*term < TERM_MAX) ) {
      return 1;
    } else {
      return -1;
    }
  } else {
    return 0;
  }
}

// SS pinを出力設定する
void initSS() {
  int ii;
  for ( ii = 0; ii < (sizeof(gSSPin) / sizeof(gSSPin[0])); ii++) {
    pinMode(gSSPin[ii], OUTPUT); //SSピンを出力設定
    digitalWrite(gSSPin[ii], HIGH); //SSピンを出力設定を解除
  }
}

// selectSSstartとselectSSendのセットで使用する
int selectSSstart(int term) {
  SPI.beginTransaction(gsettings[term]);
  digitalWrite(gSSPin[term], LOW); //SSピンを出力設定
}

int selectSSend(int term) {
  digitalWrite(gSSPin[term], HIGH); //SSピンを出力設定を解除
  SPI.endTransaction();
}
