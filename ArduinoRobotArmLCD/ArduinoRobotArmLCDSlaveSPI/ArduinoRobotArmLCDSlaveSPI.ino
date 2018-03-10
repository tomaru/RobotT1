#include <LiquidCrystal.h>

LiquidCrystal lcd = LiquidCrystal(8, 7, 6, 5, 4, 3, 2);

//#define DEBUG

#ifdef DEBUG    //Macros are usually in all capital letters.
#define DebugSerialPrint(...)    Serial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
#define DebugSerialPrintln(...)  Serial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#else
#define DebugSerialPrint(...)     //now defines a blank line
#define DebugSerialPrintln(...)   //now defines a blank line
#endif

#define MAX_BUFF 255
char bufferSerial[MAX_BUFF];
int index = 0;

#include <SPI.h>
/*
   SPIスレーブ
   SS   - Pin10
   MOSI - Pin11
   MISO - Pin12
   SCK  - Pin13
*/
SPISettings settings(500000, MSBFIRST, SPI_MODE2);

void DecodeMasterDataCursor(char *input, int size) {
  if (size >= 2) {
    int digit10 = input[0] -  '0' ;
    int digit01 = input[1] - '0' ;
    int row = input[2] - '0' ;
#ifdef DEBUG
    Serial.println(digit10);
    Serial.println(digit01);
    Serial.println(row);
#endif
    if ( digit10 == 1) {
      lcd.setCursor(10 + digit01, row);
    } else {
      lcd.setCursor(digit01, row);
    }
  }
}

void setup() {
#ifdef DEBUG
  Serial.begin (230400);
#endif
  DebugSerialPrintln("Slav");
  // 2. SPIに使用されるピンのpinModeなどを設定する
  pinMode(MISO, OUTPUT); //MISOを出力
  //SPI.setBitOrder(MSBFIRST);  //最上位ビット(MSB)から送信
  //SPI.setClockDivider(SPI_CLOCK_DIV128);  //通信速度をデフォルト4->32
  //SPI.setDataMode(SPI_MODE2);   //アイドル5Vで0V→5Vの変化で送信する
  SPI.beginTransaction(settings);
  SPCR |= _BV(SPE);

  SPI.attachInterrupt();
  lcd.begin(16, 2);                                   // 液晶の 行数、列数 を設定
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Hello.");
  lcd.setCursor(0, 1);
  lcd.print("This is Slave.");
}

// SPI割り込み処理
ISR(SPI_STC_vect)
{
  bufferSerial[index] = SPDR;
  if (bufferSerial[index] == '\r' || bufferSerial[index] == '\n' || bufferSerial[index] == '\0') {
    goto END;
  }
  //DebugSerialPrint(String(index));
  //DebugSerialPrintln( bufferSerial[index]);
  index++;
  //バッファ以上の場合は中断
  if (index >= MAX_BUFF) {
    DebugSerialPrint("2break");
    index = 0;
    goto END;
  }
  if ( bufferSerial[index - 1] == ';' ) {
    DebugSerialPrintln("Enter");
    //終端文字を足す
    bufferSerial[index - 1] = '\0';
#ifdef DEBUG
    Serial.println(bufferSerial);
    Serial.println(index);
#endif
    lcd.clear();
    DecodeMasterDataCursor(bufferSerial, index);
    // 表示位置用の文字列を除く
    char *sendbuffer = &(bufferSerial[3]);
    lcd.print(sendbuffer);
    index = 0;
  }
END:
  ;
}

void loop() {
}
