
int callcount = 0;

void setup() {
  // put your setup code here, to run once:

  Serial.begin (115200);
}

void loop() {
  // put your main code here, to run repeatedly:

  String outputstr;
  // term(1桁) column(2桁) row(1桁)　表示文字列1 終端文字(;)
  if( callcount % 2 ) {
    outputstr =  "1000Hello! " + String(callcount++) + ';';
  } else {
    outputstr =  "0011Hello! " + String(callcount++) + ';';
  }
  Serial.println(outputstr);
  delay(200);
}
