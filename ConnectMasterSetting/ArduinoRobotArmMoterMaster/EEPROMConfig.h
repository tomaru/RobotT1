#ifndef EEPROMConfig_h
#define EEPROMConfig_h

#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"
extern "C" {
#include "CalcCRC.h"
}

#define CONFIG_NUM 5
#define USE 1
#define NO_USE 0

typedef struct config_t
{
  int delta_x;
  int delta_y;
  int delta_z;
  unsigned short crc;
} configuration;

configuration conf_def = {1, 1, 1, 0x0000};

int check_eeprom_crc(configuration conf)
{
  unsigned short crc = crc16(0, (unsigned char*)&conf, sizeof(configuration) - 2);
  if ( crc == conf.crc ) {
    return 1;
  } else {
    return -1;
  }

}

configuration read_config(configuration conf_def)
{
  int ii;
  int eeprom_address;
  unsigned short temp_crc;
  configuration temp = conf_def;
  unsigned short conf_def_crc = crc16(0, (unsigned char*)&temp, sizeof(configuration) - 2);
  int eeprom_valid[CONFIG_NUM];
  int OK;
  int use_conf_def_flg = USE;
  configuration conf_ram;
  configuration conf_eeprom[CONFIG_NUM];

  conf_def.crc = conf_def_crc;
  // EEPROMからの読み出しとチェック
  for (ii = 0; ii < CONFIG_NUM; ii++) {
    // eeprom_addressは0, (length/CONFIG_NUM)*1, (N/CONFIG_NUM)*2, (N/CONFIG_NUM)*3, ...
    eeprom_address = (EEPROM.length() / CONFIG_NUM) * ii;
#ifdef DEBUG
    Serial.println("==============");
    Serial.print("eeprom_address  ");
    Serial.println(eeprom_address);
#endif
    // 書き込み先は  ..., 1/4,1/3,1/2
    EEPROM_readAnything(eeprom_address, conf_eeprom[ii]);
#ifdef DEBUG
    Serial.println("==============");
    Serial.println(conf_eeprom[ii].delta_x);
    Serial.println(conf_eeprom[ii].delta_y);
    Serial.println(conf_eeprom[ii].delta_z);
    Serial.println(conf_eeprom[ii].crc);
#endif

    // CRC計算
    OK = check_eeprom_crc(conf_eeprom[ii]);
#ifdef DEBUG
    Serial.print("OK  ");
    Serial.println(OK);
#endif
    if ( OK == 1) {
      // Valid
      conf_ram = conf_eeprom[ii];
      eeprom_valid[ii] = 1;
      use_conf_def_flg = NO_USE;
    } else {
      //Invalid
      eeprom_valid[ii] = -1;
    }
  }
  // EEPROMのデータに正しいものがあればROMに読み込む
  // EEPROMのデータがすべて不正ならば引数のデフォルト設定とする
#ifdef DEBUG
  Serial.println("==============");
#endif
  for (ii = 0; ii < CONFIG_NUM; ii++) {
    eeprom_address =(EEPROM.length() / CONFIG_NUM) * ii;
#ifdef DEBUG
    Serial.print("eeprom_address  ");
    Serial.println(eeprom_address);
#endif
    if ( eeprom_valid[ii] == -1) {
#ifdef DEBUG
      Serial.print("use_conf_def_flg  ");
      Serial.println(use_conf_def_flg);
#endif
      if ( use_conf_def_flg == USE ) {
        EEPROM_writeAnything(eeprom_address, conf_def);
        conf_ram = conf_def;
      } else {
        EEPROM_writeAnything(eeprom_address, conf_ram);
      }
    }
  }
  return conf_ram;
}
#endif