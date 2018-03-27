#ifndef EEPROMConfig_h
#define EEPROMConfig_h

#include <SoftwareSerial.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"
extern "C" {
#include "CalcCRC.h"
}

#define CONFIG_NUM 3
#define USE 1
#define NO_USE 0
#define VER 5

typedef struct config_t
{
  int ver;
  unsigned short x;
  unsigned short y;
  unsigned short z;
  int delta_x;
  int delta_y;
  int delta_z;
  unsigned short  speed_x;
  unsigned short  speed_y;
  unsigned short  speed_z;
  int ltime;
  int mode;
  unsigned short crc;
} configuration;

configuration conf_def = {VER, 500, 500, 500, 110, 110, 110, 50, 50, 50, 120, 1, 0x0000};

int check_eeprom_crc(configuration conf)
{
  unsigned short crc = crc16(0, (unsigned char*)&conf, sizeof(configuration) - 2);
  if ( crc == conf.crc ) {
    return 1;
  } else {
    return -1;
  }

}

int write_config(configuration &conf)
{
  int ii;
  int eeprom_address;
  unsigned short calc_crc = crc16(0, (unsigned char*)&conf, sizeof(configuration) - 2);
  conf.crc = calc_crc;
#ifdef DEBUG_CONFIG
  Serial.println("==============");
  Serial.print("conf.crc  ");
  Serial.println(conf.crc);
#endif
  for (ii = 0; ii < CONFIG_NUM; ii++) {
    // eeprom_addressは0, (length/CONFIG_NUM)*1, (N/CONFIG_NUM)*2, (N/CONFIG_NUM)*3, ...
    eeprom_address = (EEPROM.length() / CONFIG_NUM) * ii;
#ifdef DEBUG_CONFIG
    Serial.println("==============");
    Serial.print("eeprom_address  ");
    Serial.println(eeprom_address);
#endif
    EEPROM_writeAnything(eeprom_address, conf);

  }
  return 1;
}

configuration read_config()
{
  int ii;
  int eeprom_address;
  unsigned short temp_crc;
  configuration temp = conf_def;
  unsigned short conf_def_crc = crc16(0, (unsigned char*)&temp, sizeof(configuration) - 2);
  int eeprom_valid[CONFIG_NUM];
  int OK;
  int use_conf_def_flg = USE;
  configuration conf;
  configuration conf_eeprom[CONFIG_NUM];

  // 初期値のCRC計算
  conf_def.crc = conf_def_crc;

  // EEPROMからの読み出しとチェック
  for (ii = 0; ii < CONFIG_NUM; ii++) {
    // eeprom_addressは0, (length/CONFIG_NUM)*1, (N/CONFIG_NUM)*2, (N/CONFIG_NUM)*3, ...
    eeprom_address = (EEPROM.length() / CONFIG_NUM) * ii;
#ifdef DEBUG_CONFIG
    Serial.println("==============");
    Serial.print("eeprom_address  ");
    Serial.println(eeprom_address);
#endif
    EEPROM_readAnything(eeprom_address, conf_eeprom[ii]);
#ifdef DEBUG_CONFIG
    Serial.println("==============");
    Serial.println(conf_eeprom[ii].delta_x);
    Serial.println(conf_eeprom[ii].delta_y);
    Serial.println(conf_eeprom[ii].delta_z);
    Serial.println(conf_eeprom[ii].crc);
#endif

    // CRC計算
    OK = check_eeprom_crc(conf_eeprom[ii]);
#ifdef DEBUG_CONFIG
    Serial.print("OK  ");
    Serial.println(OK);
    Serial.print("conf_eeprom[ii].ver  ");
    Serial.println(conf_eeprom[ii].ver);
#endif
    if ( OK == 1 && conf_eeprom[ii].ver == VER) {
      // Valid
      conf = conf_eeprom[ii];
      eeprom_valid[ii] = 1;
      use_conf_def_flg = NO_USE;
    } else {
      //Invalid
      eeprom_valid[ii] = -1;
    }
  }
  // EEPROMのデータに正しいものがあればROMに読み込む
  // EEPROMのデータがすべて不正ならば引数のデフォルト設定とする
#ifdef DEBUG_CONFIG
  Serial.println("====1 conf_def=======");
  Serial.println(conf_def.delta_x);
  Serial.println(conf_def.delta_y);
  Serial.println(conf_def.delta_z);
  Serial.println(conf_def.crc);
  Serial.println("====0 conf=======");
  Serial.println(conf.delta_x);
  Serial.println(conf.delta_y);
  Serial.println(conf.delta_z);
  Serial.println(conf.crc);
  Serial.println("====== use_conf_def_flg ======");
  Serial.println(use_conf_def_flg);
#endif
  for (ii = 0; ii < CONFIG_NUM; ii++) {
    eeprom_address = (EEPROM.length() / CONFIG_NUM) * ii;
#ifdef DEBUG_CONFIG
    Serial.print("eeprom_address  ");
    Serial.println(eeprom_address);
#endif
    if ( eeprom_valid[ii] == -1) {
#ifdef DEBUG_CONFIG
      Serial.print("use_conf_def_flg  ");
      Serial.println(use_conf_def_flg);
#endif
      if ( use_conf_def_flg == USE ) {
        EEPROM_writeAnything(eeprom_address, conf_def);
        conf = conf_def;
      } else {
        EEPROM_writeAnything(eeprom_address, conf);
      }
    }
  }
  return conf;
}
#endif
