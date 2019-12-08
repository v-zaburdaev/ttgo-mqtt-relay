// https://github.com/m5stack/M5Stack/blob/master/src/utility/Power.cpp

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00
#define IP5306_REG_SYS_CTL1 (0x01)
#define IP5306_REG_SYS_CTL2 (0x02)
#define IP5306_REG_READ0 (0x70)
#define IP5306_REG_READ1 (0x71)
#define IP5306_REG_READ3 (0x78)
#define IP5306_REG_CHG_CTL0 (0x20)
#define IP5306_REG_CHG_CTL1 (0x21)
#define IP5306_REG_CHG_DIG  (0x24)

//- REG_CTL0
#define BOOST_ENABLE_BIT (0x20)
#define CHARGE_OUT_BIT (0x10)
#define BOOT_ON_LOAD_BIT (0x04)
#define BOOST_OUT_BIT (0x02)
#define BOOST_BUTTON_EN_BIT (0x01)

//- REG_CTL1
#define BOOST_SET_BIT (0x80)
#define WLED_SET_BIT (0x40)
#define SHORT_BOOST_BIT (0x20)
#define VIN_ENABLE_BIT (0x04)

//- REG_CTL2
#define SHUTDOWNTIME_MASK (0x0c)
#define SHUTDOWNTIME_64S (0x0c)
#define SHUTDOWNTIME_32S (0x04)
#define SHUTDOWNTIME_16S (0x08)
#define SHUTDOWNTIME_8S  (0x00)

//- REG_READ0
#define CHARGE_ENABLE_BIT (0x08)

//- REG_READ1
#define CHARGE_FULL_BIT (0x08)

//- REG_READ2
#define LIGHT_LOAD_BIT (0x20)
#define LOWPOWER_SHUTDOWN_BIT (0x01)

//- CHG
#define CURRENT_100MA  (0x01 << 0)
#define CURRENT_200MA  (0x01 << 1)
#define CURRENT_400MA  (0x01 << 2)
#define CURRENT_800MA  (0x01 << 3)
#define CURRENT_1600MA  (0x01 << 4)

#define BAT_4_2V      (0x00)
#define BAT_4_3V      (0x01)
#define BAT_4_3_5V    (0x02)
#define BAT_4_4V      (0x03)

#define CHG_CC_BIT    (0x20)

bool setPowerBoostKeepOn(int en)
{
  Wire.beginTransmission(IP5306_ADDR);
  Wire.write(IP5306_REG_SYS_CTL0);
  if (en) {
    Wire.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    Wire.write(0x35); // 0x37 is default reg value
  }
  return Wire.endTransmission() == 0;
}

bool isCharging() {
  uint8_t data;
  Wire.beginTransmission(IP5306_ADDR);
    Wire.write(IP5306_REG_READ0);
    byte err = Wire.endTransmission();
    if(err==0){
        Wire.requestFrom (IP5306_ADDR, 1);
        delay(10);
        while(Wire.available())    // slave may send less than requested
        { 
          char c = Wire.read();    // receive a byte as character
          if(c & CHARGE_ENABLE_BIT) {
            return true;
            }
        }
    }
    
  return false;
}


// full return true, else return false
bool isChargeFull() {
  uint8_t data;
  Wire.beginTransmission(IP5306_ADDR);
    Wire.write(IP5306_REG_READ1);
    byte err = Wire.endTransmission();
    if(err==0){
        Wire.requestFrom (IP5306_ADDR, 1);
        delay(10);
        while(Wire.available())    // slave may send less than requested
        { 
          char c = Wire.read();    // receive a byte as character
          if(c & CHARGE_FULL_BIT) {
            return true;
            }
        }
    }
    
  return false;
}

int8_t getBatteryLevel() {
    uint8_t data;
    Wire.beginTransmission(IP5306_ADDR);
    Wire.write(IP5306_REG_READ3);
    byte err = Wire.endTransmission();
    if(err==0){
        Wire.requestFrom (IP5306_ADDR, 1);
        delay(10);
      while(Wire.available())    // slave may send less than requested
        { 
          char c = Wire.read();    // receive a byte as character
          switch (c & 0xF0) {
            case 0x00:
            SerialMon.println("100%");         // print the character
              return 100;
            case 0x80:
            SerialMon.println("75%");         // print the character
              return 75;
            case 0xC0:
            SerialMon.println("50%");         // print the character
              return 50;
            case 0xE0:
            SerialMon.println("25%");         // print the character
              return 25;
            default:
            SerialMon.println("0");         // print the character
              return 0;
          }
        }
      } else {
        SerialMon.println("getBatteryLevel Transmission error "+(char) err);
      }

  return -1;
}
