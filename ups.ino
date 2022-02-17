#include <Wire.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#define epromsize 200     // EEPROM 字节分配
#define BQ25713_ADDR 0x6b // 芯片IIC地址BQ25713是6b，25713b是6a
#define CHG_OK_PIN 4      // D2 // BQ25713 charge_OK 输出
#define MB_LED_PIN 14     // D5 // 电脑开机状态LED,接到电脑上
#define MB_START_PIN 12   // D6 // 电脑开机引脚,接到电脑上
#define BOX_SW_PIN 13     // D7 // 机箱开机引脚,接到机箱开关 接到VCC。开机时为重设网络参数
#define BOX_LED_PIN 15    // D8 // 机箱开机状态LED,接到机箱led上 GND
//#define LED_PIN 2               // D0 //板载LED
#define SCL 0                    // D3
#define SDA 2                    // D4
#define ChargerStatus_ADDR 0x20  // R 充电状态
#define IIN_DPM_ADDR 0X24        // R 实际输入电流限制值 ICO以后要重设输入电流限制
#define ADC_ADDR 0x26            // R 0x26-0x2D  ADC测量结果
#define ManufacturerID_ADDR 0x2E // ==40H
#define DeviceID_ADDR 0x2F       // 芯片ID 88h (BQ25713) 8Ah (BQ25713B)
#define ChargerOption0_ADDR 0x00 //
#define ChargerOption1_ADDR 0x30 //
#define ChargerOption2_ADDR 0x32 //
#define ChargerOption3_ADDR 0x34 //
#define ProchotOption0_ADDR 0X36
#define ProchotOption1_ADDR 0X38
#define ADCENS_ADDR 0x3A           // ADC功能设置，启用还是关闭测量
#define ChargeCurrent_ADDR 0x02    // 0-8128mA /64mA offset 0
#define MaxChargeVoltage_ADDR 0x04 // 1024-19200mV /8mV offset 0
#define OTGVoltage_ADDR 0x06       // 4280-20800mV /8mV offset  0
#define OTGCurrent_ADDR 0x08       // 50-6400mA /50mA offset 50mA，电流限制不是2进制组合需要转换成2进制，8位数据
#define InputVoltage_ADDR 0x0A     // 3200-19584mV /64mV offset 3200，低于这个值就引发vidpm
#define MinSysVolt_ADDR 0x0C       // 1024-16128mV /256mV offset 0
#define IIN_LIM_ADDR 0x0E          // 50-6400mA /50mA offset 50mA，电流限制不是2进制组合需要转换成2进制，8位数据

byte chargeopt01, chargeopt00, chargeopt31, chargeopt30, chargeopt32, chargeopt33, chargeopt34, chargeopt35, prohotopt36, prohotopt37, prohotopt38, prohotopt39;
String ssid = "", password = "";
char mqtt_server[50] = "";
char ups_IP[16] = "";
char mqtt_user[30] = "";
char mqtt_pwd[30] = "";
long now, j;
int i = 0;
byte getADC[8], MSB, LSB, BatStatus;
boolean ACstat, bqflag;
struct
{
  float PSYS;
  float PIN;
  int VBUS;
  int ICHG;
  int IDCHG;
  int CMPIN;
  int IIN;
  int VBAT;
  int VSYS;
} ADC;
struct BqPara
{
  int ChargeCurrent;
  int MaxChargeVoltage;
  int MinSysVolt; // 8位
  int MinInputV;  //输入Vdpm
  int IIn_Limt;   // 8位
  int IIN_DPM;    // 8位只读
  int VBatOff;    // 16位
};                // SET_PARA;
BqPara SET_PARA, READ_PARA;
WiFiClient ups;
PubSubClient client(ups);
void setup()
{
  pinMode(CHG_OK_PIN, INPUT_PULLUP);
  pinMode(MB_LED_PIN, INPUT_PULLUP);
  pinMode(MB_START_PIN, OUTPUT);
  pinMode(BOX_SW_PIN, INPUT_PULLUP);
  pinMode(BOX_LED_PIN, OUTPUT);
  digitalWrite(MB_START_PIN, 1); //控制电脑启动的引脚 上电拉高
  Serial.begin(57600);           //初始化串口配置
  Wire.begin(SCL, SDA);          // 初始化IIC 通讯 并指定引脚做通讯
  delay(100);
  ReadRomBqConf();
  if (!digitalRead(BOX_SW_PIN)) // D7 重新配置网络参数
    WriteRomNetConf(20);        //写网络参数到rom
  ReadRomNetConf(20);           //读取网络参数
  WiFi.mode(WIFI_STA);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}
void loop()
{
  if (!digitalRead(BOX_SW_PIN)) //机箱开关触发开机
  {
    delay(50);
    if (!digitalRead(BOX_SW_PIN))
    {
      digitalWrite(MB_START_PIN, LOW);
      delay(500);
      digitalWrite(MB_START_PIN, HIGH);
    }
  }
  client.loop();             // callback()调用
  if (millis() - now > 2000) //每2s执行一次
  {
    j++;
    digitalWrite(BOX_LED_PIN, digitalRead(MB_LED_PIN));//电脑开机状态转到机箱LED
    Serial.print("mb led ");
    Serial.println(digitalRead(MB_LED_PIN));
    if (WiFi.status() != WL_CONNECTED)
      reconnectwifi();
    if (!client.connected() && WiFi.status() == WL_CONNECTED)
      reconnectmqtt();
    if (client.connected())
    {
      client.publish("ups/status", "online"); // ups上线可用
      client.publish("ups/IP", ups_IP);
    }
    now = millis();
    sectohms(now / 1000);
    ChargeStatus();
    ReadSetPara();
    ParaPublish();
    ADCstatus();
    bqflag = mreadBQ25(ADC_ADDR, getADC, 8); // adc状态监控
    if (bqflag)
    {
      ADCcalc();
      ADCSerial();
      ADCpublish();
    }
    if (abs(READ_PARA.IIN_DPM - SET_PARA.IIn_Limt) > 100) //重新设置输入电流限制
      SetInLimtCurrent(SET_PARA.IIn_Limt);
    if (abs(SET_PARA.ChargeCurrent - READ_PARA.ChargeCurrent) > 100) //重新设定充电电流
      SetChargeCurrent(SET_PARA.ChargeCurrent);
    if (!(j % 60))
      nascontrol(); //大约每3分钟执行1次nascontrol函数
  }
}
boolean mreadBQ25(byte regAddress, byte *dataVal, byte arrLen)
{
  Wire.beginTransmission(BQ25713_ADDR);
  Wire.write(regAddress);
  byte ack = Wire.endTransmission();
  if (ack == 0)
  {
    Wire.requestFrom((int)BQ25713_ADDR, (int)arrLen);
    if (Wire.available() > 0)
    {
      for (uint8_t i = 0; i < arrLen; i++)
      {
        dataVal[i] = Wire.read();
      }
    }
    return true;
  }
  else
  {
    return false;
  }
}
boolean writeBQ25(byte regAddress, byte dataVal0, byte dataVal1)
{
  Wire.beginTransmission(BQ25713_ADDR);
  Wire.write(regAddress);
  Wire.write(dataVal0);
  Wire.write(dataVal1);
  byte ack = Wire.endTransmission();
  if (ack == 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}
void SetChargeCurrent(int c)
{
  setBytes(c, 0, 8128, 0, 64);
  writeBQ25(ChargeCurrent_ADDR, LSB, MSB); //充电电流
}
void SetMaxChargeVoltage(int c)
{
  setBytes(c, 1024, 19200, 0, 8);
  writeBQ25(MaxChargeVoltage_ADDR, LSB, MSB);
}
void SetMinSysVoltage(int c)
{
  setBytes(c, 1024, 16128, 0, 256);
  writeBQ25(MinSysVolt_ADDR, 0, MSB);
}
void SetInLimtCurrent(int c)
{
  setBytes(c, 50, 6400, 50, 50);
  writeBQ25(IIN_LIM_ADDR, 0, LSB / 50); //电流限制不是2进制组合需要转换成2进制，8位数据
}
void SetInVoltage(int c)
{
  setBytes(c, 3200, 19584, 3200, 64);
  writeBQ25(InputVoltage_ADDR, LSB, MSB);
}
void ADCcalc()
{
  // ADC.PSYS = getADC[0] * 0.4;//psys= Vsys(mV)/Rsys(R) * 10^3
  if (getADC[1])
    ADC.VBUS = getADC[1] * 64 + 3200;
  else
    ADC.VBUS = 0;
  ADC.IDCHG = getADC[2] * 256;
  ADC.ICHG = getADC[3] * 64;
  ADC.CMPIN = getADC[4] * 12;
  ADC.IIN = getADC[5] * 50;
  if (getADC[6])
    ADC.VBAT = getADC[6] * 64 + 2880;
  else
    ADC.VBAT = 0;
  if (getADC[7])
    ADC.VSYS = getADC[7] * 64 + 2880;
  else
    ADC.VSYS = 0;
  ADC.PSYS = (ADC.IIN * ADC.VBUS + (ADC.IDCHG - ADC.ICHG) * ADC.VBAT) / 1000000.0;
  ADC.PIN = (ADC.IIN * ADC.VBUS) / 1000000.0;
}
void ADCSerial()
{
  Serial.print("PIN_mV : ");
  Serial.println(ADC.PIN);
  Serial.print("PSYS_W : ");
  Serial.println(ADC.PSYS);
  Serial.print("VBUS_mV : ");
  Serial.println(ADC.VBUS);
  Serial.print("ICHG_mA : ");
  Serial.println(ADC.ICHG);
  Serial.print("IDHG_mA : ");
  Serial.println(ADC.IDCHG);
  Serial.print("IIN_mA ");
  Serial.println(ADC.IIN);
  Serial.print("VBAT_mV ");
  Serial.println(ADC.VBAT);
  Serial.print("VSYS_mV : ");
  Serial.println(ADC.VSYS);
}
void ADCpublish()
{
  char temp[8] = "";
  snprintf(temp, 6, "%f", ADC.PIN);
  client.publish("ups/ADC/PIN", temp);
  snprintf(temp, 6, "%f", ADC.PSYS);
  client.publish("ups/ADC/PSYS", temp);
  snprintf(temp, 6, "%d", ADC.VBUS);
  client.publish("ups/ADC/VBUS", temp);
  snprintf(temp, 6, "%d", ADC.ICHG);
  client.publish("ups/ADC/ICHG", temp);
  snprintf(temp, 6, "%d", ADC.IDCHG);
  client.publish("ups/ADC/IDCHG", temp);
  snprintf(temp, 6, "%d", ADC.IIN);
  client.publish("ups/ADC/IIN", temp);
  snprintf(temp, 6, "%d", ADC.VBAT);
  client.publish("ups/ADC/VBAT", temp);
  snprintf(temp, 6, "%d", ADC.VSYS);
  client.publish("ups/ADC/VSYS", temp);
  snprintf(temp, 6, "%f", 100 * (1 - 1.0 * (SET_PARA.MaxChargeVoltage - ADC.VBAT) / (SET_PARA.MaxChargeVoltage - SET_PARA.MinSysVolt))); //计算电池容量根据设置电池放电参数估算0-100%
  client.publish("ups/BatQ", temp);
  if (ACstat)
    client.publish("ups/AC", "Online");
  else
    client.publish("ups/AC", "Offline");
  if (ADC.IDCHG > 80)
    client.publish("ups/battery", "discharge");
  else
  {
    if (BatStatus == 2)
      client.publish("ups/battery", "fast charge");
    if (BatStatus == 1)
      client.publish("ups/battery", "pre charge");
    if (BatStatus == 0)
      client.publish("ups/battery", "normal");
  }
}
void setBytes(uint16_t value, uint16_t minVal, uint16_t maxVal, uint16_t offset, uint16_t resVal)
{
  if (value < minVal)
    value = minVal;
  if (value > maxVal)
    value = maxVal;
  value = value - offset;
  value = value / resVal;
  value = value * resVal;
  LSB = value && 0xff;
  MSB = value >> 8;
}
void ChargeStatus()
{
  byte dataVal[2];
  char temp[8] = "";
  boolean bqACK;
  bqACK = mreadBQ25(ChargerStatus_ADDR, dataVal, 2);
  if (bqACK)
  {
    snprintf(temp, 6, "%d", dataVal[0]);
    client.publish("ups/ChargeStatus0", temp);
    snprintf(temp, 6, "%d", dataVal[1]);
    client.publish("ups/ChargeStatus1", temp);
    if (dataVal[1] & 0B10000000)
    {
      Serial.println("AC OnLine");
      ACstat = true;
    }
    else
    {
      Serial.println("AC OffLine");
      ACstat = false;
    }
    if (dataVal[1] & 0B1000000)
    {
      Serial.println("ICO DONE Y");                     // ICO 以后需要重设输入电流设置
      writeBQ25(IIN_LIM_ADDR, 0x00, SET_PARA.IIn_Limt); // 输入电流设置
      delay(10);
    }
    if (dataVal[1] & 0B100000)
      Serial.println("IN_VAP Y");
    if (dataVal[1] & 0B10000)
      Serial.println("VINDPM Y");
    if (dataVal[1] & 0B1000)
      Serial.println("IINDPM Y");
    if (dataVal[1] & 0B1)
      Serial.println("IN OTG Y");
    if (dataVal[0] & 0B10000000)
      Serial.println("ACOV Y");
    if (dataVal[0] & 0B1000000)
      Serial.println("BATOC Y");
    if (dataVal[0] & 0B100000)
      Serial.println("ACOC Y");
    if (dataVal[0] & 0B10000)
    {
      Serial.println("SYSOVP Y");
      writeBQ25(ChargerStatus_ADDR, 0x00, 0x0); // ChargerStatus_ADDR
    }
    if (dataVal[0] & 0B1000)
    {
      Serial.println("SYS SHORT Y");
      writeBQ25(ChargerStatus_ADDR, 0x00, 0x0);
    }
    if (dataVal[0] & 0B100)
      Serial.println("LATCH_OFF Y");
    if (dataVal[0] & 0B10)
      Serial.println("OTG_OVP Y");
    if (dataVal[0] & 0B1)
      Serial.println("OTG_UVP Y");
    if (dataVal[1] & 0B100)
    {
      Serial.println("FAST CHARGE mode");
      BatStatus = 2;
    }
    if (dataVal[1] & 0B10)
    {
      Serial.println("PRE CHARGE mode");
      BatStatus = 1;
    }
    if (!(dataVal[1] & 0B110))
    {
      Serial.println("battery no CHARGE ");
      BatStatus = 0;
    }
  }
}
void ADCstatus()
{
  byte dataVal[2];
  boolean bqACK;
  bqACK = mreadBQ25(ADCENS_ADDR, dataVal, 2);
  if (bqACK)
  {
    if ((dataVal[1] & 0B10000000) && (dataVal[0] == 0x7f))
    {
      Serial.println("ADC_CONV : Continuous update");
    }
    else
    {
      Serial.println("ADC_CONV : One-shot update");
      writeBQ25(ADCENS_ADDR, 0x7f, 0xa0); //打开测量系统功能，并启动测量 连续转换
    }
  }
}
void reconnectwifi()
{
  i = 0;
  Serial.print("Connecting to [");
  Serial.print(ssid);
  Serial.print("]\n");
  Serial.print("use password [");
  Serial.print(password);
  Serial.print("]\n");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && i < 10)
  {
    delay(1500);
    Serial.print(".");
    i++;
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nWiFi connected\nIP address: ");
    Serial.println(WiFi.localIP());
  }
}
void reconnectmqtt()
{
  Serial.print("\nConnecting MQTT server to [");
  Serial.print(mqtt_server);
  Serial.print("] ");
  Serial.print("use:[");
  Serial.print(mqtt_user);
  Serial.print(":");
  Serial.print(mqtt_pwd);
  Serial.print("]\n");
  if (client.connect("ups-8266", mqtt_user, mqtt_pwd))
  {
    Serial.println("\nUPS MQTT connected");
    client.publish("ups/status", "online"); // ups上线可用
    sprintf(ups_IP, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
    client.publish("ups/IP", ups_IP);
    client.subscribe("ups/set/InitBQ");
    client.subscribe("ups/set/ChargeI");
    client.subscribe("ups/set/MaxChargeV");
    client.subscribe("ups/set/MinSysV");
    client.subscribe("ups/nas/Restart");
    client.subscribe("ups/set/MaxInI");
    client.subscribe("ups/set/MinInV");
    client.subscribe("ups/set/VBatOff");
  }
}
void ReadRomBqConf()
{
  unsigned int c;
  EEPROM.begin(epromsize);
  c = (EEPROM.read(2) << 8) + EEPROM.read(1); //充电电流
  Serial.print("charge current:");
  Serial.println(c);
  SET_PARA.ChargeCurrent = c;
  SetChargeCurrent(c);
  c = (EEPROM.read(4) << 8) + EEPROM.read(3); //最大充电电压
  Serial.print("max chagre V:");
  Serial.println(c);
  SetMaxChargeVoltage(c);
  c = (EEPROM.read(6) << 8) + EEPROM.read(5); //最小系统电压3s对应9v
  Serial.print("min sys V:");
  Serial.println(c);
  SetMinSysVoltage(c);
  c = (EEPROM.read(8) << 8) + EEPROM.read(7); //最小输入电压，触发VIDPM的电压
  Serial.print("input VIDPM:");
  Serial.println(c);
  SetInVoltage(c);
  c = (EEPROM.read(10) << 8) + EEPROM.read(9); //输入电流设置
  SET_PARA.IIn_Limt = c;
  Serial.print("input current:");
  Serial.println(c);
  SetInLimtCurrent(c);
  SET_PARA.VBatOff = (EEPROM.read(12) << 8) + EEPROM.read(11); // 电池关机电压
  Serial.print("bat poweroff:");
  Serial.println(SET_PARA.VBatOff);
  /* Serial.println("eeprom ");
   for (i = 0; i < 70; i++)
   {
     Serial.print("addr ");
     Serial.print(i);
     Serial.print(" : ");
     Serial.println(EEPROM.read(i));
   }*/
  EEPROM.end();
}
void WriteRomNetConf(int i) // i 为rom 开始地址
{
  Serial.print("\nstart net config info\n ");
  EEPROM.begin(epromsize);
  for (int j = 0; j < epromsize; j++) //清空eeprom
  {
    EEPROM.write(j, 255);
    delay(10);
  }
  EEPROM.end();
  delay(100);
  EEPROM.begin(epromsize);
  Serial.print("\nplease first input wifi ssid:\n ");
  while (!Serial.available())
  {
  }
  delay(100);
  while (Serial.available() > 0)
  {
    EEPROM.write(i, Serial.read());
    i++;
  }
  EEPROM.write(i, '\0');
  i++;
  Serial.print("\nplease input wifi password,if no password,input 'NULL':\n ");
  while (!Serial.available())
  {
  }
  delay(100);
  while (Serial.available() > 0)
  {
    EEPROM.write(i, Serial.read());
    i++;
  }
  EEPROM.write(i, '\0');
  i++;
  Serial.print("\nplease input mqtt server:\n ");
  while (!Serial.available())
  {
  }
  delay(100);
  while (Serial.available() > 0)
  {
    EEPROM.write(i, Serial.read());
    i++;
  }
  EEPROM.write(i, '\0');
  i++;
  Serial.print("\nplease input mqtt user:\n ");
  while (!Serial.available())
  {
  }
  delay(100);
  while (Serial.available() > 0)
  {
    EEPROM.write(i, Serial.read());
    i++;
  }
  EEPROM.write(i, '\0');
  i++;
  Serial.print("\nplease input mqtt password:\n ");
  while (!Serial.available())
  {
  }
  delay(100);
  while (Serial.available() > 0)
  {
    EEPROM.write(i, Serial.read());
    i++;
  }
  EEPROM.write(i, '\0');
  i++;
  EEPROM.end();
}
void ReadRomNetConf(int i)
{
  int j;
  char ch;
  EEPROM.begin(epromsize);
  do
  {
    ch = EEPROM.read(i);
    ssid = ssid + ch;
    i++;
  } while (ch != '\0');
  do
  {
    ch = EEPROM.read(i);
    password = password + ch;
    i++;
  } while (ch != '\0');
  if (password == "NULL" || password == "null")
    password = "";
  j = 0;
  do
  {
    ch = EEPROM.read(i);
    mqtt_server[j] = ch;
    i++;
    j++;
  } while (ch != '\0');
  j = 0;
  do
  {
    ch = EEPROM.read(i);
    mqtt_user[j] = ch;
    i++;
    j++;
  } while (ch != '\0');
  j = 0;
  do
  {
    ch = EEPROM.read(i);
    mqtt_pwd[j] = ch;
    i++;
    j++;
  } while (ch != '\0');
  EEPROM.end();
  Serial.print("\nyou input ssid:[");
  Serial.print(ssid);
  Serial.print("]\n");
  Serial.print("you input pwd:[");
  Serial.print(password);
  Serial.print("]\n");
  Serial.print("you input mqtt server:[");
  Serial.print(mqtt_server);
  Serial.print("]\n");
  Serial.print("you input mqtt user:[");
  Serial.print(mqtt_user);
  Serial.print("]\n");
  Serial.print("you input mqtt password:[");
  Serial.print(mqtt_pwd);
  Serial.print("]\n");
  Serial.println("output finish! ");
}
void nascontrol()
{
  if (digitalRead(MB_LED_PIN)) //如果开机
  {
    delay(200);
    if (digitalRead(MB_LED_PIN)) //延时防抖 再判断是否开机
    {
      if ((!digitalRead(CHG_OK_PIN)) || (!ACstat)) //如果电源断电或有错误
      {
        if (ADC.VBAT < SET_PARA.VBatOff) //电池电压低于关机电压
        {
          digitalWrite(MB_START_PIN, LOW);
          delay(500);
          digitalWrite(MB_START_PIN, HIGH);
          client.publish("ups/nas", "powerOn");
        }
      }
    }
  }
  else
  {
    delay(200);
    if (!(digitalRead(MB_LED_PIN))) ////延时防抖 再判断是否开机
    {
      if (digitalRead(CHG_OK_PIN) && ACstat) //没有错误则开机
      {
        digitalWrite(MB_START_PIN, LOW);
        delay(500);
        digitalWrite(MB_START_PIN, HIGH);
        client.publish("ups/nas", "powerOff");
      }
    }
  }
}
void InitBQ25() //重新初始化bq25芯片配置
{
  chargeopt00 = 0x0e; // 0000 1110=0e
  chargeopt01 = 0x87; // 1000 0111=87
  chargeopt30 = 0x01; // 0000 0001=1
  chargeopt31 = 0x93; // 1001 0011=93
  chargeopt32 = 0x3f; // 0011 1111=3f
  chargeopt33 = 0x02; // 0000 0010=2
  chargeopt34 = 0x31; // 0011 0001=31
  chargeopt35 = 0x08; // 0000 1000=8
  prohotopt36 = 0x6a;
  prohotopt37 = 0x4a;
  prohotopt38 = 0;
  prohotopt39 = 0x81;
  writeBQ25(ChargerOption0_ADDR, chargeopt00, chargeopt01);
  delay(50);
  writeBQ25(ChargerOption1_ADDR, chargeopt30, chargeopt31);
  delay(50);
  writeBQ25(ChargerOption2_ADDR, chargeopt32, chargeopt33);
  delay(50);
  writeBQ25(ChargerOption3_ADDR, chargeopt34, chargeopt35);
  delay(50);
  writeBQ25(ProchotOption0_ADDR, prohotopt36, prohotopt37);
  delay(50);
  writeBQ25(ProchotOption1_ADDR, prohotopt38, prohotopt39);
  delay(50);
  /*SetInLimtCurrent(3500); //输入电流设置 默认3.2A
  delay(50);
  SetMinSysVoltage(11000); //最小系统电压3s对应9v
  delay(100);
  SetMaxChargeVoltage(12600); //最大充电电压
  delay(50);
  SetChargeCurrent(1500); //充电电流
  delay(50);
  SetInVoltage(11000); //最小输入电压，触发VIDPM的电压值
  delay(50);*/
  writeBQ25(ADCENS_ADDR, 0x7f, 0xa0); //打开ADC，并启动连续转换
  delay(50);
  writeBQ25(0x06, 0, 0); // OTG电压为0
  delay(50);
  writeBQ25(0x08, 0, 0); // OTG电流为0，不使用OTG功能
  // SET_PARA.VBatOff = 12600;
}
void callback(char *intopic, byte *payload, unsigned int length)
{
  Serial.print("Message arriced [");
  Serial.print(intopic);
  Serial.print("]");
  for (int i = 0; i < length; i++)
    Serial.print((char)payload[i]);
  if (!strcmp(intopic, "ups/nas/Restart"))
  {
    if ((char)payload[0] == '1')
    {
      digitalWrite(MB_START_PIN, LOW);
      delay(500);
      digitalWrite(MB_START_PIN, HIGH);
    }
  }
  if (!strcmp(intopic, "ups/set/InitBQ")) //重新初始化芯片配置参数位默认值
  {
    if ((char)payload[0] == '1')
    {
      InitBQ25();
    }
  }
  if (!strcmp(intopic, "ups/set/ChargeI")) //充电电流
  {
    unsigned int c = 0;
    for (int i = 0; i < length; i++)
    {
      c += (int)(payload[i] - 48);
      if (i < length - 1)
        c *= 10;
    }
    EEPROM.begin(epromsize);
    EEPROM.write(1, c & 0xff);
    EEPROM.write(2, c >> 8);
    EEPROM.end();
    SET_PARA.ChargeCurrent = c / 64 * 64; //最小充电电流64ma
    SetChargeCurrent(c);
  }
  if (!strcmp(intopic, "ups/set/MaxChargeV")) //最大充电电压
  {
    unsigned int c = 0;
    for (int i = 0; i < length; i++)
    {
      c += (int)(payload[i] - 48);
      if (i < length - 1)
        c *= 10;
    }
    EEPROM.begin(epromsize);
    EEPROM.write(3, c & 0xff);
    EEPROM.write(4, c >> 8);
    EEPROM.end();
    SetMaxChargeVoltage(12600); //最大充电电压
  }
  if (!strcmp(intopic, "ups/set/MinSysV")) //最小系统电压
  {
    unsigned int c = 0;
    for (int i = 0; i < length; i++)
    {
      c += (int)(payload[i] - 48);
      if (i < length - 1)
        c *= 10;
    }
    SetMinSysVoltage(c);
    EEPROM.begin(epromsize);
    EEPROM.write(5, c & 0xff);
    EEPROM.write(6, c >> 8);
    EEPROM.end();
  }
  if (!strcmp(intopic, "ups/set/MinInV")) //输入电压设置
  {
    unsigned int c = 0;
    for (int i = 0; i < length; i++)
    {
      c += (int)(payload[i] - 48);
      if (i < length - 1)
        c *= 10;
    }
    SetInVoltage(c); //输入电压设置
    EEPROM.begin(epromsize);
    EEPROM.write(7, c & 0xff);
    EEPROM.write(8, c >> 8);
    EEPROM.end();
  }
  if (!strcmp(intopic, "ups/set/MaxInI")) //最大输入电流
  {
    unsigned int c = 0;
    for (int i = 0; i < length; i++)
    {
      c += (int)(payload[i] - 48);
      if (i < length - 1)
        c *= 10;
    }
    EEPROM.begin(epromsize);
    EEPROM.write(9, c & 0xff);
    EEPROM.write(10, c >> 8);
    EEPROM.end();
    SET_PARA.IIn_Limt = c;
    SetInLimtCurrent(c); //输入电流设置  0f 0111 1111 0x7F 0e 00000 0X0 最大电流6.35A
  }
  if (!strcmp(intopic, "ups/set/VBatOff")) //电池关机电压,电源不在线时
  {
    unsigned int c = 0;
    for (int i = 0; i < length; i++)
    {
      c += (int)(payload[i] - 48);
      if (i < length - 1)
        c *= 10;
    }
    SET_PARA.VBatOff = c;
    EEPROM.begin(epromsize);
    EEPROM.write(11, c & 0xff);
    EEPROM.write(12, c >> 8);
    EEPROM.end();
  }
}
void ReadSetPara()
{
  byte dataVal[2];
  dataVal[0] = 0;
  dataVal[1] = 0;
  if (mreadBQ25(ChargeCurrent_ADDR, dataVal, 2))
  {
    READ_PARA.ChargeCurrent = dataVal[1] << 8 + dataVal[0];
  }
  if (mreadBQ25(MaxChargeVoltage_ADDR, dataVal, 2))
  {
    READ_PARA.MaxChargeVoltage = dataVal[1] << 8 + dataVal[0];
  }
  if (mreadBQ25(MinSysVolt_ADDR, dataVal, 2))
  {
    READ_PARA.MinSysVolt = dataVal[1] << 8 * 256;
  }
  if (mreadBQ25(InputVoltage_ADDR, dataVal, 2))
  {
    READ_PARA.MinInputV = dataVal[1] << 8 + dataVal[0] + 3200;
  }
  if (mreadBQ25(IIN_LIM_ADDR, dataVal, 2))
  {
    READ_PARA.IIn_Limt = dataVal[1] * 50 + 50;
  }
  if (mreadBQ25(IIN_DPM_ADDR, dataVal, 2))
  { //只读,数据为实际输入电流设置
    READ_PARA.IIN_DPM = dataVal[1] * 50 + 50;
  }
}
void ParaPublish()
{
  char temp[8] = "";
  snprintf(temp, 6, "%d", READ_PARA.MinSysVolt);
  client.publish("ups/para/MinSysV", temp);
  Serial.print("setting MinSysV is ");
  Serial.println(READ_PARA.MinSysVolt);
  snprintf(temp, 6, "%d", READ_PARA.MaxChargeVoltage);
  client.publish("ups/para/MaxChargeV", temp);
  Serial.print("setting MaxChargeVoltage is ");
  Serial.println(READ_PARA.MaxChargeVoltage);
  snprintf(temp, 6, "%d", READ_PARA.ChargeCurrent);
  client.publish("ups/para/ChargeI", temp);
  Serial.print("setting ChargeCurrent is ");
  Serial.println(READ_PARA.ChargeCurrent);
  snprintf(temp, 6, "%d", READ_PARA.MinInputV);
  client.publish("ups/para/MinInV", temp);
  Serial.print("setting MinInputV is ");
  Serial.println(READ_PARA.MinInputV);
  snprintf(temp, 6, "%d", READ_PARA.IIn_Limt);
  client.publish("ups/para/MaxInI", temp);
  Serial.print("setting IIn_Limt is ");
  Serial.println(READ_PARA.IIn_Limt);
  snprintf(temp, 6, "%d", READ_PARA.IIN_DPM);
  client.publish("ups/para/VIDPM", temp);
  Serial.print("setting IIN_DPM is ");
  Serial.println(READ_PARA.IIN_DPM);
}
void sectohms(int tsec)
{
  int h, m, d, s, i, j;
  char thms[5], hms[12];
  d = tsec / 3600 / 24;
  tsec %= (3600 * 24);
  h = tsec / 3600;
  m = (tsec % 3600) / 60;
  s = tsec % 60;
  i = 0;
  if (d)
  {
    while (d)
    {
      thms[i] = '0' + d % 10;
      d /= 10;
      i++;
    }
    for (j = 0; j < i; j++)
    {
      hms[j] = thms[i - j - 1];
    }
    hms[i] = 'd';
    i++;
  }
  if (h)
  {
    if (h > 9)
    {
      hms[i] = '0' + h / 10;
      i++;
    }
    hms[i] = '0' + h % 10;
    i++;
    hms[i] = 'h';
    i++;
  }
  if (m)
  {
    if (m > 9)
    {
      hms[i] = '0' + m / 10;
      i++;
    }
    hms[i] = '0' + m % 10;
    i++;
    hms[i] = 'm';
    i++;
  }
  /* hms[i] = '0' + s / 10;//计算秒数
   i++;
   hms[i] = '0' + s % 10;
   i++;
   hms[i] = 's';
   i++;*/
  hms[i] = '\0';
  client.publish("ups/uptime", hms);
  Serial.print("uptime:");
  Serial.println(hms);
}
//finish
