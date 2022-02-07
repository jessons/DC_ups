#include <Wire.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
//#include<ESP8266mDNS.h>
#define epromsize 200     // EEPROM 字节分配
#define BQ25713_ADDR 0x6b //芯片IIC地址BQ25713是6b，25713b是6a
#define CHG_OK_PIN 4//D2     // BQ25713 charge_OK 输出
#define MB_LED_PIN 14//D5     //电脑开机状态LED,接到电脑上
#define MB_START_PIN 12//D6   //电脑开机引脚,接到电脑上
#define RESET_PIN 5//D1      //重设网络参数
#define BOX_SW_PIN 15//D8     //电脑开机引脚,接到机箱开关
#define BOX_LED_PIN 13//D7    //电脑开机状态LED,接到机箱led上
//#define RESET_BQ_PIN D7            //重设芯片
#define SCL 0//d3
#define SDA  2//d4
#define ChargerStatus_ADDR 0x20    // R
#define IIN_DPM_ADDR 0X24          // R 实际输入电流限制值 ICO以后要重设输入电流限制
#define ADC_ADDR 0x26              // R 0x26-0x2D
#define ManufacturerID_ADDR 0x2E   //==40H
#define DeviceID_ADDR 0x2F         //==I2C: 88h (BQ25713); 8Ah (BQ25713B)
#define ADCENS_ADDR 0x3A           // ADC功能设置，启用还是关闭测量
#define MinSysVolt_ADDR 0x0C       //默认不改9.216v for 3s 1024-16128mV /256mV
#define ChargerOption0_ADDR 0x00   //
#define ChargerOption1_ADDR 0x30   //
#define ChargerOption2_ADDR 0x32   //
#define ChargerOption3_ADDR 0x34   //
#define ChargeCurrent_ADDR 0x02    // 64-8128mA /64mA
#define MaxChargeVoltage_ADDR 0x04 // 范围1024-19200mV /8mV
#define IIN_LIM_ADDR 0x0E          // 01111111=0x7F 输入电流限制 范围 50-6400mA /50mA offset 50mA
#define InputVoltage_ADDR 0x0A     //输入电压设置3200-19520，低于这个值就引发vidpm
#define OTGVoltage_ADDR 0x06       // offset 1.28v 1280mV
#define OTGCurrent_ADDR 0x08
#define ProchotOption0_ADDR 0X36
#define ProchotOption1_ADDR 0X38

byte chargeopt01, chargeopt00, chargeopt31, chargeopt30, chargeopt32, chargeopt33, chargeopt34, chargeopt35;
String ssid = "", password = "";
char mqtt_server[50] = "";
char ups_IP[16] = "";
char mqtt_user[30] = "";
char mqtt_pwd[30] = "";
long now, j;
int i = 0,chargeI;
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
struct
{
  int MinSysVolt; // 8位
  int MaxChargeVoltage;
  int ChargeCurrent;
  int MinInputV; //输入Vdpm
  int IIn_Limt;  // 8位
  int IIN_DPM;   // 8位只读
  int VBatOff;   // 16位
} SET_PARA;
WiFiClient ups;
PubSubClient client(ups);
void setup()
{
  pinMode(CHG_OK_PIN, INPUT_PULLUP);
  pinMode(MB_LED_PIN, INPUT_PULLUP);
  pinMode(MB_START_PIN, OUTPUT);
  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(BOX_SW_PIN, INPUT_PULLUP);
  pinMode(BOX_LED_PIN, OUTPUT);
  digitalWrite(MB_START_PIN, 1); //控制电脑启动的引脚 上电拉高
  Serial.begin(57600);           //初始化串口配置
  Wire.begin(SCL,SDA);//D3, D4);            // 初始化IIC 通讯 并指定引脚做通讯
  delay(100);
  read_rom_bq_conf();
  if (!digitalRead(RESET_PIN)) // D1=0重新配置网络参数
    write_rom_net_conf(20);    //写网络参数到rom
  read_rom_net_conf(20);       //读取网络参数
  WiFi.mode(WIFI_STA);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}
void loop()
{
  // digitalWrite(MB_START_PIN, digitalRead(BOX_SW_PIN));
  digitalWrite(BOX_LED_PIN, digitalRead(MB_LED_PIN));
  client.loop();             // callback()调用
  if (millis() - now > 2000) //每2s执行一次
  {
    j++;
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
    read_set_para();
    parapublish();
    ADCstatus();
    if(chargeI>100)
    SetChargeCurrent(SET_PARA.ChargeCurrent);
    // delay(100);
    bqflag = mreadBQ25(ADC_ADDR, getADC, 8); // adc状态监控
    if (bqflag)
    {
      ADCcalc();
      ADCSerial();
      ADCpublish();
    }
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
void ADCcalc()
{
  // ADC.PSYS = getADC[0] * 0.4;
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
  LSB = (byte)(value);
  MSB = (byte)(value >> 8);
}
void ChargeStatus()
{
  byte dataVal[2];
  boolean bqACK;
  bqACK = mreadBQ25(ChargerStatus_ADDR, dataVal, 2);
  if (bqACK)
  {
    if (dataVal[1] & 0B10000000)
    {
      Serial.println("AC STATE OnLine");
      ACstat = true;
    }
    else
    {
      Serial.println("AC STATE OffLine");
      ACstat = false;
    }
    if (dataVal[1] & 0B1000000)
    {
      Serial.println("ICO DONE Y");                     // ICO 以后需要重设输入电流设置
      writeBQ25(IIN_LIM_ADDR, 0x00, SET_PARA.IIn_Limt); //输入电流设置  0f 0111 1111 0x7F 0e 00000 0X0 最大电流6.4A
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
    if (dataVal[1] & 0B10000000)
    {
      Serial.println("ADC_CONV : Continuous update");
    }
    else
    {
      Serial.println("ADC_CONV : One-shot update");
      writeBQ25(0x3a, 0x7f, 0xa0); //打开测量系统功能，并启动测量 连续转换
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
void read_rom_bq_conf()
{
  EEPROM.begin(epromsize);
  SET_PARA.VBatOff = EEPROM.read(0) + EEPROM.read(1) * 255; // S
  Serial.print("bat poweroff:");
  Serial.println(SET_PARA.VBatOff);
  LSB = EEPROM.read(2);
  MSB = EEPROM.read(3);
  Serial.print("max chagre V:");
  Serial.println(LSB + MSB * 256);
  writeBQ25(MaxChargeVoltage_ADDR, LSB, MSB); //最大充电电压
  LSB = EEPROM.read(4);
  MSB = EEPROM.read(5);
  Serial.print("charge I:");
  Serial.println(LSB + MSB * 256);
  writeBQ25(ChargeCurrent_ADDR, LSB, MSB); //充电电流
  LSB = EEPROM.read(6);
  MSB = EEPROM.read(7);
  Serial.print("input VIDPM:");
  Serial.println(LSB + MSB * 256);
  writeBQ25(InputVoltage_ADDR, LSB, MSB); //最小输入电压，触发VIDPM的电压值16=4.096v
  MSB = EEPROM.read(8);
  SET_PARA.IIn_Limt = MSB;
  Serial.print("input I:");
  Serial.println(MSB);
  writeBQ25(IIN_LIM_ADDR, 0, MSB); //输入电流设置 默认3.2A
  MSB = EEPROM.read(9);
  Serial.print("min sys V:");
  Serial.println(MSB);
  writeBQ25(MinSysVolt_ADDR, 0, MSB); //最小系统电压3s对应9v
  EEPROM.end();
}
void write_rom_net_conf(int i) // i 为rom 开始地址
{
  Serial.print("\nplease first input wifi ssid:\n ");
  while (!Serial.available())
  {
  }
  delay(100);
  EEPROM.begin(epromsize);
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
  Serial.print("\nplease first input mqtt server:\n ");
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
  Serial.print("\nplease first input mqtt user:\n ");
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
  Serial.print("\nplease first input mqtt password:\n ");
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
void read_rom_net_conf(int i)
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
  if (password == "NULL")
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
      if ((!digitalRead(CHG_OK_PIN)) || (!ACstat)) //如果电源断电
      {                                            //关机条件
        if (ADC.VBAT < SET_PARA.VBatOff)
        { //电池电压低于10v
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
      if (digitalRead(CHG_OK_PIN) && ACstat) //开机
      {
        digitalWrite(MB_START_PIN, LOW);
        delay(500);
        digitalWrite(MB_START_PIN, HIGH);
        client.publish("ups/nas", "powerOff");
      }
    }
  }
}
void SetChargeCurrent(int dataval)
{
  setBytes(dataval, 0, 8128, 0, 64);
  writeBQ25(ChargeCurrent_ADDR, LSB, MSB); //充电电流
}
void InitBQ25()
{
  chargeopt00 = 0x0e; // 0000 1110=0e
  chargeopt01 = 0x87; // 1000 0111=87
  chargeopt30 = 0x01; // 0000 0001=1
  chargeopt31 = 0x93; // 1001 0011=93
  chargeopt32 = 0x3f; // 0011 1111=3f
  chargeopt33 = 0x02; // 0000 0010=2
  chargeopt34 = 0x11; // 0001 0001=11
  chargeopt35 = 0x08; // 0000 1000=8
  writeBQ25(ChargerOption0_ADDR, chargeopt00, chargeopt01);
  delay(50);
  writeBQ25(ChargerOption1_ADDR, chargeopt30, chargeopt31);
  delay(50);
  writeBQ25(ChargerOption2_ADDR, chargeopt32, chargeopt33);
  delay(50);
  writeBQ25(ChargerOption3_ADDR, chargeopt34, chargeopt35); //设置充电选项3 35 0000 1000 0X08 34 0001 0000 0X16  35[4] 1开启OTG,35[3] 1 ICO开启 34[6] 1 vap 开启
  delay(50);
  writeBQ25(IIN_LIM_ADDR, 0, 0x41); //输入电流设置 默认3.2A
  delay(50);
  setBytes(10500, 1024, 16128, 0, 256);
  writeBQ25(MinSysVolt_ADDR, LSB, MSB); //最小系统电压3s对应9v
  delay(100);
  setBytes(12600, 1024, 19200, 0, 8);
  writeBQ25(MaxChargeVoltage_ADDR, LSB, MSB); //最大充电电压
  delay(50);
  setBytes(1500, 64, 8128, 0, 64);
  writeBQ25(ChargeCurrent_ADDR, LSB, MSB); //充电电流
  delay(50);
  writeBQ25(InputVoltage_ADDR, 0, 16); //最小输入电压，触发VIDPM的电压值16=4.096v
  delay(50);
  writeBQ25(ADCENS_ADDR, 0x7f, 0xa0); //打开ADC，并启动连续转换
  delay(50);
  writeBQ25(0x06, 0, 0); // OTG电压为0
  delay(50);
  writeBQ25(0x08, 0, 0); // OTG电流为0，不使用OTG功能
  SET_PARA.VBatOff = 12600;
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
  if (!strcmp(intopic, "ups/set/InitBQ"))
  {
    if ((char)payload[0] == '1')
    {
      InitBQ25();
    }
  }
  if (!strcmp(intopic, "ups/set/ChargeI"))
  { //判断是否是订阅的主题充电电流
    int c = 0;
    for (int i = 0; i < length; i++)
    {
      c += (int)(payload[i] - 48);
      if (i < length - 1)
        c *= 10;
    }
    EEPROM.begin(epromsize);
    EEPROM.write(4, c % 255);
    EEPROM.write(5, c / 255);
    EEPROM.end();
    chargeI=c;
    SetChargeCurrent(c);
  }
  if (!strcmp(intopic, "ups/set/MaxChargeV")) //最大充电电压
  {                                           //判断是否是订阅的主题最大充电电压
    int c = 0;
    for (int i = 0; i < length; i++)
    {
      c += (int)(payload[i] - 48);
      if (i < length - 1)
        c *= 10;
    }
    EEPROM.begin(epromsize);
    EEPROM.write(2, c % 255);
    EEPROM.write(3, c / 255);
    EEPROM.end();
    setBytes(c, 1024, 19200, 0, 8);
    writeBQ25(MaxChargeVoltage_ADDR, LSB, MSB); //最大充电电压
  }
  if (!strcmp(intopic, "ups/set/MinSysV")) //最小系统电压3s对应9v
  {
    int c = 0;
    for (int i = 0; i < length; i++)
    {
      c += (int)(payload[i] - 48);
      if (i < length - 1)
        c *= 10;
    }
    c /= 256;
    writeBQ25(MinSysVolt_ADDR, 0, c); //最小系统电压3s对应9v
    EEPROM.begin(epromsize);
    EEPROM.write(9, c);
    EEPROM.end();
  }
  if (!strcmp(intopic, "ups/set/MaxInI"))
  { //判断是否是订阅的主题最大输入电流
    int c = 0;
    for (int i = 0; i < length; i++)
    {
      c += (int)(payload[i] - 48);
      if (i < length - 1)
        c *= 10;
    }
    c /= 50;
    EEPROM.begin(epromsize);
    EEPROM.write(8, c);
    EEPROM.end();
    writeBQ25(IIN_LIM_ADDR, 0, c); //输入电流设置  0f 0111 1111 0x7F 0e 00000 0X0 最大电流6.35A
  }
  if (!strcmp(intopic, "ups/set/MinInV")) //输入电压设置offset 3200mV
  {
    int c = 0;
    for (int i = 0; i < length; i++)
    {
      c += (int)(payload[i] - 48);
      if (i < length - 1)
        c *= 10;
    }
    setBytes(c, 64, 16320, 3200, 64);
    writeBQ25(InputVoltage_ADDR, LSB, MSB); //输入电压设置
    EEPROM.begin(epromsize);
    EEPROM.write(6, LSB);
    EEPROM.write(7, MSB);
    EEPROM.end();
  }
  if (!strcmp(intopic, "ups/set/VBatOff")) //关机电池电压、ac不在线时
  {
    int c = 0;
    for (int i = 0; i < length; i++)
    {
      c += (int)(payload[i] - 48);
      if (i < length - 1)
        c *= 10;
    }
    SET_PARA.VBatOff = c;
    EEPROM.begin(epromsize);
    EEPROM.write(0, c % 255);
    EEPROM.write(1, c / 255);
    EEPROM.end();
  }
}
void read_set_para()
{
  byte dataVal[2];
  mreadBQ25(MinSysVolt_ADDR, dataVal, 2);
  SET_PARA.MinSysVolt = dataVal[1] * 256;
  mreadBQ25(MaxChargeVoltage_ADDR, dataVal, 2);
  SET_PARA.MaxChargeVoltage = dataVal[1] * 256 + (dataVal[0] >> 3) * 8;
  mreadBQ25(ChargeCurrent_ADDR, dataVal, 2);
  SET_PARA.ChargeCurrent = dataVal[1] * 256 + (dataVal[0] >> 6) * 64;
  mreadBQ25(InputVoltage_ADDR, dataVal, 2);
  SET_PARA.MinInputV = dataVal[1] * 256 + (dataVal[0] >> 6) * 64 + 3200;
  mreadBQ25(IIN_LIM_ADDR, dataVal, 2);
  SET_PARA.IIn_Limt = dataVal[1] * 50;
  mreadBQ25(IIN_DPM_ADDR, dataVal, 2); //只读,数据为实际输入电流设置
  SET_PARA.IIN_DPM = dataVal[1] * 50 + 50;
}
void parapublish()
{
  char temp[8] = "";
  snprintf(temp, 6, "%d", SET_PARA.MinSysVolt);
  client.publish("ups/para/MinSysV", temp);
  Serial.print("setting MinSysV is ");
  Serial.println(SET_PARA.MinSysVolt);

  snprintf(temp, 6, "%d", SET_PARA.MaxChargeVoltage);
  client.publish("ups/para/MaxChargeV", temp);
  Serial.print("setting MaxChargeVoltage is ");
  Serial.println(SET_PARA.MaxChargeVoltage);

  snprintf(temp, 6, "%d", SET_PARA.ChargeCurrent);
  client.publish("ups/para/ChargeI", temp);
  Serial.print("setting ChargeCurrent is ");
  Serial.println(SET_PARA.ChargeCurrent);

  snprintf(temp, 6, "%d", SET_PARA.MinInputV);
  client.publish("ups/para/MinInV", temp);
  Serial.print("setting MinInputV is ");
  Serial.println(SET_PARA.MinInputV);

  snprintf(temp, 6, "%d", SET_PARA.IIn_Limt);
  client.publish("ups/para/MaxInI", temp);
  Serial.print("setting IIn_Limt is ");
  Serial.println(SET_PARA.IIn_Limt);

  snprintf(temp, 6, "%d", SET_PARA.IIN_DPM);
  client.publish("ups/para/VIDPM", temp);
  Serial.print("setting IIN_DPM is ");
  Serial.println(SET_PARA.IIN_DPM);
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