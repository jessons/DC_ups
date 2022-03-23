#line 1 "d:\\code\\DC_ups\\README.md"
# DC_ups
直流ups控制器  
TI bq25713作为驱动板核心控制器，使用esp8266单片机作为控制器，单片机通过I2C通讯与bq25713通讯，对驱动板控制与监控。同时控制电池充放电。  
esp8266单片机通过MQTT协议与上位机通讯，监控电压电流，并可以设置控制参数，通过GPIO连接电脑开机引脚控制电脑开关机。  
性能参数  
输入最大21v 6.4A，输出维持在不低于电池电压以上。电池放电测试中超过10A，理论放电32A输出，电池充电电流可达8A，可以输入电源和电池同时放电供负载使用。超过3A建议将芯片mosfet加散热片。支持锂电池，蓄电池等各类电池。  
接线说明(以nodemcu为例)  
 D2   //BQ25713 charge_OK 输出  
 D3   // SCL  
 D4   // SDA  
 D5   //电脑开机状态LED,接到电脑上led引脚，用来判断电脑是否开机  
 D6   //电脑开机引脚,接到电脑主板上的开机引脚  
 D7   //机箱开机开关,开机时按下重设网络参数，引脚接到GND触发  
 D8   //机箱状态LED,接到GND   
MQTT 通讯  
订阅主题（接收配置参数）  
    client.subscribe("ups/set/InitBQ");//重新初始化bq25芯片参数发送1重新初始化  
    client.subscribe("ups/set/ChargeI");//充电电流  
    client.subscribe("ups/set/MaxChargeV");//最大充电电源  
    client.subscribe("ups/set/MinSysV");//最小系统电压  
    client.subscribe("ups/nas/Restart");//重启电脑 发送1 触发开机按钮，再发送1 开机  
    client.subscribe("ups/set/MaxInI");//最大输入电流  
    client.subscribe("ups/set/MinInV");//最小输入电压，触发vidpm的电压  
    client.subscribe("ups/set/VBatOff");//关机电池电压  
发表主题（发送状态参数）  
    client.publish("ups/status", "online"); // ups上线可用  
    client.publish("ups/IP", ups_IP);  
    client.publish("ups/ADC/PIN", temp);  
    client.publish("ups/ADC/PSYS", temp);  
    client.publish("ups/ADC/VBUS", temp);  
    client.publish("ups/ADC/ICHG", temp);  
    client.publish("ups/ADC/IDCHG", temp);  
    client.publish("ups/ADC/IIN", temp);  
    client.publish("ups/ADC/VBAT", temp);  
    client.publish("ups/ADC/VSYS", temp);  
    client.publish("ups/BatQ", temp);电池容量根据设置电池放电参数估算0-100%  
    client.publish("ups/AC", "Online");//电源在线  
    client.publish("ups/AC", "Offline");//电源不在线  
    client.publish("ups/battery", "discharge");  
    client.publish("ups/battery", "fast charge");  
    client.publish("ups/battery", "pre charge");  
    client.publish("ups/battery", "normal");    
    client.publish("ups/ChargeStatus0", temp);   
    client.publish("ups/ChargeStatus1", temp);  
    client.publish("ups/nas", "powerOn");  
    client.publish("ups/nas", "powerOff");  
    client.publish("ups/para/MinSysV", temp);  
    client.publish("ups/para/MaxChargeV", temp);  
    client.publish("ups/para/ChargeI", temp);  
    client.publish("ups/para/MinInV", temp);  
    client.publish("ups/para/MaxInI", temp);  
    client.publish("ups/para/VIDPM", temp);  
    client.publish("ups/uptime", hms);  
