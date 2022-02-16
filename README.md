# DC_ups
直流ups控制器  
TI bq25713作为驱动板核心控制器，使用esp8266单片机作为控制器，单片机通过I2C通讯与bq25713通讯，对驱动板控制与监控。同时控制电池充放电。  
esp8266单片机通过MQTT协议与上位机通讯，监控电压电流，并可以设置控制参数，通过GPIO连接电脑开机引脚控制电脑开关机。  
性能参数  
输入最大21v 6.4A，输出维持在不低于电池电压以上。电池放电测试中可以到10A，电池充电电流可达8A，可以输入电源和电池同时放电供负载使用。超过3A建议将芯片mosfet加散热片。支持锂电池，蓄电池等各类电池。  
接线说明  
 D2   //BQ25713 charge_OK 输出  
 D5   //电脑开机状态LED,接到电脑上led引脚，用来判断电脑是否开机  
 D6   //电脑开机引脚,接到电脑主板上的开机引脚  
 D1   //重设网络参数 引脚接到gnd触发  
 D8   //机箱上电脑开机引脚,接到机箱开关接到VCC，高电平触发  
 D7   //机箱电脑开机状态LED,接到gnd  
 D3   // SCL  
 D4   // SDA  