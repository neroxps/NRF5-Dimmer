/*
 *
 * Created by Neroxps <neroxps@gmail.com>
 * Copyright (C) 2013-2020 Neroxps
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - 2020年7月25日 - 初始版本
 * Version 1.1 - 2020年7月25日 - 优化渐变逻辑，修复BUG
 *
 * http://www.mysensors.org/build/dimmer
 */

// Enable debug prints to serial monitor
// #define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF5_ESB
#define MY_RF24_PA_LEVEL RF24_PA_MAX //MINI LOW HIGH MAX
#define MY_NRF5_ESB_REVERSE_ACK_RX
#define MY_TRANSPORT_WAIT_READY_MS 5000 //等待5秒，如果没连到网关直接启动
//#define MY_RX_MESSAGE_BUFFER_FEATURE
//#define MY_RX_MESSAGE_BUFFER_SIZE 10 //8MHZ 5-10 no-define 20  no-irq 3
#define MY_BAUD_RATE 9600 //115200 19200 9600 =8MHZ  4800 =1MHZ
// #define MY_NODE_ID 10  //Static NodeId
//#define MY_TRANSPORT_UPLINK_CHECK_DISABLED
//#define MY_PARENT_NODE_IS_STATIC
//#define MY_PARENT_NODE_ID 0
//#define MY_OTA_FIRMWARE_FEATURE
//#define MY_REPEATER_FEATURE //注释下列启用中继功能

#include <MySensors.h>
#include <Bounce2.h>

// 来自 Fastled 的 EVERY_N
// 从 https://github.com/FastLED/FastLED/blob/cfce255bbb6f299544b0e5c59e67f12d326ee4d4/lib8tion.h
// 截取片段代码实现非阻塞延迟
#include "Every_N.h"
#include "Light_dimmer.h"

#define SN "DimmableLED"
#define SV "1.1"

#define DIM_WHITE_PIN 11
#define DIM_WARM_PIN 13

// Light_dimmer 的 EEPROM 占用2个字节
#define EEPROM_DIM_WHITE_LEVEL_LAST 1
#define EEPROM_DIM_WARM_LEVEL_LAST 3
#define EEPROM_DELAY_MS 5

// #DEFINE CHILD_ID_RED = 75
// #DEFINE CHILD_ID_GREEN 76
// #DEFINE CHILD_ID_BLUE  77
#define CHILD_ID_WHITE 78
#define CHILD_ID_WARM 79
#define CHILD_ID_CUSTOM_1 23
// LED渐变时间间隔 毫秒
static uint8_t  delay_ms = 0;

static Light_dimmer Dimmer_White(DIM_WHITE_PIN, CHILD_ID_WHITE, EEPROM_DIM_WHITE_LEVEL_LAST);
static Light_dimmer Dimmer_Warm(DIM_WARM_PIN, CHILD_ID_WARM, EEPROM_DIM_WARM_LEVEL_LAST);
static MyMessage Custom(CHILD_ID_CUSTOM_1,V_VAR1);

Bounce debouncer = Bounce();

/***
  * 在 Mysensor 开始之前执行
  */
void before() {
  pinMode(PIN_LED1, OUTPUT);
  digitalWrite(PIN_LED1, HIGH);
  hwPinMode(PIN_BUTTON1,INPUT_PULLUP);
  debouncer.attach(PIN_BUTTON1);
  debouncer.interval(20);
}

void blinkity(uint8_t pulses, uint8_t repetitions) {
  for (int x = 0; x < repetitions; x++) {
    for (int i = 0; i < pulses; i++) {
      digitalWrite(PIN_LED1, HIGH);
      wait(20);
      digitalWrite(PIN_LED1, LOW);
      wait(100);
    }
    wait(100);
  }
}

/***
 * 向网关公告当前设备类型
 * 设备类型可从 https://www.mysensors.org/download/serial_api_20#variable-types 查询
 * 设备类型定义在 https://github.com/mysensors/MySensors/blob/development/core/MyMessage.h
 */
void presentation(){
  sendSketchInfo(SN, SV);
  wait(100);
  // Register the LED Dimmable Light with the gateway
  Dimmer_White.presentation();
  wait(100);
  Dimmer_Warm.presentation();
  wait(100);
  present(CHILD_ID_CUSTOM_1,S_CUSTOM);
  wait(100);
}

/***
 * Dimmable LED initialization method
 */
void setup(){
  /*检测当前2.4G链接并输出BLINK-LED */
  if (transportCheckUplink() == false) {
    blinkity(4, 2);
  }
  if (isTransportReady() == true)  {
    blinkity(2, 1);
  }

  // 从 eeprom 取出 delay的值
  if(loadState(EEPROM_DELAY_MS) > 100){
    // 如果是第一次启动给 delay_ms 设定一个初始值
    delay_ms = 30;
    saveState(EEPROM_DELAY_MS,min(max(delay_ms,0),100));
  } else {
    delay_ms=min(max(loadState(EEPROM_DELAY_MS),0),100);
  }
  send(Custom.set(delay_ms));

  Dimmer_White.setup();
  Dimmer_Warm.setup();
}

/***
 *  Dimmable LED main processing loop
 */
void loop(){
  /***
   *  板载按钮切换灯状态
   */
  boolean changed = debouncer.update();
  int debouncer_value = debouncer.read();
  if ( changed && debouncer_value == 0){
    blinkity(2, 3);
    Dimmer_White.toggle();
    Dimmer_Warm.toggle();
  }

  /***
   *  更新 LED 灯状态
   *  EVERY_N_MILLIS_I 是 fastLED lib8tion.h 库中宏定义
   *  这个宏定义指向 #define INSTANTIATE_EVERY_N_TIME_PERIODS(NAME,TIMETYPE,TIMEGETTER)
   *  这个宏定义最终编译的时候会代替成一个 Function 里面定义了一个叫 NAME 的 Class 对象
   *  如果我们在程序中需要修改 EVERY_N_MILLIS_I 的延迟事件，需要如下操作：
   *  EVERY_N_MILLIS_I(delay_time,2){ // 其中2ms是初始值
   *    // 这里写 LED 展示代码
   *  } 
   *  delay_time.setPeriod(10); // 这里是吧延迟时间更新为 10ms
   */ 
  EVERY_N_MILLIS_I(delay_time,delay_ms){
    Dimmer_White.show();
    Dimmer_Warm.show();
  }
  delay_time.setPeriod(delay_ms);
}



void receive(const MyMessage &message)
{
  /***
   *  场景响应
   */
  if (message.type == V_SCENE_ON){
    Dimmer_White.toggle();
    Dimmer_Warm.toggle();
  }

  /***
   *  来自外部调光信号 0-100
   */
  if ( message.type == V_DIMMER || message.type == V_LIGHT ){
    switch (message.sensor) {
      case CHILD_ID_WHITE:
        Dimmer_White.receive(message);
        break;
      case CHILD_ID_WARM:
        Dimmer_Warm.receive(message);
        break;
    }
  }

  /***
   *  来自外部信号自定义变量 V_VAR1，这里是设置灯光的渐变速度
   */
  if( message.type == V_VAR1 && message.sensor == S_CUSTOM ){
    delay_ms = message.getInt();
    saveState(EEPROM_DELAY_MS,min(max(delay_ms,0),100));
  }
}
