/**
 * Light_dimmer：
 *    Mysensor 调光灯库
 *    初始化：
 *       new Light_dimmer(PWM引脚,child-sensor-id,EEPROM地址);
 *       注意： EEPROM地址 库里面是占2个字节，第一个字节存放灯亮度信息 uint8_t ，第二个字节存放灯状态 V_LIGHT
 */


#define LIGHT_OFF 0
#define LIGHT_ON 1
#define MAX_LEVEL 100
#define MIN_LEVEL 0

class Light_dimmer
{
  private:
    int pin_;
    uint8_t sensorId_;
    uint8_t Dimmer_Level_pos_;
    uint8_t Light_State_pos_;
    char convBuffer_[10];
    uint8_t currentLevel_ = 0;
    bool sendStated_ = false; // 是否发送状态
    bool LightState_;
  public:
    MyMessage * dimmerMsg;
    MyMessage * lightMsg;
    uint8_t toLevel;
    void show();
    void setup();
    void presentation();
    void receive(const MyMessage &message);
    int loadLevelState(const uint8_t &pos);
    void saveLevelState(const uint8_t &pos, const byte &data);
    void toggle();
    void ChangeLightState(const bool& State);
    Light_dimmer(const int &pin, const uint8_t &sensorId, const uint8_t &pos);
};

Light_dimmer::Light_dimmer(const int &pin, const uint8_t &sensorId, const uint8_t &pos)
    : pin_(pin),
      sensorId_(sensorId),
      Dimmer_Level_pos_(pos),
      dimmerMsg(new MyMessage(sensorId_, V_DIMMER)),
      lightMsg(new MyMessage(sensorId_, V_LIGHT)){
        Light_State_pos_ = pos + 1;
      }

void Light_dimmer::setup()
{
  pinMode(pin_, OUTPUT);
  // request(sensorId_, V_DIMMER);
  // wait(1000);
  LightState_ = this->loadLevelState(Light_State_pos_);
  if (LightState_ == LIGHT_ON)
  {
    toLevel = this->loadLevelState(Dimmer_Level_pos_);
  } else {
    toLevel = LIGHT_OFF;
  }
  // 初始化的时候需要公告当前状态值
  send(dimmerMsg->set(toLevel));
  send(lightMsg->set(LightState_));
}

void Light_dimmer::presentation()
{
  present(sensorId_,S_DIMMER);
  present(sensorId_,S_DIMMER);
}

void Light_dimmer::ChangeLightState(const bool &State){
  LightState_=(bool)State;
  send(lightMsg->set(LightState_));
}

void Light_dimmer::show(){
  if (currentLevel_ != toLevel) {
    sendStated_ = false;
    currentLevel_ < toLevel ?  currentLevel_++ : currentLevel_-- ;
    analogWrite( pin_, (int)(currentLevel_ / 100. * 255) );
  }
  // 当亮度切换完毕后刷新灯的状态。
  if (currentLevel_ == toLevel && !sendStated_){
    send(dimmerMsg->set(currentLevel_));

    // 当亮度改变的时候，同时更新灯状态
    if (currentLevel_ != 0 && LightState_ == LIGHT_OFF){
      this->ChangeLightState(LIGHT_ON);
    }
    if (currentLevel_ == 0 && LightState_ == LIGHT_ON){
      this->ChangeLightState(LIGHT_OFF);
    }
    sendStated_ = true;
  }
}

int Light_dimmer::loadLevelState(const uint8_t &pos) {
  uint8_t min;
  uint8_t max;
  if (pos == Dimmer_Level_pos_)
  {
    min=MIN_LEVEL;
    max=MAX_LEVEL;
  }

  if (pos == Light_State_pos_)
  {
    min=LIGHT_OFF;
    max=LIGHT_ON;
  }

  return min(max(loadState(pos),min),max);
}

void Light_dimmer::saveLevelState(const uint8_t &pos, const byte &data) {
  uint8_t min;
  uint8_t max;
  if (pos == Dimmer_Level_pos_)
  {
    min=MIN_LEVEL;
    max=MAX_LEVEL;
  }

  if (pos == Light_State_pos_)
  {
    min=LIGHT_OFF;
    max=LIGHT_ON;
  }
  saveState(pos,min(max(data,min),max));
}

void Light_dimmer::receive(const MyMessage &message)
{
  if (message.type == V_LIGHT) 
  {
    // Incoming on/off command sent from controller ("1" or "0")
    message.getBool() == LIGHT_ON ? LightState_=LIGHT_ON : LightState_=LIGHT_OFF;
    this->saveLevelState(Light_State_pos_, LightState_);
    int newLevel = LIGHT_OFF;
    if (LightState_==LIGHT_ON) {
      // Pick up last saved dimmer level from the eeprom
      newLevel = this->loadLevelState(Dimmer_Level_pos_);
    } 
    toLevel = newLevel;
  }
  if (message.type == V_DIMMER) {
    // Incoming dim-level command sent from controller (or ack message)
    int newLevel = atoi(message.getString(convBuffer_));

    // 应当先保存状态，然后再改变值，否则渐变的时候保存值会导致渐变闪烁
    wait(100);
    this->saveLevelState(Dimmer_Level_pos_, newLevel); 
    toLevel = newLevel;
  }
}

void Light_dimmer::toggle() {
  if (currentLevel_ == LIGHT_OFF )
  {
    toLevel = this->loadLevelState(Dimmer_Level_pos_);
  } 
  else
  {
    toLevel = 0;
  }
}
