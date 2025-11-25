  #include <ESP32Servo.h>
  #include "sbus.h"  // Библиотека Bolder Flight Systems SBUS


  enum class side_t 
  {
      RIGHT,
      LEFT,
      NO_SIDE
  };

  // Пины esp32 отвечающие за посылку PPM на esc двигателей
  const int ESC_MAIN_ENGINE_PIN  = 13;
  const int ESC_LEFT_ENGINE_PIN  = 14;
  const int ESC_RIGHT_ENGINE_PIN = 15;



  // Границы PPM у esc двигателей
  const int ESC_MIN_PPM = 1000;
  const int ESC_MAX_PPM = 2000;



  // PPM выключенного/включенного мотороа
  // Поворотные двигатели (однонаправленные)
  const int ESC_LR_ENGINE_OFF = 1000;
  const int ESC_LR_ENGINE_FULL_THROTTLE = 2000;
  // Нвгнетательный двигатель (двунаправленный)
  const int ESC_MAIN_ENGINE_OFF = 1500;
  const int ESC_MAIN_ENGINE_FULL_THROTTLE = 2000;



  // Каналы управления (нумерация с 0)
  const int MAIN_ENGINE_CHANNEL = 10;
  const int MAIN_ENGINE_MIN_CHANNEL_VALUE = 172;
  const int MAIN_ENGINE_MAX_CHANNEL_VALUE = 1100;

  const int FORWARD_THROTTLE_CHANNEL = 2;
  const int FORWARD_MIN_CHANNEL_VALUE = 172;
  const int FORWARD_MAX_CHANNEL_VALUE = 1800;

  const int ROTATE_THROTTLE_CHANNEL = 0;
  const int ROTATE_ENGINE_MIN_CHANNEL_VALUE = 172;
  const int ROTATE_ENGINE_MIDDLE_CHANNEL_VALUE = 1000;
  const int ROTATE_ENGINE_MAX_CHANNEL_VALUE = 1800;

  const int ARM_CHANNEL = 4;

  // Создаём объект приёмника SBUS для ESP32
  // Пояснения:
  // &Serial2  — используем второй UART
  // 16        — RX-пин (куда подключен SBUS сигнал от приёмника)
  // 17        — TX-пин (не используется, но требуется библиотекой)
  // true      — инвертированный сигнал SBUS (такой у FrSky R9MM)
  // false     — "fast" режим отключён
  bfs::SbusRx sbus_rx(&Serial2, 16, 17, true, false);
  uint16_t channels[16] = {0};



  Servo esc_me; // main engine
  Servo esc_le; // left engine
  Servo esc_re; // right engine



  void setup() {
    Serial.begin(115200);
    delay(500);

    sbus_rx.Begin();
    Serial.println("ESP32 + FrSky R9MM OTA SBUS Receiver started");
    

    // initialize esc pins
    esc_me.attach(ESC_MAIN_ENGINE_PIN, ESC_MIN_PPM, ESC_MAX_PPM);
    esc_le.attach(ESC_LEFT_ENGINE_PIN, ESC_MIN_PPM, ESC_MAX_PPM);
    esc_re.attach(ESC_RIGHT_ENGINE_PIN, ESC_MIN_PPM, ESC_MAX_PPM);
    esc_me.writeMicroseconds(ESC_MAIN_ENGINE_OFF);
    esc_le.writeMicroseconds(ESC_LR_ENGINE_OFF);
    esc_re.writeMicroseconds(ESC_LR_ENGINE_OFF);

    delay(5000);
  }

  void loop() {
    if (sbus_rx.Read())
    {
      bfs::SbusData data = sbus_rx.data();
      
      for (int i = 0; i < bfs::SbusData::NUM_CH; i++) channels[i] = data.ch[i];

      bool failsafe = data.failsafe;

      // channel value -> PPM
      int esc_me_Value = map(channels[MAIN_ENGINE_CHANNEL],  MAIN_ENGINE_MIN_CHANNEL_VALUE, MAIN_ENGINE_MAX_CHANNEL_VALUE, ESC_MAIN_ENGINE_OFF, ESC_MAIN_ENGINE_FULL_THROTTLE);
      int esc_le_Value = map(channels[FORWARD_THROTTLE_CHANNEL],  FORWARD_MIN_CHANNEL_VALUE, FORWARD_MAX_CHANNEL_VALUE, ESC_LR_ENGINE_OFF,   ESC_LR_ENGINE_FULL_THROTTLE  );
      int esc_re_Value = map(channels[FORWARD_THROTTLE_CHANNEL], FORWARD_MIN_CHANNEL_VALUE, FORWARD_MAX_CHANNEL_VALUE, ESC_LR_ENGINE_OFF,   ESC_LR_ENGINE_FULL_THROTTLE  );

      int rotate_channel_cur = channels[ROTATE_THROTTLE_CHANNEL];

      side_t side = side_t::NO_SIDE;
      if ((rotate_channel_cur - ROTATE_ENGINE_MIDDLE_CHANNEL_VALUE) < -50)
      {
        side = side_t::RIGHT;
      }
      else if ((rotate_channel_cur - ROTATE_ENGINE_MIDDLE_CHANNEL_VALUE) > 50)
      {
        side = side_t::LEFT;
      }
      

      // right rotate
      if (side == side_t::RIGHT)
      {
          int ppm_rotate_channel_cur = map(abs(1000 - rotate_channel_cur) + ROTATE_ENGINE_MIN_CHANNEL_VALUE, ROTATE_ENGINE_MIN_CHANNEL_VALUE, ROTATE_ENGINE_MIDDLE_CHANNEL_VALUE, 0, 500);
          esc_re_Value += ppm_rotate_channel_cur;
          esc_le_Value -= ppm_rotate_channel_cur;

      }
      // left rotate
      else if (side == side_t::LEFT)
      {
          int ppm_rotate_channel_cur = map(abs(rotate_channel_cur - 1000) + ROTATE_ENGINE_MIDDLE_CHANNEL_VALUE, ROTATE_ENGINE_MIDDLE_CHANNEL_VALUE, ROTATE_ENGINE_MAX_CHANNEL_VALUE, 0, 500);
          esc_re_Value -= ppm_rotate_channel_cur;
          esc_le_Value += ppm_rotate_channel_cur;  
      }

      // check limits
      if (esc_re_Value < ESC_LR_ENGINE_OFF)
          esc_re_Value = ESC_LR_ENGINE_OFF;
      else if (esc_re_Value > ESC_LR_ENGINE_FULL_THROTTLE)
          esc_re_Value = ESC_LR_ENGINE_FULL_THROTTLE;


      if (esc_le_Value < ESC_LR_ENGINE_OFF)
          esc_le_Value = ESC_LR_ENGINE_OFF;
      else if (esc_le_Value > ESC_LR_ENGINE_FULL_THROTTLE)
          esc_le_Value = ESC_LR_ENGINE_FULL_THROTTLE;
          
      // failsafe    
      if (failsafe)
      {
        esc_me_Value = ESC_MAIN_ENGINE_OFF;
        esc_le_Value = ESC_LR_ENGINE_OFF;
        esc_re_Value = ESC_LR_ENGINE_OFF;
      }
      if (channels[ARM_CHANNEL] < 1000)
      {
        esc_me_Value = ESC_MAIN_ENGINE_OFF;
        esc_le_Value = ESC_LR_ENGINE_OFF;
        esc_re_Value = ESC_LR_ENGINE_OFF;
      }
      
      // debug info
      // Serial.print("me CHANNEL VALUE:"); Serial.print(channels[MAIN_ENGINE_CHANNEL]);
      // Serial.print("    forward ch val"); Serial.print(channels[FORWARD_THROTTLE_CHANNEL]);
      // Serial.print("    rotate ch val"); Serial.print(channels[ROTATE_THROTTLE_CHANNEL]);
      // Serial.print("main PPM: "); Serial.println(esc_me_Value);
      // Serial.print("    right PPM: "); Serial.println(esc_re_Value);
      // Serial.print("    left PPM: "); Serial.println(esc_le_Value);
      Serial.print("    arm: "); Serial.println(channels[ARM_CHANNEL]);
      
      // load new ppm
      esc_me.writeMicroseconds(esc_me_Value);
      esc_le.writeMicroseconds(esc_le_Value);
      esc_re.writeMicroseconds(esc_re_Value);
    }
    delay(100);
  }
