#include <ESP32Servo.h>
#include "sbus.h"  // Библиотека Bolder Flight Systems SBUS



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
const int LEFT_ENGINE_CHANNEL = 2;
const int RIGHT_ENGINE_CHANNEL = 3;

const int MIN_CHANNEL_VALUE = 172;
const int MAX_CHANNEL_VALUE = 1700;

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

    // переводим данные с канала в PPM
    int esc_me_Value = map(channels[MAIN_ENGINE_CHANNEL],  MIN_CHANNEL_VALUE, MAX_CHANNEL_VALUE, ESC_MAIN_ENGINE_OFF, ESC_MAIN_ENGINE_FULL_THROTTLE);
    int esc_le_Value = map(channels[LEFT_ENGINE_CHANNEL],  MIN_CHANNEL_VALUE, MAX_CHANNEL_VALUE, ESC_LR_ENGINE_OFF,   ESC_LR_ENGINE_FULL_THROTTLE  );
    int esc_re_Value = map(channels[RIGHT_ENGINE_CHANNEL], MIN_CHANNEL_VALUE, MAX_CHANNEL_VALUE, ESC_LR_ENGINE_OFF,   ESC_LR_ENGINE_FULL_THROTTLE  );

    if (failsafe) // failsafe
    {
      esc_me_Value = ESC_MAIN_ENGINE_OFF;
      esc_le_Value = ESC_LR_ENGINE_OFF;
      esc_re_Value = ESC_LR_ENGINE_OFF;
    }
    
    Serial.print("me CHANNEL VALUE:"); Serial.print(channels[MAIN_ENGINE_CHANNEL]);
    Serial.print(" PPM: "); Serial.println(esc_me_Value);
    Serial.print("    le CHANNEL VALUE:"); Serial.print(channels[LEFT_ENGINE_CHANNEL]);
    Serial.print(" PPM: "); Serial.println(esc_le_Value);
    Serial.print("    re CHANNEL VALUE:"); Serial.print(channels[RIGHT_ENGINE_CHANNEL]);
    Serial.print(" PPM: "); Serial.println(esc_re_Value);
    
    esc_me.writeMicroseconds(esc_me_Value);
    esc_le.writeMicroseconds(esc_le_Value);
    esc_re.writeMicroseconds(esc_re_Value);
  }
  delay(100);
}
