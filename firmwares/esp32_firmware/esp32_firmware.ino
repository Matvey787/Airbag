// For working with a gyroscope/accelerometer
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
// For working with GPS and Compass
#include <HardwareSerial.h>
#include <QMC5883LCompass.h>
#include <TinyGPS++.h>
// For working with PWM (motors)
#include <ESP32Servo.h>
// For working with communication channels
#include <sbus.h>
// For working with parsing firmware settings (firmware_vars.json)
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <Wire.h>
#include <cstdarg>
#include <math.h>
#include <string>
#include <unordered_map>
#include <vector>

// MAVLink headers ЗАКОММЕНТИРОВАНЫ до решения аппаратных вопросов
// #include <generated/include/common/mavlink.h>

// ----------------------------------------------------------------------------
// DEBUG MACRO & CONFIGURATION
// ----------------------------------------------------------------------------
#define DEBUG_M // Включено по умолчанию, пока MAVLink отключен

#ifdef DEBUG_M
#define DBG_SERIAL Serial
#define DBG_PRINT(...) DBG_SERIAL.printf(__VA_ARGS__)
#define DBG_PRINTLN(...) DBG_SERIAL.println(__VA_ARGS__)
#define MAVLINK_ENABLED false
#else
#define DBG_PRINT(...)
#define DBG_PRINTLN(...)
#define MAVLINK_ENABLED true
#endif

namespace Setup
{
constexpr const char* WIFI_NAME     = "ESP32-Debug";
constexpr const char* WIFI_PASSWORD = "12345678";
constexpr uint16_t WIFI_PORT        = 8080;

constexpr int ESC_MAIN  = 13;
constexpr int ESC_LEFT  = 14;
constexpr int ESC_RIGHT = 15;

constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;

// MAVLink пины ЗАКОММЕНТИРОВАНЫ
// constexpr int MAVLINK_RX      = 33;
// constexpr int MAVLINK_TX      = 32;
// constexpr uint32_t MAVLINK_BAUD = 57600;

// SBUS Receiver
constexpr int SBUS_RX = 16;
constexpr int SBUS_TX = 26;

// GPS (Аппаратный UART1)
constexpr int GPS_RX    = 17;
constexpr int GPS_TX    = 18;
constexpr uint32_t GPS_BAUD = 9600;

constexpr int DHT_FRONT_BAY = 23;
constexpr int DHT_REAR_BAY  = 19;

constexpr int PPM_MIN   = 1000;
constexpr int PPM_MAX   = 2000;
constexpr int LR_OFF    = 1000;
constexpr int LR_FULL   = 2000;
constexpr int MAIN_OFF  = 1500;
constexpr int MAIN_FULL = 2000;

constexpr int CH_MAIN_ENGINE     = 10;
constexpr int CH_MAIN_ENGINE_MIN = 172;
constexpr int CH_MAIN_ENGINE_MAX = 1100;
constexpr int CH_FORWARD         = 2;
constexpr int CH_FORWARD_MIN     = 172;
constexpr int CH_FORWARD_MAX     = 1800;
constexpr int CH_ROTATE          = 0;
constexpr int CH_ROTATE_MIN      = 172;
constexpr int CH_ROTATE_MID      = 1000;
constexpr int CH_ROTATE_MAX      = 1800;
constexpr int CH_ARM             = 4;
constexpr int CH_ARM_MIN         = 900;
constexpr int CH_ARM_MAX         = 1100;
constexpr int CH_HH              = 8;
constexpr int CH_HH_MIN          = 900;
constexpr int CH_HH_MAX          = 1100;
constexpr int CH_COUNT           = 16;

constexpr uint8_t MAG_MAX_REJECT_STREAK = 5;
constexpr float MAG_SPIKE_THRESHOLD     = 45.0f;
constexpr float MAG_DT_MAX              = 0.1f;

constexpr float PID_INTEGRAL_LIMIT  = 200.0f;
constexpr int PID_MAX_OUTPUT        = 300;
constexpr int PID_MAX_STEP_PER_LOOP = 15;
constexpr float PID_D_FILTER_ALPHA  = 0.2f;
constexpr float PID_P               = 0.8f;
constexpr float PID_I               = 0.03f;
constexpr float PID_D               = 0.05f;

constexpr float HH_MAG_CORRECTION_GAIN = 0.08f;
constexpr float HH_ROT_WEIGHT_GYRO     = 0.8;
constexpr float HH_MAX_MAG_RATE        = 15.0f;
constexpr float HH_STICK_RATE_SCALE    = 90.0f;
constexpr int HH_STICK_DEADZONE        = 50;
constexpr float HH_DT                  = 0.02f;

constexpr unsigned long ENV_POLL_INTERVAL_MS = 2000;
constexpr unsigned long SAVE_INTERVAL_MS     = 5000;
} // namespace Setup

// ----------------------------------------------------------------------------
// Wi-Fi Handler
// ----------------------------------------------------------------------------
class WiFiLink
{
    WiFiServer server_{ Setup::WIFI_PORT };
    WiFiClient client_;

    void sendConfig(const std::unordered_map<std::string, float>& params)
    {
        DBG_PRINT("START_CONFIG\n");
        for (auto const& [key, val] : params)
            DBG_PRINT("SET %s %.4f\n", key.c_str(), val);
        DBG_PRINT("END_CONFIG\n");
    }

    bool handleCommand(
        const String& cmd, std::unordered_map<std::string, float>& params)
    {
        if (cmd == "GET_CONFIG")
        {
            sendConfig(params);
            return false;
        }
        else if (cmd.startsWith("SET "))
        {
            auto firstSpace  = cmd.indexOf(' ');
            auto secondSpace = cmd.indexOf(' ', firstSpace + 1);
            if (secondSpace > 0)
            {
                params[cmd.substring(firstSpace + 1, secondSpace).c_str()] =
                    cmd.substring(secondSpace + 1).toFloat();
                return true;
            }
        }
        else if (cmd == "HI_ESP")
            DBG_PRINT("HI_GUI\n");
        return false;
    }

public:
    void begin(const char* ssid,
        const char* password,
        uint16_t port = Setup::WIFI_PORT)
    {
        WiFi.mode(WIFI_AP);
        WiFi.softAP(ssid, password);
        server_ = WiFiServer(port);
        server_.begin();
        DBG_PRINT("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
        DBG_PRINT("TCP server on port %u\n", port);
    }

    bool poll(std::unordered_map<std::string, float>& params)
    {
        bool modified = false;
        if (server_.hasClient())
        {
            if (client_)
                client_.stop();
            client_ = server_.available();
            DBG_PRINT("TCP client connected\n");
        }
        if (client_ && !client_.connected())
        {
            client_.stop();
            DBG_PRINT("Disconnected\n");
        }
        if (client_ && client_.available())
        {
            String line = client_.readStringUntil('\n');
            line.trim();
            if (line.length() > 0 && handleCommand(line, params))
                modified = true;
        }
        return modified;
    }

    void printf(const char* fmt, ...)
    {
        char buf[256];
        va_list args;
        va_start(args, fmt);
        vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);
        if (client_ && client_.connected())
            client_.print(buf);
    }
};

// ----------------------------------------------------------------------------
// Firmware Parameters Manager
// ----------------------------------------------------------------------------
class FirmwareParams
{
    std::unordered_map<std::string, float> params_;
    bool dirty_                     = false;
    unsigned long lastSaveTimeMs_   = 0;
    WiFiLink& link_;

public:
    explicit FirmwareParams(WiFiLink& link) : link_(link) {}

    float* getParamPtr(const std::string& key, float defaultVal)
    {
        auto it = params_.find(key);
        if (it != params_.end())
            return &(it->second);
        params_[key] = defaultVal;
        return &params_[key];
    }

    template <bool SendToGui = false>
    void setParam(const std::string& key, float val)
    {
        params_[key] = val;
        if constexpr (SendToGui)
            link_.printf("SET %s %.4f\n", key.c_str(), val);
    }

    bool isDirty() const { return dirty_; }
    void markDirty() { dirty_ = true; }
    void clearDirty() { dirty_ = false; }
    unsigned long& lastSaveTime() { return lastSaveTimeMs_; }
    const std::unordered_map<std::string, float>& getAll() const
    {
        return params_;
    }
    std::unordered_map<std::string, float>& getAllMutable() { return params_; }

    void loadFromFS()
    {
        if (!LittleFS.begin(true))
        {
            DBG_PRINTLN("FS Mount Fail");
            return;
        }
        File file = LittleFS.open("/firmware_vars.json", "r");
        if (!file)
        {
            DBG_PRINTLN("JSON Open Fail");
            return;
        }

        DynamicJsonDocument doc(16384);
        DeserializationError error = deserializeJson(doc, file);
        if (error)
        {
            DBG_PRINT("JSON Parse Error: ");
            DBG_PRINTLN(error.c_str());
            file.close();
            return;
        }

        params_.clear();
        for (JsonObject obj : doc.as<JsonArray>())
            for (JsonPair kv : obj)
                if (kv.value().as<JsonObject>().containsKey("val"))
                    params_[std::string(kv.key().c_str())] =
                        kv.value()["val"].as<float>();

        file.close();
        DBG_PRINTLN("Params loaded from JSON");
    }

    void saveToFS()
    {
        if (!dirty_)
            return;
        DynamicJsonDocument doc(16384);
        JsonArray array = doc.to<JsonArray>();
        for (const auto& [key, val] : params_)
        {
            JsonObject obj                       = array.createNestedObject();
            obj.createNestedObject(key)["val"] = val;
        }

        File file = LittleFS.open("/firmware_vars.json", "w");
        if (!file || serializeJson(doc, file) == 0)
            DBG_PRINTLN("FS Write Fail");
        else
            DBG_PRINT("Params saved (%d entries)\n", params_.size());

        file.close();
        dirty_          = false;
        lastSaveTimeMs_ = millis();
    }
};

/*
// ----------------------------------------------------------------------------
// MAVLink Handler — ПОЛНОСТЬЮ ЗАКОММЕНТИРОВАН
// Раскомментируйте весь блок ниже, когда будете готовы к подключению OSD
// ----------------------------------------------------------------------------
#if MAVLINK_ENABLED
class MavlinkHandler
{
    HardwareSerial mavSerial_;
    FirmwareParams& fp_;
    uint8_t sysId_, compId_;
    unsigned long lastHeartbeat_;
    mavlink_status_t status_;

    void sendHeartbeat()
    {
        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack(sysId_,
            compId_,
            &msg,
            MAV_TYPE_GROUND_ROVER,
            MAV_AUTOPILOT_ARDUPILOTMEGA,
            0,
            0,
            MAV_STATE_ACTIVE);
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavSerial_.write(buf, mavlink_msg_to_send_buffer(buf, &msg));
    }

    void sendAttitude()
    {
        mavlink_message_t msg;
        mavlink_msg_attitude_pack(sysId_,
            compId_,
            &msg,
            millis(),
            *fp_.getParamPtr("ROLL", 0) * DEG_TO_RAD,
            *fp_.getParamPtr("PITCH", 0) * DEG_TO_RAD,
            *fp_.getParamPtr("YAW", 0) * DEG_TO_RAD,
            *fp_.getParamPtr("ANG_VEL_X", 0) * DEG_TO_RAD,
            *fp_.getParamPtr("ANG_VEL_Y", 0) * DEG_TO_RAD,
            *fp_.getParamPtr("ANG_VEL_Z", 0) * DEG_TO_RAD);
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavSerial_.write(buf, mavlink_msg_to_send_buffer(buf, &msg));
    }

    void handleMessage(const mavlink_message_t& msg)
    {
        if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG)
        {
            mavlink_command_long_t cmd;
            mavlink_msg_command_long_decode(&msg, &cmd);
            if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM)
                DBG_PRINT("MAV: Arm req: %d\n", (int)(cmd.param1 > 0.5f));
        }
    }

public:
    MavlinkHandler(FirmwareParams& fp,
        uint8_t sysId  = 1,
        uint8_t compId = MAV_COMP_ID_ONBOARD_COMPUTER) :
        mavSerial_(2), fp_(fp), sysId_(sysId), compId_(compId),
        lastHeartbeat_(0)
    {}

    void begin()
    {
        mavSerial_.begin(Setup::MAVLINK_BAUD,
            SERIAL_8N1,
            Setup::MAVLINK_RX,
            Setup::MAVLINK_TX);
        mavSerial_.setRxBufferSize(2048);
        DBG_PRINT("MAVLink OK (GPIO%d/%d @ %lu)\n",
            Setup::MAVLINK_TX,
            Setup::MAVLINK_RX,
            Setup::MAVLINK_BAUD);
    }

    void update()
    {
        unsigned long now = millis();
        if (now - lastHeartbeat_ >= 1000)
        {
            lastHeartbeat_ = now;
            sendHeartbeat();
        }

        static unsigned long lastAtt = 0;
        if (now - lastAtt >= 20)
        {
            lastAtt = now;
            sendAttitude();
        }

        while (mavSerial_.available())
        {
            mavlink_message_t msg;
            if (mavlink_parse_char(
                    MAVLINK_COMM_0, mavSerial_.read(), &msg, &status_))
                handleMessage(msg);
        }
    }
};
#endif
*/

// ----------------------------------------------------------------------------
// Heading Sensor
// ----------------------------------------------------------------------------
class HeadingSensor
{
    Adafruit_MPU6050 mpu_;
    QMC5883LCompass mag_;
    FirmwareParams& fp_;
    bool mpuReady_ = false, magReady_ = false;
    float lastValidHeading_            = 0;
    unsigned long lastHeadingTime_     = 0;
    uint8_t rejectStreak_              = 0;

    float tiltCompensatedHeading()
    {
        mag_.read();
        float mx = mag_.getX(), my = mag_.getY(), mz = mag_.getZ();
        float ax = *fp_.getParamPtr("ACCEL_X", 0);
        float ay = *fp_.getParamPtr("ACCEL_Y", 0);
        float az = *fp_.getParamPtr("ACCEL_Z", 0);

        float norm = sqrtf(ax * ax + ay * ay + az * az);
        if (norm < 0.01f)
            norm = 1.0f;
        ax /= norm;
        ay /= norm;
        az /= norm;

        float pitchRad = -asinf(constrain(-ax, -1.0f, 1.0f));
        float rollRad  = atan2f(-ay, -az);
        float Xh       = mx * cosf(pitchRad) + mz * sinf(pitchRad);
        float Yh = mx * sinf(rollRad) * sinf(pitchRad) + my * cosf(rollRad) -
                   mz * sinf(rollRad) * cosf(pitchRad);
        float heading = atan2f(-Yh, Xh) * RAD_TO_DEG;
        return heading < 0 ? heading + 360.0f : heading;
    }

public:
    explicit HeadingSensor(FirmwareParams& fp) : fp_(fp) {}

    bool begin()
    {
        mpuReady_ = mpu_.begin();
        if (mpuReady_)
        {
            mpu_.setAccelerometerRange(MPU6050_RANGE_4_G);
            mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
            mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);
        }
        mag_.init();
        mag_.setSmoothing(10, true);
        magReady_ = true;
        return mpuReady_;
    }

    void update()
    {
        if (!mpuReady_)
            return;
        sensors_event_t a, g, temp;
        mpu_.getEvent(&a, &g, &temp);

        fp_.setParam<true>("ACCEL_X", a.acceleration.x);
        fp_.setParam<true>("ACCEL_Y", a.acceleration.y);
        fp_.setParam<true>("ACCEL_Z", a.acceleration.z);
        fp_.setParam<true>("ANG_VEL_X", g.gyro.x * RAD_TO_DEG);
        fp_.setParam<true>("ANG_VEL_Y", g.gyro.y * RAD_TO_DEG);
        fp_.setParam<true>("ANG_VEL_Z", -g.gyro.z * RAD_TO_DEG);

        float norm = sqrtf(a.acceleration.x * a.acceleration.x +
                           a.acceleration.y * a.acceleration.y +
                           a.acceleration.z * a.acceleration.z);
        if (norm > 0.01f)
        {
            fp_.setParam<true>("PITCH",
                -asinf(constrain(-a.acceleration.x / norm, -1.0f, 1.0f)) *
                    RAD_TO_DEG);
            fp_.setParam<true>("ROLL",
                atan2f(-a.acceleration.y, -a.acceleration.z) * RAD_TO_DEG);
        }
    }

    float getHeading()
    {
        float h = mpuReady_ ? tiltCompensatedHeading() : []()
        {
            QMC5883LCompass m;
            m.read();
            float az = m.getAzimuth();
            return az < 0 ? az + 360 : az;
        }();

        unsigned long now = millis();
        float dt          = (now - lastHeadingTime_) / 1000.0f;
        lastHeadingTime_  = now;
        if (dt <= 0)
            dt = 0.02f;

        float diff = h - lastValidHeading_;
        while (diff > 180)
            diff -= 360;
        while (diff < -180)
            diff += 360;

        if (fabsf(diff) > *fp_.getParamPtr("MAG_SPIKE_THRESHOLD",
                              Setup::MAG_SPIKE_THRESHOLD) &&
            dt < *fp_.getParamPtr("MAG_DT_MAX", Setup::MAG_DT_MAX))
        {
            if (++rejectStreak_ <
                (uint8_t)*fp_.getParamPtr(
                    "MAG_MAX_REJECT_STREAK", Setup::MAG_MAX_REJECT_STREAK))
                return lastValidHeading_;
        }
        rejectStreak_     = 0;
        lastValidHeading_ = h;
        fp_.setParam<true>("YAW", h);
        return h;
    }

    bool isMpuReady() const { return mpuReady_; }
    bool isMagReady() const { return magReady_; }
    float gyroZ() const
    {
        return *const_cast<FirmwareParams&>(fp_).getParamPtr("ANG_VEL_Z", 0);
    }
    float gyroX() const
    {
        return *const_cast<FirmwareParams&>(fp_).getParamPtr("ANG_VEL_X", 0);
    }
    float gyroY() const
    {
        return *const_cast<FirmwareParams&>(fp_).getParamPtr("ANG_VEL_Y", 0);
    }
    float pitch() const
    {
        return *const_cast<FirmwareParams&>(fp_).getParamPtr("PITCH", 0);
    }
    float roll() const
    {
        return *const_cast<FirmwareParams&>(fp_).getParamPtr("ROLL", 0);
    }
};

// ----------------------------------------------------------------------------
// Environment Sensor
// ----------------------------------------------------------------------------
class BayEnvironmentSensor
{
    DHT dht_;
    FirmwareParams& fp_;
    std::string tempKey_, humKey_;
    unsigned long lastPollMs_ = 0;
    float temperature_ = NAN, humidity_ = NAN;

public:
    BayEnvironmentSensor(uint8_t pin,
        FirmwareParams& fp,
        const std::string& tk,
        const std::string& hk,
        uint8_t type = DHT11) :
        dht_(pin, type), fp_(fp), tempKey_(tk), humKey_(hk)
    {}
    void begin() { dht_.begin(); }
    void update()
    {
        if (millis() - lastPollMs_ < Setup::ENV_POLL_INTERVAL_MS)
            return;
        lastPollMs_ = millis();
        float t = dht_.readTemperature(), h = dht_.readHumidity();
        if (isnan(t) || isnan(h))
            return;
        temperature_ = t;
        humidity_    = h;
        fp_.setParam<true>(tempKey_, t);
        fp_.setParam<true>(humKey_, h);
    }
    float temperature() const { return temperature_; }
    float humidity() const { return humidity_; }
};

// ----------------------------------------------------------------------------
// Channels & Mixers
// ----------------------------------------------------------------------------
struct SbusFrame
{
    uint16_t channels[Setup::CH_COUNT];
    bool failsafe;
};
class IChannelObserver
{
public:
    virtual ~IChannelObserver()                     = default;
    virtual void onChannelsUpdate(const SbusFrame&) = 0;
};

class ChannelBus
{
    std::vector<IChannelObserver*> obs_;

public:
    void subscribe(IChannelObserver* o) { obs_.push_back(o); }
    void publish(const SbusFrame& f)
    {
        for (auto* o : obs_)
            o->onChannelsUpdate(f);
    }
};

class ChannelHandler
{
    bfs::SbusRx sbus_rx_;
    ChannelBus bus_;

public:
    ChannelHandler() :
        sbus_rx_(&Serial2, Setup::SBUS_RX, Setup::SBUS_TX, true, false)
    {}
    void begin() { sbus_rx_.Begin(); }
    void subscribe(IChannelObserver* o) { bus_.subscribe(o); }
    void update()
    {
        if (sbus_rx_.Read())
        {
            SbusFrame f;
            f.failsafe = sbus_rx_.data().failsafe;
            for (int i = 0; i < Setup::CH_COUNT; i++)
                f.channels[i] = sbus_rx_.data().ch[i];
            bus_.publish(f);
        }
    }
};

class ThrottleMixer : public IChannelObserver
{
    FirmwareParams& fp_;

public:
    ThrottleMixer(FirmwareParams& f) : fp_(f) {}
    void onChannelsUpdate(const SbusFrame& f) override
    {
        int chM =
            (int)*fp_.getParamPtr("CH_MAIN_ENGINE", Setup::CH_MAIN_ENGINE);
        fp_.setParam<true>("PWM_ME",
            map(f.channels[chM],
                *fp_.getParamPtr(
                    "CH_MAIN_ENGINE_MIN", Setup::CH_MAIN_ENGINE_MIN),
                *fp_.getParamPtr(
                    "CH_MAIN_ENGINE_MAX", Setup::CH_MAIN_ENGINE_MAX),
                Setup::MAIN_OFF,
                Setup::MAIN_FULL));

        int chF = (int)*fp_.getParamPtr("CH_FORWARD", Setup::CH_FORWARD);
        int fwd = map(f.channels[chF],
            Setup::CH_FORWARD_MIN,
            Setup::CH_FORWARD_MAX,
            Setup::LR_OFF,
            Setup::LR_FULL);
        fp_.setParam<false>("PWM_LE", fwd);
        fp_.setParam<false>("PWM_RE", fwd);
    }
};

class RotationMixer : public IChannelObserver
{
    FirmwareParams& fp_;

public:
    RotationMixer(FirmwareParams& f) : fp_(f) {}
    void onChannelsUpdate(const SbusFrame& f) override
    {
        int offset =
            f.channels[(int)*fp_.getParamPtr("CH_ROTATE", Setup::CH_ROTATE)] -
            Setup::CH_ROTATE_MID;
        int le = *fp_.getParamPtr("PWM_LE", Setup::LR_OFF),
            re = *fp_.getParamPtr("PWM_RE", Setup::LR_OFF);
        if (abs(offset) > 50)
        {
            int ppm = map(abs(offset), 50, 800, 0, 400);
            fp_.setParam<false>("PWM_LE", offset < 0 ? le - ppm : le + ppm);
            fp_.setParam<false>("PWM_RE", offset < 0 ? re + ppm : re - ppm);
        }
    }
};

class RatePid
{
    FirmwareParams& fp_;
    float integral_ = 0, prevGyro_ = 0, filteredD_ = 0;
    int prevOutput_         = 0;
    unsigned long lastTime_ = 0;

public:
    RatePid(FirmwareParams& f) : fp_(f) {}
    void reset()
    {
        integral_ = prevGyro_ = filteredD_ = 0;
        prevOutput_                        = 0;
        lastTime_                          = millis();
    }
    int update(float target, float gyro)
    {
        unsigned long now = millis();
        float dt          = (now - lastTime_) / 1000.0f;
        lastTime_         = now;
        if (dt <= 0 || dt > 0.1f)
            dt = 0.02f;

        float err = target - gyro;
        integral_ = constrain(integral_ + err * dt,
            -*fp_.getParamPtr("PID_INTEGRAL_LIMIT", Setup::PID_INTEGRAL_LIMIT),
            *fp_.getParamPtr("PID_INTEGRAL_LIMIT", Setup::PID_INTEGRAL_LIMIT));
        filteredD_ =
            (1 - *fp_.getParamPtr(
                     "PID_D_FILTER_ALPHA", Setup::PID_D_FILTER_ALPHA)) *
                filteredD_ +
            *fp_.getParamPtr("PID_D_FILTER_ALPHA", Setup::PID_D_FILTER_ALPHA) *
                (gyro - prevGyro_) / dt;
        prevGyro_ = gyro;

        float out = *fp_.getParamPtr("PID_P", Setup::PID_P) * err +
                    *fp_.getParamPtr("PID_I", Setup::PID_I) * integral_ -
                    *fp_.getParamPtr("PID_D", Setup::PID_D) * filteredD_;
        int raw   = constrain((int)roundf(out),
              -(int)*fp_.getParamPtr("PID_MAX_OUTPUT", Setup::PID_MAX_OUTPUT),
              (int)*fp_.getParamPtr("PID_MAX_OUTPUT", Setup::PID_MAX_OUTPUT));
        int delta = constrain(raw - prevOutput_,
            -(int)*fp_.getParamPtr(
                "PID_MAX_STEP_PER_LOOP", Setup::PID_MAX_STEP_PER_LOOP),
            (int)*fp_.getParamPtr(
                "PID_MAX_STEP_PER_LOOP", Setup::PID_MAX_STEP_PER_LOOP));
        return (prevOutput_ += delta);
    }
};

class HeadingHoldController : public IChannelObserver
{
    HeadingSensor& sensor_;
    WiFiLink& link_;
    FirmwareParams& fp_;
    RatePid pid_;
    float targetHeading_ = 0;
    bool active_         = false;
    static float wrap360(float v)
    {
        while (v < 0)
            v += 360;
        while (v >= 360)
            v -= 360;
        return v;
    }
    static float angleErr(float t, float c)
    {
        float e = t - c;
        while (e > 180)
            e -= 360;
        while (e < -180)
            e += 360;
        return e;
    }

public:
    HeadingHoldController(HeadingSensor& s, WiFiLink& l, FirmwareParams& f) :
        sensor_(s), link_(l), fp_(f), pid_(f)
    {}
    void onChannelsUpdate(const SbusFrame& f) override
    {
        bool armed =
            f.channels[(int)*fp_.getParamPtr("CH_ARM", Setup::CH_ARM)] >=
                *fp_.getParamPtr("CH_ARM_MIN", Setup::CH_ARM_MIN) &&
            f.channels[(int)*fp_.getParamPtr("CH_ARM", Setup::CH_ARM)] <=
                *fp_.getParamPtr("CH_ARM_MAX", Setup::CH_ARM_MAX);
        bool hhOn = f.channels[(int)*fp_.getParamPtr("CH_HH", Setup::CH_HH)] >=
                        *fp_.getParamPtr("CH_HH_MIN", Setup::CH_HH_MIN) &&
                    f.channels[(int)*fp_.getParamPtr("CH_HH", Setup::CH_HH)] <=
                        *fp_.getParamPtr("CH_HH_MAX", Setup::CH_HH_MAX);
        int rotOff =
            f.channels[(int)*fp_.getParamPtr("CH_ROTATE", Setup::CH_ROTATE)] -
            Setup::CH_ROTATE_MID;
        bool centered = abs(rotOff) <= Setup::HH_STICK_DEADZONE;

        if (!(sensor_.isMagReady() && sensor_.isMpuReady() && hhOn && armed &&
                !f.failsafe))
        {
            if (active_)
            {
                active_ = false;
                pid_.reset();
            }
            return;
        }

        if (!active_)
        {
            targetHeading_ = sensor_.getHeading();
            pid_.reset();
            active_ = true;
            link_.printf("HH ON %.1f\n", targetHeading_);
        }

        float scale =
            *fp_.getParamPtr("HH_STICK_RATE_SCALE", Setup::HH_STICK_RATE_SCALE);
        float wGyro =
            *fp_.getParamPtr("HH_ROT_WEIGHT_GYRO", Setup::HH_ROT_WEIGHT_GYRO);
        float mGain = *fp_.getParamPtr(
            "HH_MAG_CORRECTION_GAIN", Setup::HH_MAG_CORRECTION_GAIN);
        float maxMR =
            *fp_.getParamPtr("HH_MAX_MAG_RATE", Setup::HH_MAX_MAG_RATE);
        float dt = *fp_.getParamPtr("HH_DT", Setup::HH_DT);

        if (!centered)
        {
            float desRate = -(float)rotOff / 800.0f * scale;
            targetHeading_ =
                wrap360(targetHeading_ +
                        (wGyro * sensor_.gyroZ() + (1 - wGyro) * desRate) * dt);
            pid_.reset();
        }
        else
        {
            float mErr = angleErr(targetHeading_, sensor_.getHeading());
            float mCmd = constrain(mErr * mGain, -maxMR, maxMR);
            targetHeading_ =
                wrap360(targetHeading_ + (sensor_.gyroZ() + mCmd) * dt);
            int corr = pid_.update(mCmd, sensor_.gyroZ());
            fp_.setParam<false>(
                "PWM_LE", *fp_.getParamPtr("PWM_LE", Setup::LR_OFF) - corr);
            fp_.setParam<false>(
                "PWM_RE", *fp_.getParamPtr("PWM_RE", Setup::LR_OFF) + corr);
        }
    }
};

// ----------------------------------------------------------------------------
// GPS Handler (UART1: RX=17, TX=18)
// ----------------------------------------------------------------------------
class GPSHandler
{
    HardwareSerial gpsSerial_;
    FirmwareParams& fp_;
    TinyGPSPlus gps_;

public:
    GPSHandler(FirmwareParams& fp) : gpsSerial_(1), fp_(fp) {}

    void begin()
    {
        gpsSerial_.begin(
            Setup::GPS_BAUD, SERIAL_8N1, Setup::GPS_RX, Setup::GPS_TX);
        gpsSerial_.setRxBufferSize(1024);
        DBG_PRINT("GPS OK (GPIO%d/%d @ %lu)\n",
            Setup::GPS_TX,
            Setup::GPS_RX,
            Setup::GPS_BAUD);
    }

    void update()
    {
        while (gpsSerial_.available())
        {
            char c = gpsSerial_.read();
            if (gps_.encode(c))
            {
                if (gps_.location.isValid())
                {
                    fp_.setParam<true>("GPS_LAT", gps_.location.lat());
                    fp_.setParam<true>("GPS_LON", gps_.location.lng());
                }
                if (gps_.date.isValid())
                {
                    fp_.setParam<true>("GPS_DAY", gps_.date.day());
                    fp_.setParam<true>("GPS_MONTH", gps_.date.month());
                    fp_.setParam<true>("GPS_YEAR", gps_.date.year());
                }
                if (gps_.time.isValid())
                {
                    fp_.setParam<true>("GPS_HOUR", gps_.time.hour());
                    fp_.setParam<true>("GPS_MIN", gps_.time.minute());
                    fp_.setParam<true>("GPS_SEC", gps_.time.second());
                }
                fp_.setParam<true>("GPS_SAT", gps_.satellites.value());
                fp_.setParam<true>("GPS_SPD", gps_.speed.kmph());
            }
        }
    }
};

// ----------------------------------------------------------------------------
// ESC Output
// ----------------------------------------------------------------------------
class EscOutput : public IChannelObserver
{
    Servo me_, le_, re_;
    FirmwareParams& fp_;

public:
    EscOutput(FirmwareParams& f) : fp_(f) {}
    void begin()
    {
        me_.attach(Setup::ESC_MAIN, Setup::PPM_MIN, Setup::PPM_MAX);
        le_.attach(Setup::ESC_LEFT, Setup::PPM_MIN, Setup::PPM_MAX);
        re_.attach(Setup::ESC_RIGHT, Setup::PPM_MIN, Setup::PPM_MAX);
        me_.writeMicroseconds(Setup::MAIN_OFF);
        le_.writeMicroseconds(Setup::LR_OFF);
        re_.writeMicroseconds(Setup::LR_OFF);
    }
    void onChannelsUpdate(const SbusFrame& f) override
    {
        bool armed = f.channels[Setup::CH_ARM] >=
                         *fp_.getParamPtr("CH_ARM_MIN", Setup::CH_ARM_MIN) &&
                     f.channels[Setup::CH_ARM] <=
                         *fp_.getParamPtr("CH_ARM_MAX", Setup::CH_ARM_MAX);
        if (f.failsafe || !armed)
        {
            fp_.setParam<false>("PWM_ME", Setup::MAIN_OFF);
            fp_.setParam<false>("PWM_LE", Setup::LR_OFF);
            fp_.setParam<false>("PWM_RE", Setup::LR_OFF);
        }
        fp_.setParam<true>("PWM_LE",
            constrain(*fp_.getParamPtr("PWM_LE", Setup::LR_OFF),
                Setup::LR_OFF,
                Setup::LR_FULL));
        fp_.setParam<true>("PWM_RE",
            constrain(*fp_.getParamPtr("PWM_RE", Setup::LR_OFF),
                Setup::LR_OFF,
                Setup::LR_FULL));

        me_.writeMicroseconds(*fp_.getParamPtr("PWM_ME", Setup::MAIN_OFF));
        le_.writeMicroseconds(*fp_.getParamPtr("PWM_LE", Setup::LR_OFF));
        re_.writeMicroseconds(*fp_.getParamPtr("PWM_RE", Setup::LR_OFF));
    }
};

// ----------------------------------------------------------------------------
// Globals
// ----------------------------------------------------------------------------
WiFiLink wifiLink;
FirmwareParams fwParams(wifiLink);

// MAVLink handler ЗАКОММЕНТИРОВАН
// #if MAVLINK_ENABLED
// MavlinkHandler mavlinkHandler(fwParams);
// #endif

HeadingSensor headingSensor(fwParams);
BayEnvironmentSensor frontEnv(
    Setup::DHT_FRONT_BAY, fwParams, "TEMP_FRONT", "HUM_FRONT");
BayEnvironmentSensor rearEnv(
    Setup::DHT_REAR_BAY, fwParams, "TEMP_REAR", "HUM_REAR");
ChannelHandler channelHandler;
ThrottleMixer throttleMixer(fwParams);
RotationMixer rotationMixer(fwParams);
HeadingHoldController hhCtrl(headingSensor, wifiLink, fwParams);
GPSHandler gpsHandler(fwParams);
EscOutput escOutput(fwParams);

void setup()
{
    Serial.begin(115200); // UART0 всегда свободен для отладки
    
    DBG_PRINTLN("=== BOOT ===");
    DBG_PRINTLN("Mode: DEBUG (MAVLink disabled)");
    DBG_PRINTLN("UART0: Free for Serial Monitor");
    DBG_PRINTLN("UART1: GPS (GPIO17/18)");
    DBG_PRINTLN("UART2: SBUS (GPIO16/26)");

    fwParams.loadFromFS();
    delay(1000);
    wifiLink.begin(Setup::WIFI_NAME, Setup::WIFI_PASSWORD);
    Wire.begin(Setup::I2C_SDA, Setup::I2C_SCL);

    if (headingSensor.begin())
        DBG_PRINTLN("MPU6050 OK");
    else
        DBG_PRINTLN("MPU6050 FAIL");

    frontEnv.begin();
    rearEnv.begin();
    channelHandler.begin();
    gpsHandler.begin();

    // MAVLink НЕ инициализируется
    // #if MAVLINK_ENABLED
    // mavlinkHandler.begin();
    // #endif

    escOutput.begin();
    channelHandler.subscribe(&throttleMixer);
    channelHandler.subscribe(&rotationMixer);
    channelHandler.subscribe(&hhCtrl);
    channelHandler.subscribe(&escOutput);

    DBG_PRINTLN("Ready.");
    delay(5000);
}

void loop()
{
    if (wifiLink.poll(fwParams.getAllMutable()))
        fwParams.markDirty();
    headingSensor.update();
    gpsHandler.update();
    
    // MAVLink НЕ обновляется
    // #if MAVLINK_ENABLED
    // mavlinkHandler.update();
    // #endif
    
    frontEnv.update();
    rearEnv.update();
    if (fwParams.isDirty() &&
        millis() - fwParams.lastSaveTime() >= Setup::SAVE_INTERVAL_MS)
        fwParams.saveToFS();
    channelHandler.update();
}
