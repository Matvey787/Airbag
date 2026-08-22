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

// ----------------------------------------------------------------------------
// Default Setting if something goes wrong with json
// ----------------------------------------------------------------------------
namespace Setup
{
constexpr const char* WIFI_NAME		= "ESP32-Debug";
constexpr const char* WIFI_PASSWORD = "12345678";
constexpr uint16_t WIFI_PORT		= 8080;

constexpr int ESC_MAIN	= 13;
constexpr int ESC_LEFT	= 14;
constexpr int ESC_RIGHT = 15;

constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;

constexpr int SBUS_RX = 16;
constexpr int SBUS_TX = 26;

constexpr int DHT_FRONT_BAY = 23;
constexpr int DHT_REAR_BAY	= 19;

constexpr int PPM_MIN	= 1000;
constexpr int PPM_MAX	= 2000;
constexpr int LR_OFF	= 1000;
constexpr int LR_FULL	= 2000;
constexpr int MAIN_OFF	= 1500;
constexpr int MAIN_FULL = 2000;

constexpr int CH_MAIN_ENGINE	 = 10;
constexpr int CH_MAIN_ENGINE_MIN = 172;
constexpr int CH_MAIN_ENGINE_MAX = 1100;
constexpr int CH_FORWARD		 = 2;
constexpr int CH_FORWARD_MIN	 = 172;
constexpr int CH_FORWARD_MAX	 = 1800;
constexpr int CH_ROTATE			 = 0;
constexpr int CH_ROTATE_MIN		 = 172;
constexpr int CH_ROTATE_MID		 = 1000;
constexpr int CH_ROTATE_MAX		 = 1800;
constexpr int CH_ARM			 = 4;
constexpr int CH_ARM_MIN		 = 900;
constexpr int CH_ARM_MAX		 = 1100;
constexpr int CH_HH				 = 8;
constexpr int CH_HH_MIN			 = 900;
constexpr int CH_HH_MAX			 = 1100;
constexpr int CH_COUNT			 = 16;

constexpr uint8_t MAG_MAX_REJECT_STREAK = 5;
constexpr float MAG_SPIKE_THRESHOLD		= 45.0f;
constexpr float MAG_DT_MAX				= 0.1f;

constexpr float PID_INTEGRAL_LIMIT	= 200.0f;
constexpr int PID_MAX_OUTPUT		= 300;
constexpr int PID_MAX_STEP_PER_LOOP = 15;
constexpr float PID_D_FILTER_ALPHA	= 0.2f;
constexpr float PID_P				= 0.8f;
constexpr float PID_I				= 0.03f;
constexpr float PID_D				= 0.05f;

constexpr float HH_MAG_CORRECTION_GAIN = 0.08f;
constexpr float HH_ROT_WEIGHT_GYRO	   = 0.8;
constexpr float HH_MAX_MAG_RATE		   = 15.0f;
constexpr float HH_STICK_RATE_SCALE	   = 90.0f;
constexpr int HH_STICK_DEADZONE		   = 50;
constexpr float HH_DT				   = 0.02f;

constexpr unsigned long ENV_POLL_INTERVAL_MS = 2000;
constexpr unsigned long SAVE_INTERVAL_MS	 = 5000;

constexpr float GPS_LAT = 0;
constexpr float GPS_LON = 0;
constexpr int GPS_DAY	= 0;
constexpr int GPS_MONTH = 0;
constexpr int GPS_YEAR	= 0;
constexpr int GPS_HOUR	= 0;
constexpr int GPS_MIN	= 0;
constexpr int GPS_SEC	= 0;
constexpr int GPS_SAT	= 0;
constexpr float GPS_SPD = 0;
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
		printf("START_CONFIG\n");
		for (auto const& [key, val] : params)
		{
			printf("SET %s %.4f\n", key.c_str(), val);
		}
		printf("END_CONFIG\n");
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
			auto firstSpace	 = cmd.indexOf(' ');
			auto secondSpace = cmd.indexOf(' ', firstSpace + 1);
			if (secondSpace > 0)
			{
				String keyStr	= cmd.substring(firstSpace + 1, secondSpace);
				float val		= cmd.substring(secondSpace + 1).toFloat();
				std::string key = keyStr.c_str();
				params[key]		= val;
				return true;
			}
		}
		else if (cmd == "HI_ESP")
		{
			printf("HI_GUI\n");
		}
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
		printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
		printf("TCP server on port %u\n", port);
	}

	bool poll(std::unordered_map<std::string, float>& params)
	{
		bool modified = false;
		if (server_.hasClient())
		{
			if (client_)
				client_.stop();
			client_ = server_.available();
			printf("TCP client connected\n");
		}
		if (client_ && !client_.connected())
		{
			client_.stop();
			printf("TCP client disconnected\n");
		}
		if (client_ && client_.available())
		{
			String line = client_.readStringUntil('\n');
			line.trim();
			if (line.length() > 0)
			{
				if (handleCommand(line, params))
					modified = true;
			}
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
	bool dirty_					  = false;
	unsigned long lastSaveTimeMs_ = 0;
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
		{
			link_.printf("SET %s %.4f\n", key.c_str(), val);
		}
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
			Serial.println("ERROR: Failed to mount LittleFS");
			return;
		}
		File file = LittleFS.open("/firmware_vars.json", "r");
		if (!file)
		{
			Serial.println("ERROR: Failed to open firmware_vars.json");
			return;
		}
		DynamicJsonDocument doc(16384);
		DeserializationError error = deserializeJson(doc, file);
		if (error)
		{
			Serial.print("ERROR: JSON parsing failed: ");
			Serial.println(error.c_str());
			file.close();
			return;
		}
		params_.clear();
		JsonArray array = doc.as<JsonArray>();
		for (JsonObject obj : array)
		{
			for (JsonPair kv : obj)
			{
				std::string key		= kv.key().c_str();
				JsonObject innerObj = kv.value();
				if (innerObj.containsKey("val"))
				{
					params_[key] = innerObj["val"].as<float>();
				}
			}
		}
		file.close();
		Serial.println("INFO: Parameters loaded from JSON.");
	}

	void saveToFS()
	{
		if (!dirty_)
			return;
		DynamicJsonDocument doc(16384);
		JsonArray array = doc.to<JsonArray>();
		for (const auto& [key, val] : params_)
		{
			JsonObject obj	 = array.createNestedObject();
			JsonObject inner = obj.createNestedObject(key);
			inner["val"]	 = val;
		}
		File file = LittleFS.open("/firmware_vars.json", "w");
		if (!file)
		{
			Serial.println(
				"ERROR: Failed to open firmware_vars.json for writing");
			return;
		}
		if (serializeJson(doc, file) == 0)
		{
			Serial.println("ERROR: Failed to write JSON to file");
		}
		else
		{
			Serial.printf("INFO: Parameters saved to LittleFS (%d entries)\n",
				params_.size());
		}
		file.close();
		dirty_			= false;
		lastSaveTimeMs_ = millis();
	}
};

WiFiLink wifiLink;
FirmwareParams fwParams(wifiLink);

// ----------------------------------------------------------------------------
// Heading Sensor
// ----------------------------------------------------------------------------
class HeadingSensor
{
	Adafruit_MPU6050 mpu_;
	QMC5883LCompass mag_;
	FirmwareParams& fp_;
	bool mpuReady_ = false;
	bool magReady_ = false;
	float lastValidHeading_{ 0 };
	unsigned long lastHeadingTime_{ 0 };
	uint8_t rejectStreak_{ 0 };

	float tiltCompensatedHeading()
	{
		mag_.read();
		float mx = mag_.getX();
		float my = mag_.getY();
		float mz = mag_.getZ();

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
		float Xh	   = mx * cosf(pitchRad) + mz * sinf(pitchRad);
		float Yh = mx * sinf(rollRad) * sinf(pitchRad) + my * cosf(rollRad) -
				   mz * sinf(rollRad) * cosf(pitchRad);
		float heading = atan2f(-Yh, Xh) * RAD_TO_DEG;
		if (heading < 0)
			heading += 360.0f;
		return heading;
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
		magReady_		  = true;
		lastValidHeading_ = 0;
		lastHeadingTime_  = millis();
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
		float h = mpuReady_ ? tiltCompensatedHeading() : [&]()
		{
			mag_.read();
			float az = mag_.getAzimuth();
			return az < 0 ? az + 360.0f : az;
		}();

		unsigned long now = millis();
		float dt		  = (now - lastHeadingTime_) / 1000.0f;
		lastHeadingTime_  = now;
		if (dt <= 0)
			dt = 0.02f;

		float diff = h - lastValidHeading_;
		while (diff > 180.0f)
			diff -= 360.0f;
		while (diff < -180.0f)
			diff += 360.0f;

		float spikeThreshold =
			*fp_.getParamPtr("MAG_SPIKE_THRESHOLD", Setup::MAG_SPIKE_THRESHOLD);
		float dtMax			= *fp_.getParamPtr("MAG_DT_MAX", Setup::MAG_DT_MAX);
		bool looksLikeSpike = (fabsf(diff) > spikeThreshold) && (dt < dtMax);

		if (looksLikeSpike)
		{
			rejectStreak_++;
			uint8_t maxReject = (uint8_t)*fp_.getParamPtr(
				"MAG_MAX_REJECT_STREAK", Setup::MAG_MAX_REJECT_STREAK);
			if (rejectStreak_ < maxReject)
			{
				fp_.setParam<true>("YAW", lastValidHeading_);
				return lastValidHeading_;
			}
		}
		rejectStreak_	  = 0;
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
	const std::string tempKey_;
	const std::string humKey_;
	unsigned long lastPollMs_ = 0;
	float temperature_		  = NAN;
	float humidity_			  = NAN;

public:
	BayEnvironmentSensor(uint8_t pin,
		FirmwareParams& fp,
		const std::string& tempKey,
		const std::string& humKey,
		uint8_t type = DHT11) :
		dht_(pin, type), fp_(fp), tempKey_(tempKey), humKey_(humKey)
	{}

	void begin() { dht_.begin(); }

	void update()
	{
		unsigned long now = millis();
		if (now - lastPollMs_ < Setup::ENV_POLL_INTERVAL_MS)
			return;

		lastPollMs_ = now;

		float t = dht_.readTemperature();
		float h = dht_.readHumidity();

		if (isnan(t) || isnan(h))
			return;

		temperature_ = t;
		humidity_	 = h;

		fp_.setParam<true>(tempKey_, temperature_);
		fp_.setParam<true>(humKey_, humidity_);
	}

	float temperature() const { return temperature_; }
	float humidity() const { return humidity_; }
};

// ----------------------------------------------------------------------------
// Channels Handler
// ----------------------------------------------------------------------------
struct SbusFrame
{
	uint16_t channels[Setup::CH_COUNT];
	bool failsafe;
};

class IChannelObserver
{
public:
	virtual ~IChannelObserver()							  = default;
	virtual void onChannelsUpdate(const SbusFrame& frame) = 0;
};

class ChannelBus
{
	std::vector<IChannelObserver*> observers_;

public:
	void subscribe(IChannelObserver* obs) { observers_.push_back(obs); }
	void publish(const SbusFrame& frame)
	{
		for (auto* obs : observers_)
			obs->onChannelsUpdate(frame);
	}
};

class ChannelHandler
{
	bfs::SbusRx sbus_rx_;
	ChannelBus channel_bus_;

public:
	ChannelHandler() :
		sbus_rx_(&Serial2, Setup::SBUS_RX, Setup::SBUS_TX, true, false)
	{}
	void begin() { sbus_rx_.Begin(); }
	void subscribe(IChannelObserver* obs) { channel_bus_.subscribe(obs); }
	void update()
	{
		if (sbus_rx_.Read())
		{
			bfs::SbusData data = sbus_rx_.data();
			SbusFrame frame;
			frame.failsafe = data.failsafe;
			for (int i = 0; i < Setup::CH_COUNT; i++)
				frame.channels[i] = data.ch[i];
			channel_bus_.publish(frame);
		}
	}
};

// ----------------------------------------------------------------------------
// PID Controller
// ----------------------------------------------------------------------------
class RatePid
{
	FirmwareParams& fp_;
	float integral_			= 0.0f;
	float prevGyro_			= 0.0f;
	float filteredD_		= 0.0f;
	int prevOutput_			= 0;
	unsigned long lastTime_ = 0;

public:
	explicit RatePid(FirmwareParams& fp) : fp_(fp) {}

	void reset()
	{
		integral_	= 0.0f;
		prevGyro_	= 0.0f;
		filteredD_	= 0.0f;
		prevOutput_ = 0;
		lastTime_	= millis();
	}

	int update(float targetRate, float currentGyro)
	{
		unsigned long now = millis();
		float dt		  = (now - lastTime_) / 1000.0f;
		lastTime_		  = now;
		if (dt <= 0.0f || dt > 0.1f)
			dt = 0.02f;

		float kp = *fp_.getParamPtr("PID_P", Setup::PID_P);
		float ki = *fp_.getParamPtr("PID_I", Setup::PID_I);
		float kd = *fp_.getParamPtr("PID_D", Setup::PID_D);
		float intLimit =
			*fp_.getParamPtr("PID_INTEGRAL_LIMIT", Setup::PID_INTEGRAL_LIMIT);
		float maxOut =
			*fp_.getParamPtr("PID_MAX_OUTPUT", Setup::PID_MAX_OUTPUT);
		float maxStep = *fp_.getParamPtr(
			"PID_MAX_STEP_PER_LOOP", Setup::PID_MAX_STEP_PER_LOOP);
		float dAlpha =
			*fp_.getParamPtr("PID_D_FILTER_ALPHA", Setup::PID_D_FILTER_ALPHA);

		float error = targetRate - currentGyro;
		float pTerm = kp * error;
		integral_ += error * dt;
		integral_	= constrain(integral_, -intLimit, intLimit);
		float iTerm = ki * integral_;

		float gyroDeriv = (currentGyro - prevGyro_) / dt;
		prevGyro_		= currentGyro;
		filteredD_		= (1.0f - dAlpha) * filteredD_ + dAlpha * gyroDeriv;
		float dTerm		= -kd * filteredD_;

		float output = pTerm + iTerm + dTerm;
		int raw	  = constrain((int)roundf(output), -(int)maxOut, (int)maxOut);
		int delta = constrain(raw - prevOutput_, -(int)maxStep, (int)maxStep);
		prevOutput_ += delta;
		return prevOutput_;
	}
};

// ----------------------------------------------------------------------------
// Mixers
// ----------------------------------------------------------------------------
class ThrottleMixer : public IChannelObserver
{
	FirmwareParams& fp_;

public:
	explicit ThrottleMixer(FirmwareParams& fp) : fp_(fp) {}

	void onChannelsUpdate(const SbusFrame& f) override
	{
		int chMain =
			(int)*fp_.getParamPtr("CH_MAIN_ENGINE", Setup::CH_MAIN_ENGINE);
		fp_.setParam<true>("CH_MAIN_ENGINE_VAL", f.channels[chMain]);
		fp_.setParam<true>("PWM_ME",
			map(f.channels[chMain],
				*fp_.getParamPtr(
					"CH_MAIN_ENGINE_MIN", Setup::CH_MAIN_ENGINE_MIN),
				*fp_.getParamPtr(
					"CH_MAIN_ENGINE_MAX", Setup::CH_MAIN_ENGINE_MAX),
				Setup::MAIN_OFF,
				Setup::MAIN_FULL));

		int chFwd = (int)*fp_.getParamPtr("CH_FORWARD", Setup::CH_FORWARD);
		fp_.setParam<true>("CH_FORWARD_VAL", f.channels[chFwd]);
		int fwd = map(f.channels[chFwd],
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
	explicit RotationMixer(FirmwareParams& fp) : fp_(fp) {}

	void onChannelsUpdate(const SbusFrame& f) override
	{
		int chRot = (int)*fp_.getParamPtr("CH_ROTATE", Setup::CH_ROTATE);
		fp_.setParam<true>("CH_ROTATE_VAL", f.channels[chRot]);
		int offset = f.channels[chRot] - Setup::CH_ROTATE_MID;
		int baseLE = *fp_.getParamPtr("PWM_LE", Setup::LR_OFF);
		int baseRE = *fp_.getParamPtr("PWM_RE", Setup::LR_OFF);

		if (offset < -50)
		{
			int ppm = map(abs(offset), 50, 800, 0, 400);
			fp_.setParam<false>("PWM_RE", baseRE + ppm);
			fp_.setParam<false>("PWM_LE", baseLE - ppm);
		}
		else if (offset > 50)
		{
			int ppm = map(abs(offset), 50, 800, 0, 400);
			fp_.setParam<false>("PWM_RE", baseRE - ppm);
			fp_.setParam<false>("PWM_LE", baseLE + ppm);
		}
	}
};

// ----------------------------------------------------------------------------
// Heading Hold
// ----------------------------------------------------------------------------
class HeadingHoldController : public IChannelObserver
{
	HeadingSensor& sensor_;
	WiFiLink& link_;
	FirmwareParams& fp_;
	RatePid pid_;
	float targetHeading_ = 0.0f;
	bool active_		 = false;

	static float wrap360(float v)
	{
		while (v < 0)
			v += 360;
		while (v >= 360)
			v -= 360;
		return v;
	}
	static float angleError(float t, float c)
	{
		float e = t - c;
		while (e > 180)
			e -= 360;
		while (e < -180)
			e += 360;
		return e;
	}

public:
	HeadingHoldController(HeadingSensor& s, WiFiLink& l, FirmwareParams& fp) :
		sensor_(s), link_(l), fp_(fp), pid_(fp)
	{}

	void onChannelsUpdate(const SbusFrame& f) override
	{
		int chArm = (int)*fp_.getParamPtr("CH_ARM", Setup::CH_ARM);
		fp_.setParam<true>("CH_ARM_VAL", f.channels[chArm]);
		bool armed = (f.channels[chArm] >=
						  *fp_.getParamPtr("CH_ARM_MIN", Setup::CH_ARM_MIN) &&
					  f.channels[chArm] <=
						  *fp_.getParamPtr("CH_ARM_MAX", Setup::CH_ARM_MAX));

		int rotOffset =
			f.channels[(int)*fp_.getParamPtr("CH_ROTATE", Setup::CH_ROTATE)] -
			Setup::CH_ROTATE_MID;
		bool stickCentered = (abs(rotOffset) <= Setup::HH_STICK_DEADZONE);

		int chHH = (int)*fp_.getParamPtr("CH_HH", Setup::CH_HH);
		fp_.setParam<true>("CH_HH_VAL", f.channels[chHH]);
		bool hhActivated = (f.channels[chHH] >= *fp_.getParamPtr("CH_HH_MIN",
													Setup::CH_HH_MIN) &&
							f.channels[chHH] <= *fp_.getParamPtr("CH_HH_MAX",
													Setup::CH_HH_MAX));

		bool requested = sensor_.isMagReady() && sensor_.isMpuReady() &&
						 hhActivated && armed && !f.failsafe;
		if (!requested)
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
			link_.printf("HH: ACTIVATED at %.1f deg\n", targetHeading_);
		}

		float stickScale =
			*fp_.getParamPtr("HH_STICK_RATE_SCALE", Setup::HH_STICK_RATE_SCALE);
		float rotWeight =
			*fp_.getParamPtr("HH_ROT_WEIGHT_GYRO", Setup::HH_ROT_WEIGHT_GYRO);
		float magGain = *fp_.getParamPtr(
			"HH_MAG_CORRECTION_GAIN", Setup::HH_MAG_CORRECTION_GAIN);
		float maxMagRate =
			*fp_.getParamPtr("HH_MAX_MAG_RATE", Setup::HH_MAX_MAG_RATE);
		float hhDt = *fp_.getParamPtr("HH_DT", Setup::HH_DT);

		if (!stickCentered)
		{
			float desRate = -(float)rotOffset / 800.0f * stickScale;
			float blended =
				rotWeight * sensor_.gyroZ() + (1 - rotWeight) * desRate;
			targetHeading_ = wrap360(targetHeading_ + blended * hhDt);
			pid_.reset();
			link_.printf("HH: MANUAL desRate:%.1f gyro:%.1f tgt:%.1f\n",
				desRate,
				sensor_.gyroZ(),
				targetHeading_);
		}
		else
		{
			float magErr = angleError(targetHeading_, sensor_.getHeading());
			float magRateCmd =
				constrain(magErr * magGain, -maxMagRate, maxMagRate);
			targetHeading_ = wrap360(targetHeading_ + sensor_.gyroZ() * hhDt);
			targetHeading_ = wrap360(targetHeading_ + magRateCmd * hhDt);
			int corr	   = pid_.update(magRateCmd, sensor_.gyroZ());
			fp_.setParam<false>(
				"PWM_LE", *fp_.getParamPtr("PWM_LE", Setup::LR_OFF) - corr);
			fp_.setParam<false>(
				"PWM_RE", *fp_.getParamPtr("PWM_RE", Setup::LR_OFF) + corr);
			link_.printf(
				"HH: HOLD magErr:%.1f magRate:%.1f corr:%d LE:%d RE:%d\n",
				magErr,
				magRateCmd,
				corr,
				*fp_.getParamPtr("PWM_LE", Setup::LR_OFF),
				*fp_.getParamPtr("PWM_RE", Setup::LR_OFF));
		}
	}
};

// ----------------------------------------------------------------------------
// GPS Handler
// ----------------------------------------------------------------------------
class GPSHandler
{
	HardwareSerial gpsSerial_;
	WiFiLink& link_;
	FirmwareParams& fp_;
	TinyGPSPlus gps_;

public:
	GPSHandler(WiFiLink& link, FirmwareParams& fp) :
		gpsSerial_(1), link_(link), fp_(fp)
	{}

	void begin() { gpsSerial_.begin(9600, SERIAL_8N1, 18, 17); }

	void update()
	{

		Serial.println("GPS HERE");
		while (gpsSerial_.available())
		{

			Serial.println("GPS HERE2");
			char c = gpsSerial_.read();
			if (gps_.encode(c))
			{
				if (gps_.location.isValid())
				{
					Serial.println("GPS SENDDDDDDDDDDDD");
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

	void applyFailsafe(const SbusFrame& f)
	{
		int chArm  = Setup::CH_ARM;
		bool armed = (f.channels[chArm] >=
						  *fp_.getParamPtr("CH_ARM_MIN", Setup::CH_ARM_MIN) &&
					  f.channels[chArm] <=
						  *fp_.getParamPtr("CH_ARM_MAX", Setup::CH_ARM_MAX));
		if (f.failsafe || !armed)
		{
			fp_.setParam<false>("PWM_ME", Setup::MAIN_OFF);
			fp_.setParam<false>("PWM_LE", Setup::LR_OFF);
			fp_.setParam<false>("PWM_RE", Setup::LR_OFF);
		}
	}

	void applyLimits()
	{
		fp_.setParam<true>("PWM_RE",
			constrain(*fp_.getParamPtr("PWM_RE", Setup::LR_OFF),
				Setup::LR_OFF,
				Setup::LR_FULL));
		fp_.setParam<true>("PWM_LE",
			constrain(*fp_.getParamPtr("PWM_LE", Setup::LR_OFF),
				Setup::LR_OFF,
				Setup::LR_FULL));
	}

	void sendToEsc()
	{
		me_.writeMicroseconds(*fp_.getParamPtr("PWM_ME", Setup::MAIN_OFF));
		le_.writeMicroseconds(*fp_.getParamPtr("PWM_LE", Setup::LR_OFF));
		re_.writeMicroseconds(*fp_.getParamPtr("PWM_RE", Setup::LR_OFF));
	}

public:
	explicit EscOutput(FirmwareParams& fp) : fp_(fp) {}

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
		applyFailsafe(f);
		applyLimits();
		sendToEsc();
	}
};


HeadingSensor headingSensor(fwParams);
BayEnvironmentSensor frontBayEnv(
	Setup::DHT_FRONT_BAY, fwParams, "TEMP_FRONT", "HUM_FRONT");
BayEnvironmentSensor rearBayEnv(
	Setup::DHT_REAR_BAY, fwParams, "TEMP_REAR", "HUM_REAR");
ChannelHandler channelHandler;
ThrottleMixer throttleMixer(fwParams);
RotationMixer rotationMixer(fwParams);
HeadingHoldController headingHold(headingSensor, wifiLink, fwParams);
GPSHandler gpsHandler(wifiLink, fwParams);
EscOutput escOutput(fwParams);

unsigned long lastEnvPollMs = 0;

void setup()
{
	Serial.begin(115200);

	fwParams.loadFromFS();
	delay(1000);

	wifiLink.begin(Setup::WIFI_NAME, Setup::WIFI_PASSWORD);
	Wire.begin(Setup::I2C_SDA, Setup::I2C_SCL);

	if (headingSensor.begin())
		wifiLink.printf("MPU6050 OK\n");
	else
		wifiLink.printf("MPU6050 NOT FOUND!\n");

	frontBayEnv.begin();
	rearBayEnv.begin();
	channelHandler.begin();
	gpsHandler.begin();

	wifiLink.printf("ESP32 + SBUS Receiver + GPS started\n");
	escOutput.begin();

	channelHandler.subscribe(&throttleMixer);
	channelHandler.subscribe(&rotationMixer);
	channelHandler.subscribe(&headingHold);
	channelHandler.subscribe(&escOutput);

	wifiLink.printf("System ready. Waiting 5s...\n");
	delay(5000);
}

void loop()
{
	if (wifiLink.poll(fwParams.getAllMutable()))
	{
		fwParams.markDirty();
	}

	headingSensor.update();
	gpsHandler.update();

	frontBayEnv.update();
	rearBayEnv.update();

	if (fwParams.isDirty() &&
		(millis() - fwParams.lastSaveTime() >= Setup::SAVE_INTERVAL_MS))
	{
		fwParams.saveToFS();
	}

	channelHandler.update();
}