#include <AsyncPulseIn.h>
#include <adafruit-sht31.h>
#include <SparkFunSX1509.h>

#define SERIAL_SPEED 115200
#define LOOP_FREQUENCY 100

#define FAN_PWM_FREQUENCY_HZ 25000

#define PIN_LED D7
#define PIN_FAN_1_PWM A4
#define PIN_FAN_2_PWM A5
#define PIN_FAN_1_TACH D3
#define PIN_FAN_2_TACH D2
#define PIN_MOTOR_PWM A7

#define SHT31_ALT_ADDRESS 0x44

uint16_t fan_speed_min_rpm = 2500;
uint16_t fan_speed_max_rpm = 12000;

uint8_t fan_pwm_resolution = 10;
uint16_t fan_pwm_range = 2^fan_pwm_resolution - 1;

uint8_t motor_pwm_resolution = 10;
uint16_t motor_pwm_frequency_hz = 10000;

float set_speed = 5000;
uint16_t set_pwm = 500;

constexpr uint kNumberFans = 2;
constexpr uint kNumberTempHumid = 1;

struct sensorDataStruct {
  float fan_rpm[kNumberFans];
  float temp_C[kNumberTempHumid];
  float humid_rel[kNumberTempHumid];
} sensor_data;

// --------------- END CONFIG DATA ---------------

// GPIO list.
std::array<uint8_t, 10> gpio_list = {6, 7, 5, 4, 3, 2, 1, 0, 14, 15};

// Pulse objects used for counting input PWM.
AsyncPulseInPulseInfo pulse_in_info_fan;
std::array<AsyncPulseIn*, kNumberFans> pulse_in_fan;

// SHT31 objects used for temperature and humidity.
std::array<Adafruit_SHT31*, kNumberTempHumid> temp_humid;

// GPIO multiplexer object.
SX1509 gpio;

// Flag to indicate if the board LED is on;
bool led_set = false;
uint loop_counter = 0;

void setup() {
  // Start the serial connection.
  Serial.begin(SERIAL_SPEED);

  // Initialize the board LED.
  pinMode(PIN_LED, OUTPUT);

  pulse_in_fan[0] = new AsyncPulseIn(PIN_FAN_1_TACH, HIGH);
  pulse_in_fan[1] = new AsyncPulseIn(PIN_FAN_2_TACH, HIGH);

  pinMode(PIN_FAN_1_TACH, INPUT_PULLUP);
  pinMode(PIN_FAN_2_TACH, INPUT_PULLUP);

  for (auto& this_temp_humid : temp_humid) {
    this_temp_humid = new Adafruit_SHT31();
  }

  // Set fan PWM pinmodes.
  pinMode(PIN_FAN_1_PWM, OUTPUT);
  pinMode(PIN_FAN_2_PWM, OUTPUT);

  // Set motor PWM pinmode.
  pinMode(PIN_MOTOR_PWM, OUTPUT);

  // Set fan PWM resolution to be able to handle 25kHz.
  analogWriteResolution(PIN_FAN_1_PWM, fan_pwm_resolution);
  analogWriteResolution(PIN_FAN_2_PWM, fan_pwm_resolution);

  // Set motor PWM resolution.
  analogWriteResolution(PIN_MOTOR_PWM, motor_pwm_resolution);

  // Set fan PWM dutycycle to 0% of 25kHz
  analogWrite(PIN_FAN_1_PWM, set_pwm, FAN_PWM_FREQUENCY_HZ);
  analogWrite(PIN_FAN_2_PWM, set_pwm, FAN_PWM_FREQUENCY_HZ);

  analogWrite(PIN_MOTOR_PWM, 700, motor_pwm_frequency_hz);

  // Start the GPIO multiplexer.
  if (gpio.begin()) {
    Serial.printf("GPIO begin: SUCCESS\n");
  } else {
    Serial.printf("GPIO begin: FAIL\n");
  }

  for (auto this_gpio : gpio_list) {
    gpio.pinMode(this_gpio, OUTPUT);
    gpio.digitalWrite(this_gpio, LOW);
  }

  // Start the temperature / humidity sensors.
  uint i = 0;
  for (auto& this_temp_humid : temp_humid) {
    gpio.digitalWrite(gpio_list[i], HIGH);
    this_temp_humid->begin(SHT31_ALT_ADDRESS);
    gpio.digitalWrite(gpio_list[i], LOW);
    i++;
  }
}

void loop() {
  uint i = 0;
  for (auto& this_temp_humid : temp_humid) {
    gpio.digitalWrite(gpio_list[i], HIGH);
    sensor_data.temp_C[0] = this_temp_humid->readTemperature();
    sensor_data.humid_rel[0] = this_temp_humid->readHumidity();
    gpio.digitalWrite(gpio_list[i], LOW);
    i++;
  }

  // Calculate the fan speeds.
  i = 0;
  for (auto& this_pulse_in_fan : pulse_in_fan) {
    this_pulse_in_fan->getNextPulse(pulse_in_info_fan);
    // Convert pulse width in microseconds to pulses per minute, which is the
    // same as rpm.
    if (pulse_in_info_fan.widthMicros > 0) {
      sensor_data.fan_rpm[i] = 60.0 * 1e6 / (float)pulse_in_info_fan.widthMicros;
    } else {
      sensor_data.fan_rpm[i] = 0.0;
    }
    i++;
  }

  loop_counter = loop_counter++ % LOOP_FREQUENCY;

  if (loop_counter == 0) {
    Serial.printf("Fan 1 RPM: %.0f, fan 2 RPM: %.0f\n", sensor_data.fan_rpm[0], sensor_data.fan_rpm[1]);
    Serial.printf("Temperature %.2f, humidity: %.2f\n", sensor_data.temp_C[0], sensor_data.humid_rel[0]);

    String message = "";
    for (int i = 0; i < kNumberFans; i++) {
      message.concat(String::format("f%d:%f\n", i+1, sensor_data.fan_rpm[i]));
    }
    message.concat(String::format("t%d:%f\n", 1, sensor_data.temp_C[0]));
    message.concat(String::format("h%d:%f\n", 1, sensor_data.humid_rel[0]));

    Particle.publish("sensors", message, PRIVATE);
  }

  if (led_set) {
    digitalWrite(PIN_LED, RESET);
    led_set = false;
  } else {
    digitalWrite(PIN_LED, SET);
    led_set = true;
  }

  delay(1000 / LOOP_FREQUENCY);
}
