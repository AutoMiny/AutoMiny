// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include <PacketSerial.h>
#include <Wire.h>

#define NUMPIXELS 21

#define LED_PIN 6
#define DIR_PIN 4
#define SERVO_PIN 10
#define MOTOR_SPEED_PIN 11
#define ENCODER_PIN 3
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define SERVO_FEEDBACK_MOTOR_PIN 0
#define NEWLED_PIN 13
#define BATTERY_PIN A6
#define ENABLE_PIN 7
#define HEARTBEAT_TIMEOUT 100
#define UPDATE_RATE 10

#define REFERENCE_VOLTAGE 5.2 //Default reference on Teensy is 3.3V
#define R1 3300.0
#define R2 1490.0
#define VOLTAGE_BUFFER_SIZE 128

#define VOLTAGE_GOOD 14.8
#define VOLTAGE_BAD 13.0
#define VOLTAGE_SHUTDOWN 12.8

int ledState = HIGH;                // ledState used to set the LED
unsigned long previousMillis = 0;   // will store last time LED was updated
bool powered = false;

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
Servo myservo; // create servo object to control a servo
int servo_pw = 1550;    // variable to set the angle of servo motor
int last_pw = 0;
bool servo_initialized = false;
volatile uint8_t ticks = 0;

int8_t direction_motor = 1;

uint16_t voltageBuffer[VOLTAGE_BUFFER_SIZE] = {0};
int voltageIndex = 0;
int16_t currentSpeed = 0;
uint8_t voltageCycle = 0;

unsigned long lastHeartbeat = 0;
unsigned long lastUpdate = 0;

PacketSerial packetSerial;

enum class MessageType :uint8_t {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    SPEED_CMD,
    STEERING_CMD,
    LED_CMD,
    STEERING_ANGLE,
    TICKS,
    IMU,
    VOLTAGE,
    HEARTBEAT,
    IMU_CALIBRATION
};

enum class VoltageStatus {
    DISABLED,
    BAD,
    WARN,
    GOOD
};

enum class Direction {
    NONE,
    REVERSE,
    FORWARD
};

VoltageStatus currentVoltageStatus = VoltageStatus::DISABLED;
Direction currentDirection = Direction::NONE;

void onLedCommand(const char* cmd_msg);
void onSteeringCommand(const int16_t cmd_msg);
void onSpeedCommand(const int16_t cmd_msg);

void encoder() {
    ticks++;
}


// ================================================================
// ===               SUBSCRIBERS                                ===
// ================================================================
void onSteeringCommand(const int16_t cmd_msg) {
    // scale it to use it with the servo (value between 0 and 180)
    servo_pw = cmd_msg;

    if (last_pw != servo_pw) {
        myservo.writeMicroseconds(servo_pw);
    }

    if (!servo_initialized) {
        // attaches the servo on pin 9 to the servo object
        myservo.attach(SERVO_PIN);
        servo_initialized = true;
    
        if (last_pw != servo_pw) {
            myservo.writeMicroseconds(servo_pw);
        }
    }
}


void displayReverseLed() {
    if (currentDirection == Direction::REVERSE) {
        return;
    }

    pixels.setBrightness(16);

    pixels.setPixelColor(2, 255, 255, 255);
    pixels.setPixelColor(3, 255, 255, 255);
    pixels.setPixelColor(4, 255, 255, 255);
    pixels.setPixelColor(6, 255, 255, 255);
    pixels.setPixelColor(7, 255, 255, 255);
    pixels.setPixelColor(8, 255, 255, 255);

    pixels.setPixelColor(12, 0, 0, 0);
    pixels.setPixelColor(13, 0, 0, 0);
    pixels.setPixelColor(14, 0, 0, 0);
    pixels.setPixelColor(17, 0, 0, 0);
    pixels.setPixelColor(18, 0, 0, 0);
    pixels.setPixelColor(19, 0, 0, 0);

    pixels.show();

    currentDirection = Direction::REVERSE;
}

void displayForwardLed() {
    if (currentDirection == Direction::FORWARD) {
        return;
    }

    pixels.setBrightness(16);

    pixels.setPixelColor(2, 0, 0, 0);
    pixels.setPixelColor(3, 0, 0, 0);
    pixels.setPixelColor(4, 0, 0, 0);
    pixels.setPixelColor(6, 0, 0, 0);
    pixels.setPixelColor(7, 0, 0, 0);
    pixels.setPixelColor(8, 0, 0, 0);

    pixels.setPixelColor(12, 255, 255, 255);
    pixels.setPixelColor(13, 255, 255, 255);
    pixels.setPixelColor(14, 255, 255, 255);
    pixels.setPixelColor(17, 255, 255, 255);
    pixels.setPixelColor(18, 255, 255, 255);
    pixels.setPixelColor(19, 255, 255, 255);
    pixels.show();

    currentDirection = Direction::FORWARD;
}

void disableDirectionLed() {
    if (currentDirection == Direction::NONE) {
        return;
    }

    pixels.setBrightness(16);

    pixels.setPixelColor(2, 0, 0, 0);
    pixels.setPixelColor(3, 0, 0, 0);
    pixels.setPixelColor(4, 0, 0, 0);
    pixels.setPixelColor(6, 0, 0, 0);
    pixels.setPixelColor(7, 0, 0, 0);
    pixels.setPixelColor(8, 0, 0, 0);

    pixels.setPixelColor(12, 0, 0, 0);
    pixels.setPixelColor(13, 0, 0, 0);
    pixels.setPixelColor(14, 0, 0, 0);
    pixels.setPixelColor(17, 0, 0, 0);
    pixels.setPixelColor(18, 0, 0, 0);
    pixels.setPixelColor(19, 0, 0, 0);

    pixels.show();

    currentDirection = Direction::NONE;
}

void onSpeedCommand(const int16_t cmd_msg) {
    int16_t motor_val = cmd_msg / 4;

    if (abs(motor_val) > 255) {
        motor_val = 255;
    }

    if (currentSpeed == motor_val) {
        return;
    }
    currentSpeed = motor_val;

    uint8_t servo_val = (uint8_t) abs(motor_val);

    // if speed is set to 0 we keep the old direction
    // and just do nothing but set the val
    // else the speed direction might get inversed
    if (motor_val < 0) {
        digitalWrite(DIR_PIN, HIGH);
        direction_motor = -1;
        displayReverseLed();
    } else if (motor_val > 0) {
        digitalWrite(DIR_PIN, LOW);
        direction_motor = 1;
        displayForwardLed();
    }

    if (servo_val < 15) {
        servo_val = 15;
        disableDirectionLed();
    }

    analogWrite(MOTOR_SPEED_PIN, servo_val);
}

void onLedCommand(const char* cmd_msg) {
    pixels.setBrightness(16);
    if (strcmp_P(cmd_msg, PSTR("left")) == 0) {
        pixels.setPixelColor(0, pixels.Color(255, 80, 0)); //yellow
        pixels.setPixelColor(1, pixels.Color(255, 80, 0)); //yellow
        pixels.setPixelColor(20, pixels.Color(255, 80, 0)); //yellow
    } else if (strcmp_P(cmd_msg, PSTR("right")) == 0) {
        pixels.setPixelColor(9, pixels.Color(255, 80, 0)); //yellow
        pixels.setPixelColor(10, pixels.Color(255, 80, 0)); //yellow
        pixels.setPixelColor(11, pixels.Color(255, 80, 0)); //yellow
    } else if (strcmp_P(cmd_msg, PSTR("brake")) == 0) {
        pixels.setPixelColor(0, pixels.Color(255, 0, 0)); //red
        pixels.setPixelColor(1, pixels.Color(255, 0, 0)); //red
        pixels.setPixelColor(9, pixels.Color(255, 0, 0)); //red
        pixels.setPixelColor(10, pixels.Color(255, 0, 0)); //red
    } else if (strcmp_P(cmd_msg, PSTR("disable")) == 0) {
        pixels.setPixelColor(0, pixels.Color(0, 0, 0));
        pixels.setPixelColor(1, pixels.Color(0, 0, 0));
        pixels.setPixelColor(9, pixels.Color(0, 0, 0));
        pixels.setPixelColor(10, pixels.Color(0, 0, 0));
        pixels.setPixelColor(11, pixels.Color(0, 0, 0));
        pixels.setPixelColor(20, pixels.Color(0, 0, 0));
    }
    pixels.show(); // This sends the updated pixel color to the hardware.
}

void displayVoltageGoodLed() {
    if (currentVoltageStatus == VoltageStatus::GOOD) {
        return;
    }

    pixels.setBrightness(16);
    pixels.setPixelColor(5, 0, 255, 0);
    pixels.setPixelColor(15, 0, 255, 0);
    pixels.setPixelColor(16, 0, 255, 0);
    pixels.show();

    currentVoltageStatus = VoltageStatus::GOOD;
}

void displayVoltageWarningLed() {
    if (currentVoltageStatus == VoltageStatus::WARN) {
        return;
    }
    pixels.setBrightness(16);
    pixels.setPixelColor(5, 255, 255, 0);
    pixels.setPixelColor(15, 255, 255, 0);
    pixels.setPixelColor(16, 255, 255, 0);
    pixels.show();

    currentVoltageStatus = VoltageStatus::WARN;
}

void displayVoltageBadLed() {
    if (currentVoltageStatus == VoltageStatus::BAD) {
        return;
    }
    pixels.setBrightness(16);
    pixels.setPixelColor(5, 255, 0, 0);
    pixels.setPixelColor(15, 255, 0, 0);
    pixels.setPixelColor(16, 255, 0, 0);
    pixels.show();

    currentVoltageStatus = VoltageStatus::BAD;

}

void disableLed() {
    if (currentVoltageStatus == VoltageStatus::DISABLED) {
        return;
    }

    pixels.clear();
    pixels.show();

    currentVoltageStatus = VoltageStatus::DISABLED;
}


int readEEPROMInt(int addr) {
    byte low, high;
    low=EEPROM.read(addr);
    high=EEPROM.read(addr+1);
    return low + ((high << 8)&0xFF00);
}

void onPacketReceived(const uint8_t* message, size_t size)
{
    auto type = static_cast<MessageType>(message[0]);

    switch(type) {
        case MessageType::DEBUG:break;
        case MessageType::INFO:break;
        case MessageType::WARN:break;
        case MessageType::ERROR:break;
        case MessageType::STEERING_ANGLE:break;
        case MessageType::TICKS:break;
        case MessageType::IMU:break;
        case MessageType::VOLTAGE:break;
        case MessageType::HEARTBEAT:break;

        case MessageType::SPEED_CMD:
            int16_t speed;
            memcpy(&speed, &message[1], sizeof(int16_t));
            onSpeedCommand(speed);
            break;
        case MessageType::STEERING_CMD:
            int16_t steering;
            memcpy(&steering, &message[1], sizeof(int16_t));
            onSteeringCommand(steering);
            break;
        case MessageType::IMU_CALIBRATION:
            break;

        case MessageType::LED_CMD:
            char cmd[size];
            memcpy(&cmd, &message[1], size);
            onLedCommand(cmd);
            break;
    }

    lastHeartbeat = millis();
}

void log(MessageType type, const __FlashStringHelper *str) {
    uint8_t size = 1 + strlen_P((PGM_P)str) + 1;
    uint8_t buf[size];
    buf[0] = (uint8_t)type;
    strcpy_P(&buf[1], (PGM_P)str);
    packetSerial.send(buf, size);
}

void log(MessageType type, char *str) {
    uint8_t size = 1 + strlen(str) + 1;
    uint8_t buf[size];
    buf[0] = (uint8_t)type;
    strcpy(&buf[1], str);
    packetSerial.send(buf, size);
}

void logerror(const __FlashStringHelper *str) {
    log(MessageType::ERROR, str);
}

void logerror(char *str) {
    log(MessageType::ERROR, str);
}

void loginfo(const __FlashStringHelper *str) {
    log(MessageType::INFO, str);
}

void loginfo(char *str) {
    log(MessageType::INFO, str);
}

void logdebug(const __FlashStringHelper *str) {
    log(MessageType::DEBUG, str);
}

void logdebug(char *str) {
    log(MessageType::DEBUG, str);
}

void logwarn(const __FlashStringHelper *str) {
    log(MessageType::WARN, str);
}

void logwarn(char *str) {
    log(MessageType::WARN, str);
}

void sendVoltage(float voltage) {
    uint8_t size = 1 + sizeof(voltage);
    uint8_t buf[size];
    buf[0] = (uint8_t)MessageType::VOLTAGE;
    memcpy(&buf[1], &voltage, sizeof(voltage));
    packetSerial.send(buf, size);
}

void sendTicks(uint8_t ticks) {
    uint8_t size = 1 + sizeof(ticks);
    uint8_t buf[size];
    buf[0] = (uint8_t)MessageType::TICKS;
    memcpy(&buf[1], &ticks, sizeof(ticks));
    packetSerial.send(buf, size);
}

void sendSteeringAngle(uint16_t steeringAngle) {
    uint8_t size = 1 + sizeof(steeringAngle);
    uint8_t buf[size];
    buf[0] = (uint8_t)MessageType::STEERING_ANGLE;
    memcpy(&buf[1], &steeringAngle, sizeof(steeringAngle));
    packetSerial.send(buf, size);
}

void setup() {
    packetSerial.begin(115200);
    packetSerial.setStream(&Serial);
    packetSerial.setPacketHandler(&onPacketReceived);

    pinMode(MOTOR_SPEED_PIN, OUTPUT);
    pinMode(BATTERY_PIN, INPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENCODER_PIN, INPUT);
    pinMode(SERVO_FEEDBACK_MOTOR_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoder, CHANGE);
    pixels.begin(); // This initializes the NeoPixel library.
    //Voltmeter
    pinMode(NEWLED_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);
}

float meanVoltage() {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < VOLTAGE_BUFFER_SIZE; i++) {
        sum += voltageBuffer[i];
    }

    float avg = sum / VOLTAGE_BUFFER_SIZE;
    float vout = (avg * REFERENCE_VOLTAGE) / 1024.0;
    return vout / (R2/(R1+R2));
}

void turnOnCar() {
    if (!powered) {
        digitalWrite(ENABLE_PIN, HIGH);
    }

    powered = true;
}

void turnOffCar() {
    if (powered) {
        digitalWrite(ENABLE_PIN, LOW);
    }
    powered = false;
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // wait for MPU interrupt or extra packet(s) available
    //while (!mpuInterrupt && fifoCount < packetSize) {}
    unsigned long ms = millis();

    if (ms - lastHeartbeat > HEARTBEAT_TIMEOUT) {
        onSpeedCommand(0);
    }

    packetSerial.update();

    if (ms - lastUpdate >= UPDATE_RATE) {
        lastUpdate = ms;
        packetSerial.update();
        sendTicks(ticks);
        packetSerial.update();
        ticks = 0;
        uint16_t steeringAngle = analogRead(SERVO_FEEDBACK_MOTOR_PIN);
        steeringAngle = analogRead(SERVO_FEEDBACK_MOTOR_PIN);
        sendSteeringAngle((uint16_t)steeringAngle);
        packetSerial.update();

        /***Voltmeter**/
        voltageCycle++;
        if (voltageCycle >= 2) {
            uint16_t vol = analogRead(BATTERY_PIN);
            vol = analogRead(BATTERY_PIN);
            voltageBuffer[voltageIndex++] = vol;
            if (voltageIndex >= VOLTAGE_BUFFER_SIZE) {
                voltageIndex = 0;
            }
            voltageCycle = 0;
        }

        float voltage = meanVoltage();

        if (ms > 2600 && voltage > VOLTAGE_SHUTDOWN){
            turnOnCar();
        } else if (voltage <= VOLTAGE_SHUTDOWN && powered) {
            // This means the car was already on and the voltage dropped below 12.8
            // Empty the voltage measurements so that the car does not turn on immediately again because the voltage went up again
            memset(voltageBuffer, 0, sizeof(voltageBuffer));
            turnOffCar();
        } else {
            turnOffCar();
        }

        if (voltage > VOLTAGE_GOOD) {
            displayVoltageGoodLed();
        } else if (voltage <= VOLTAGE_GOOD && voltage >= VOLTAGE_BAD) {
            displayVoltageWarningLed();
        } else if (voltage < VOLTAGE_BAD) {
            displayVoltageBadLed();
        }
        packetSerial.update();

        sendVoltage(voltage);
        packetSerial.update();
    }
        
}


