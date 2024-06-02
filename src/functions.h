// Libs
#include <Arduino.h>
#include <Wire.h>
#include <PWMServo.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SPIFlash.h>

// VARS

// Flashchip

// Pyros
int Pyro1 = 20;
int Pyro2 = 21;
int Pyro3 = 22;
int Pyro4 = 23;

// Buzzer
int Buzzer = 10;
bool buzzerOn = false;

int ttime = millis();
// BMP280 current barometric pressure
const float BarPressure = 1014;
// Servo
PWMServo servoSetOne;
PWMServo servoSetTwo;
int CommandAngle = 0;
double servoSetOneDefault = 0;
double servoSetTwoDefault = 0;


// BMP280
Adafruit_BMP280 bmp;
#define BMP_SCK (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS (10)

// MPU6050 IMU
Adafruit_MPU6050 mpu;
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int minVal = 265;
int maxVal = 402;

double x;
double y;
double z;
float ICmdX;
float ICmdZ;

// State
int currentState = 0;

// Default values
double DefaultX = 0;
double DefaultY = 0;
double DefaultZ = 0;
double startingAltitude = 0;

// Variables
int acceleration = 0;
int maxFinAngle = 5;
int abortAngle = 45;

// SD Card
SdFat SD;
SdFile dataFile;
const size_t TRANSFER_BUFFER_SIZE = 512;

const int chipSelect = 0; // Change to SD card pin

// Altitude
double CalibratedAltitude = 0;

// PID
// PID variables
float kp = 0.05; // Proportional gain
float ki = 0.05; // Integral gain
float kd = 0.001; // Derivative gain

float setpoint = 0.0; // Desired roll rate
float previous_error = 0.0;
float integral = 0.0;
float derivative = 0.0;
float output = 0.0;
float rollRateError = 0.0;

// Time variables
unsigned long previousTime = 0;
unsigned long currentTime;
unsigned long parachuteFireMillisTime = 0;
unsigned long timerMillis = 0;
unsigned long groundTimerMillis = 0;

float elapsedTime;

// Parachute
int parachutePyro = Pyro2;
int firedParachute = 0;

// Status Vars
double firstSampleAcc = 0;
double firstSampleAlt = 0;

// Status VAR
int status = 0;

void sdCardSetup() {
    if (!SD.begin(chipSelect, SPI_HALF_SPEED)) {
    SD.initErrorHalt();
  }
}

void pyroSetup()
{
    Serial.println("Pyro 1 HIGH!");
    tone(Buzzer, 1000);
    digitalWrite(Pyro1, HIGH);
    delay(1000);
    digitalWrite(Pyro1, LOW);
    noTone(Buzzer);
    delay(1000);
    Serial.println("Pyro 2 HIGH!");
    tone(Buzzer, 3000);
    digitalWrite(Pyro2, HIGH);
    delay(1000);
    digitalWrite(Pyro2, LOW);
    noTone(Buzzer);
    delay(1000);
    Serial.println("Pyro 3 HIGH!");
    tone(Buzzer, 5000);
    digitalWrite(Pyro3, HIGH);
    delay(1000);
    digitalWrite(Pyro3, LOW);
    noTone(Buzzer);
    delay(1000);
    Serial.println("Pyro 4 HIGH!");
    tone(Buzzer, 7000);
    digitalWrite(Pyro4, HIGH);
    delay(1000);
    digitalWrite(Pyro4, LOW);
    noTone(Buzzer);
    delay(1000);
    Serial.println("All pyros completed!");
}

void getRotData()
{
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    // MPU6050

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_addr, (uint8_t)14, (uint8_t) true); // Explicitly cast the arguments
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    int xAng = map(AcX, minVal, maxVal, -90, 90);
    int yAng = map(AcY, minVal, maxVal, -90, 90);
    int zAng = map(AcZ, minVal, maxVal, -90, 90);

    x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
    y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
    z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

    // Serial.print("X "), Serial.print(x), Serial.print(" Y "), Serial.print(y), Serial.print(" Z "), Serial.println(z);
    if (currentState < 3)
    {
        DefaultX = x, DefaultY = y, DefaultZ = z, CalibratedAltitude = bmp.readAltitude(BarPressure);
    }
}

void sensorTesting()
{
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    // Print IMU Values
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");
    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    rollRateError = g.gyro.z;
    Serial.println(" rad/s");
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_addr, (uint8_t)14, true);
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    int xAng = map(AcX, minVal, maxVal, -90, 90);
    int yAng = map(AcY, minVal, maxVal, -90, 90);
    int zAng = map(AcZ, minVal, maxVal, -90, 90);

    x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
    y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
    z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

    Serial.print("AngleX= ");
    Serial.println(x);

    Serial.print("AngleY= ");
    Serial.println(y);

    Serial.print("AngleZ= ");
    Serial.println(z);
    Serial.println("");

    Serial.println("Tested MPU6050 (IMU)");
    // BMP280
    Serial.println("Testing BMP280 (Barometer)");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(BarPressure)); /* Adjusted to local forecast! */
    Serial.println(" m");
}

void startUp() {
    Serial.begin(9600);
    // Tone to announce startup
    tone(Buzzer, 2000);
    delay(1000);
    status++;
    noTone(Buzzer);

    // Wire
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    // Main
    bmp.begin();
    mpu.begin();

    // Pyro Startup
    pinMode(Pyro1, OUTPUT);
    pinMode(Pyro2, OUTPUT);
    pinMode(Pyro3, OUTPUT);
    pinMode(Pyro4, OUTPUT);

    // Servo Startup
    servoSetOne.attach(3);
    servoSetTwo.attach(4);
    servoSetOneDefault = 90;//servoSetOne.read();
    servoSetTwoDefault = 90;//servoSetTwo.read();
    servoSetOne.write(servoSetOneDefault);
    servoSetTwo.write(servoSetTwoDefault);

    // Sensors
    sensorTesting();
    // Starting Values
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    startingAltitude = bmp.readAltitude(BarPressure);
    firstSampleAcc = a.acceleration.y;
    firstSampleAlt = startingAltitude;

    // SD Card Startup
    sdCardSetup();
    // Startup Finished
    delay(500);
    tone(Buzzer, 5000);
    delay(1000);
    noTone(Buzzer);
    Serial.println("Startup completed, all systems go, waiting for launch");
    status++;
} 

void fireParachute()
{
        if (parachuteFireMillisTime == 0)
        {
            parachuteFireMillisTime = millis();
            Serial.println("Firing parachute now");
            digitalWrite(parachutePyro, HIGH);
        } else if (millis() >= parachuteFireMillisTime + 3000 && firedParachute == 0)
        {
            digitalWrite(parachutePyro, LOW);
            Serial.println("Parachute fired");
            firedParachute = 1;
        }
}

void rollControl() {
    // Get current time
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) / 1000.0; // Convert to seconds

    // Read gyro data from MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float rollRate = (g.gyro.z - rollRateError) * 57.2958; // Convert raw gyro value to degrees/second

    // Print gyro data
    /*Serial.print("X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.print(", Z+: ");
    Serial.print(g.gyro.z - rollRateError);
    Serial.print(" rad/s, Roll Rate: ");
    Serial.print(rollRate);
    Serial.println(" °/s");*/

    // PID calculations
    float error = setpoint - rollRate;
    integral += error * elapsedTime;
    derivative = (error - previous_error) / elapsedTime;
    output = kp * error + ki * integral + kd * derivative;

    // Constrain output to servo range
    int servoOutputOne = constrain(output + servoSetOneDefault, servoSetOneDefault - maxFinAngle, servoSetOneDefault + maxFinAngle);
    int servoOutputTwo = constrain(output + servoSetTwoDefault, servoSetTwoDefault - maxFinAngle, servoSetTwoDefault + maxFinAngle);

    // Set servo positions
    servoSetOne.write(servoOutputOne);
    servoSetTwo.write(servoOutputTwo);
    /*Serial.print("Output: ");
    Serial.println(output);
    Serial.print("Servo1: ");
    Serial.println(servoOutputOne);
    Serial.print("Servo2: ");
    Serial.println(servoOutputTwo);
    Serial.println("------------");*/

    // Update previous values for next iteration
    previous_error = error;
    previousTime = currentTime;
    //delay(100);
}

void dataLog() {
    bool fileExists = SD.exists("flightData.txt");
    if (!dataFile.open("flightData.txt", O_RDWR | O_CREAT | O_AT_END)) {
        SD.errorPrint("opening flightData.txt for write failed");
    }

    if (!fileExists) {
        dataFile.print("Time,Acceleration_X,Acceleration_Y,Acceleration_Z,Rotation_X,Rotation_Y,Rotation_Z,Altitude,State,RollRateError,PID_Output,Servo1,Servo2,FiredParachute\n");
    }

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    double altitude = bmp.readAltitude(BarPressure);
    unsigned long currentMillis = millis();

    dataFile.print(currentMillis);
    dataFile.print(",");
    dataFile.print(a.acceleration.x);
    dataFile.print(",");
    dataFile.print(a.acceleration.y);
    dataFile.print(",");
    dataFile.print(a.acceleration.z);
    dataFile.print(",");
    dataFile.print(g.gyro.x);
    dataFile.print(",");
    dataFile.print(g.gyro.y);
    dataFile.print(",");
    dataFile.print(g.gyro.z);
    dataFile.print(",");
    dataFile.print(altitude);
    dataFile.print(",");
    dataFile.print(status);
    dataFile.print(",");
    dataFile.print(rollRateError);
    dataFile.print(",");
    dataFile.print(output);
    dataFile.print(",");
    dataFile.print(servoSetOne.read());
    dataFile.print(",");
    dataFile.print(servoSetTwo.read());
    dataFile.print(",");
    dataFile.print(firedParachute);
    dataFile.println();

    dataFile.close();
}



// State Detector
void update() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    // Ground
    if (status == 2) {
        if (millis() >= groundTimerMillis + 3000 || (millis() >= groundTimerMillis + 500 && buzzerOn)) {
            if (buzzerOn) {
                noTone(Buzzer);
                groundTimerMillis = millis();
                buzzerOn = false;
            } else {
                tone(Buzzer, 2000);
                groundTimerMillis = millis();
                buzzerOn = true;
                Serial.print(millis());
                Serial.println(" ms, all systems go");
                dataLog();
            }
        }
    }

    // Detect liftoff
    if (status==2) {
        if (millis() >= timerMillis + 200) {
            if (firstSampleAcc < a.acceleration.y - 9.81) {
                status++;
                Serial.println("Lift-off detected, all systems nominal");

            } else {
                firstSampleAcc = a.acceleration.y;
                timerMillis = millis();
            }
        }
    } else {
        firstSampleAcc = 0;
    }
    // Detect burnout
    if (status == 3) {
        if (millis() >= timerMillis + 200) {
            if (firstSampleAcc > a.acceleration.y) {
                status++;
                Serial.println("Burnout detected, all systems nominal");
                setpoint = 90.0;
                Serial.println("Starting rotation, setpoint: 90 °/s");
            } else {
                firstSampleAcc = a.acceleration.y;
                timerMillis = millis();
            }
        }
    }

    // Detect apogee
    if (status == 4) {
        rollControl();

        if (millis() >= timerMillis + 200) {
            if (bmp.readAltitude(BarPressure) < firstSampleAlt) {
                status++;
                Serial.println("Apogee detected, all systems nominal");
                setpoint = 0;
                Serial.println("Stopped rotation, setpoint: 0 °/s");
            } else {
                firstSampleAlt = bmp.readAltitude(BarPressure);
                timerMillis = millis();
            }
        }
    }
    if (status == 5 || status == 6) {
        fireParachute();
    }
    // Detect landing (optional)
    if (status == 5) {
        if (abs(bmp.readAltitude(BarPressure) - startingAltitude) < 10) {
            status++;
            Serial.println("Flight concluded, landed, all systems nominal");
            tone(Buzzer, 2000);
        }
    }
    if (status == 6) {
         if (millis() >= groundTimerMillis + 10000) {
            dataLog();
            groundTimerMillis = millis();
         }
    }
}

