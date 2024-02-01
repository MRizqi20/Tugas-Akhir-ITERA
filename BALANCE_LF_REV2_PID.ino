#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>

#define MIN_ABS_SPEED 10

// Define pin assignments
int motorL1 = 14;
int motorL2 = 27;
int motorR1 = 13;
int motorR2 = 12;

MPU6050 mpu;

bool dmpReady = false;
volatile bool mpuInterrupt = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

double originalSetpoint = 174.00;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

double Kp = 17;
double Kd = 0.9;
double Ki = 250;
PID pidBalancing(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// Line follower sensor pins
int S0 = 34;
int S1 = 35;
int S2 = 32;
int S3 = 33;
int SENSOR_PIN[] = {S0, S1, S2, S3};
int sensorValues[4];

// PID for line following
double lineFollowingSetpoint = 1500;  // Adjust as needed
double lineFollowingInput, lineFollowingOutput;
double lineFollowingKp = 11;  // Adjust as needed
double lineFollowingKd = 0.2;  // Adjust as needed
double lineFollowingKi = 100;  // Adjust as needed
PID pidLineFollowing(&lineFollowingInput, &lineFollowingOutput, &lineFollowingSetpoint, lineFollowingKp, lineFollowingKi, lineFollowingKd, DIRECT);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(19), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    pidBalancing.SetMode(AUTOMATIC);
    pidBalancing.SetSampleTime(10);
    pidBalancing.SetOutputLimits(-150, 150);

    pidLineFollowing.SetMode(AUTOMATIC);
    pidLineFollowing.SetSampleTime(10);
    pidLineFollowing.SetOutputLimits(-150, 150);
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // Setup line follower sensor pins
  for (int i = 0; i < 4; i++) {
    pinMode(SENSOR_PIN[i], INPUT);
  }
}

void loop() {
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
      break;
    }
  }

  if (mpuInterrupt) {
    mpuInterrupt = false;
    fifoCount = mpu.getFIFOCount();

    if (fifoCount >= packetSize) {
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      input = ypr[1] * 180 / M_PI + 180;

      // Line follower logic
      int linePosition = baca_garis();

      // Print line position and PWM values to Serial Monitor
      Serial.print("Line Position: ");
      Serial.println(linePosition);
      Serial.print("Balancing PWM: ");
      Serial.println(output);
      Serial.print("Line Following PWM: ");
      Serial.println(lineFollowingOutput);

      pidBalancing.Compute();
      pidLineFollowing.Compute();

      // Hitung output motor berdasarkan self-balancing dan line following
      int balancingOutput = output;

      // Perhitungan line following correction
      int lineFollowingCorrection = map(linePosition, -3000, 3000, -150, 150);

      // Berikan bobot pada masing-masing output
      int motorL = balancingOutput - lineFollowingCorrection;
      int motorR = balancingOutput + lineFollowingCorrection;

      // Limit kecepatan motor
      motorL = constrain(motorL, -150, 150);
      motorR = constrain(motorR, -150, 150);

      // Panggil fungsi motor untuk menggerakkan kedua motor
      moveMotors(motorL, motorR);
    }
  }
}

void moveMotors(int motorL, int motorR) {
  int leftSpeed = constrain(abs(motorL), MIN_ABS_SPEED, 150);
  int rightSpeed = constrain(abs(motorR), MIN_ABS_SPEED, 150);

  if (motorL > 0) {
    analogWrite(motorL1, leftSpeed);
    analogWrite(motorL2, 0);
  } else {
    analogWrite(motorL1, 0);
    analogWrite(motorL2, leftSpeed);
  }

  if (motorR > 0) {
    analogWrite(motorR1, rightSpeed);
    analogWrite(motorR2, 0);
  } else {
    analogWrite(motorR1, 0);
    analogWrite(motorR2, rightSpeed);
  }
}

int baca_garis() {
  for (int i = 0; i < 4; i++) {
    sensorValues[i] = analogRead(SENSOR_PIN[i]);
  }

  int linePosition = 0;

  for (int i = 0; i < 4; i++) {
    // Menggunakan nilai tengah 1500 sebagai rujukan
    linePosition += (sensorValues[i] - 1500) * (i - 1);
  }

  return linePosition;
}

void dmpDataReady() {
  mpuInterrupt = true;
}
