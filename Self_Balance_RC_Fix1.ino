#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>
#include <BluetoothSerial.h>
#include <EEPROM.h>

#define MIN_ABS_SPEED 30
#define EEPROM_ADDR_SETPOINT_FORWARD 0
#define EEPROM_ADDR_SETPOINT_BACKWARD 4

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

double originalSetpoint = 172.75;
double originalSetpointForward = 174.05;
double originalSetpointBackward = 169.55;
double setpointForward = originalSetpointForward;
double setpointBackward = originalSetpointBackward;
double movingAngleOffset = 0.1; 
double input, output;

double Kp = 21;
double Kd = 2.1;
double Ki = 500;
PID pid(&input, &output, &originalSetpoint, Kp, Ki, Kd, DIRECT);

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);

  // Inisialisasi Bluetooth
  SerialBT.begin("ESP32_BT_Robot"); // Nama Bluetooth: RobotSelfBalancing

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

    // Membaca nilai setpoint dari EEPROM
    EEPROM.get(EEPROM_ADDR_SETPOINT_FORWARD, setpointForward);
    EEPROM.get(EEPROM_ADDR_SETPOINT_BACKWARD, setpointBackward);

    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop() {
  // Baca data yang diterima melalui Bluetooth
  while (SerialBT.available()) {
    char command = SerialBT.read();
    executeCommand(command);
  }

  // Jalankan PID untuk menjaga keseimbangan robot
  if (dmpReady) {
    // Baca data dari MPU6050
    while (!mpuInterrupt && fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        break;
      } 
    }

    // Saat data tersedia dari MPU6050, lakukan pemrosesan PID
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

        // Jalankan PID
        pid.Compute();
        
        // Gerakkan motor berdasarkan output PID
        moveMotors(output, output);
        Serial.println(output); // Tampilkan nilai output PID ke Serial Monitor
      }
    }
  }
}

// Fungsi untuk mengeksekusi perintah yang diterima melalui Bluetooth
void executeCommand(char command) {
  switch(command) {
    case 'F':
      originalSetpoint = originalSetpointForward;
      setpointBackward = originalSetpointBackward;
      Serial.println("Maju"); // Tampilkan pesan bahwa robot bergerak maju
      break;
    case 'B':
      originalSetpoint= originalSetpointBackward;
      setpointBackward = originalSetpointForward;
      Serial.println("Mundur"); // Tampilkan pesan bahwa robot bergerak mundur
      break;
    case 'L':
        turnLeft();
        Serial.println("Belok Kiri"); // Tampilkan pesan bahwa robot berbelok ke kiri
      break;
    case 'R':
      turnRight();
       Serial.println("Belok Kanan"); // Tampilkan pesan bahwa robot berbelok ke kanan
      break;
           default:
      // Jika tidak ada tombol perintah Bluetooth yang ditekan, kembalikan ke setpoint original
      originalSetpoint = 172.75;
      break;
  }
}

void turnLeft() {
  // Kode untuk membuat robot berbelok ke kiri
  analogWrite(motorL1, 255);
  analogWrite(motorL2, 0);
  analogWrite(motorR1, 0);
  analogWrite(motorR2, 255);
}

void turnRight() {
  // Kode untuk membuat robot berbelok ke kanan
  analogWrite(motorL1, 0);
  analogWrite(motorL2, 255);
  analogWrite(motorR1, 255);
  analogWrite(motorR2, 0);
}

// Implementasi fungsi-fungsi untuk menggerakkan robot
void moveMotors(int motorL, int motorR) {
  if (motorL > 0) {
    analogWrite(motorL1, motorL);
    analogWrite(motorL2, 0);
  } else if (motorL < 0) {
    analogWrite(motorL1, 0);
    analogWrite(motorL2, abs(motorL));
  } else {
    analogWrite(motorL1, 0);
    analogWrite(motorL2, 0);
  }

  if (motorR > 0) {
    analogWrite(motorR1, motorR);
    analogWrite(motorR2, 0);
  } else if (motorR < 0) {
    analogWrite(motorR1, 0);
    analogWrite(motorR2, abs(motorR));
  } else {
    analogWrite(motorR1, 0);
    analogWrite(motorR2, 0);
  }
}

// Fungsi yang dipanggil ketika data siap dari MPU6050
void dmpDataReady() {
  mpuInterrupt = true;
}
