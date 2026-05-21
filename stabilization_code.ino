#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>

MPU6050 mpu;

Servo servoRoll;
Servo servoPitch;
Servo servoYaw;

#define ROLL_SERVO_PIN   3
#define PITCH_SERVO_PIN  5
#define YAW_SERVO_PIN    6

int16_t ax, ay, az;
int16_t gx, gy, gz;

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;

float roll, pitch, yaw;

unsigned long previousTime;
float elapsedTime;

float setRoll = 0;
float setPitch = 0;
float setYaw = 0;

float pidRoll, pidPitch, pidYaw;

float errorRoll, previousErrorRoll;
float errorPitch, previousErrorPitch;
float errorYaw, previousErrorYaw;

float integralRoll = 0;
float integralPitch = 0;
float integralYaw = 0;

float kp = 3.0;
float ki = 0.02;
float kd = 1.2;

int centerRoll = 90;
int centerPitch = 90;
int centerYaw = 90;

void setup() {

  Serial.begin(115200);

  Wire.begin();

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  servoRoll.attach(ROLL_SERVO_PIN);
  servoPitch.attach(PITCH_SERVO_PIN);
  servoYaw.attach(YAW_SERVO_PIN);

  servoRoll.write(centerRoll);
  servoPitch.write(centerPitch);
  servoYaw.write(centerYaw);

  previousTime = millis();

  Serial.println("3 Axis Stabilization Started");
}

void loop() {

  unsigned long currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  AccX = ax / 16384.0;
  AccY = ay / 16384.0;
  AccZ = az / 16384.0;

  GyroX = gx / 131.0;
  GyroY = gy / 131.0;
  GyroZ = gz / 131.0;

  float accRoll =
      atan2(AccY, AccZ) * 180 / PI;

  float accPitch =
      atan2(-AccX,
      sqrt(AccY * AccY + AccZ * AccZ))
      * 180 / PI;

  roll =
      0.96 * (roll + GyroX * elapsedTime)
      + 0.04 * accRoll;

  pitch =
      0.96 * (pitch + GyroY * elapsedTime)
      + 0.04 * accPitch;

  yaw += GyroZ * elapsedTime;

  errorRoll = setRoll - roll;

  integralRoll += errorRoll * elapsedTime;

  float derivativeRoll =
      (errorRoll - previousErrorRoll)
      / elapsedTime;

  pidRoll =
      (kp * errorRoll)
      + (ki * integralRoll)
      + (kd * derivativeRoll);

  previousErrorRoll = errorRoll;

  errorPitch = setPitch - pitch;

  integralPitch += errorPitch * elapsedTime;

  float derivativePitch =
      (errorPitch - previousErrorPitch)
      / elapsedTime;

  pidPitch =
      (kp * errorPitch)
      + (ki * integralPitch)
      + (kd * derivativePitch);

  previousErrorPitch = errorPitch;

  errorYaw = setYaw - yaw;

  integralYaw += errorYaw * elapsedTime;

  float derivativeYaw =
      (errorYaw - previousErrorYaw)
      / elapsedTime;

  pidYaw =
      (kp * errorYaw)
      + (ki * integralYaw)
      + (kd * derivativeYaw);

  previousErrorYaw = errorYaw;


  int rollServo =
      constrain(centerRoll + pidRoll, 0, 180);

  int pitchServo =
      constrain(centerPitch + pidPitch, 0, 180);

  int yawServo =
      constrain(centerYaw + pidYaw, 0, 180);

  servoRoll.write(rollServo);
  servoPitch.write(pitchServo);
  servoYaw.write(yawServo);

  Serial.print("Roll: ");
  Serial.print(roll);

  Serial.print(" Pitch: ");
  Serial.print(pitch);

  Serial.print(" Yaw: ");
  Serial.print(yaw);

  Serial.print(" | Servo R: ");
  Serial.print(rollServo);

  Serial.print(" P: ");
  Serial.print(pitchServo);

  Serial.print(" Y: ");
  Serial.println(yawServo);

  delay(5);
}
