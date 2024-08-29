#include<Wire.h>
#include<Servo.h>
#include<Adafruit_MPU6050.h>

int updateInt = 0;

Servo elevon1;
Servo elevon2;
int angle;
int angle2;
int yangle = 0;
const byte switch_pin = 12; 
const byte elev = 10;
int elev_neutral = 90;
int elev_max = 70;


const int MPU_addr = 0x68;

int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;

float Accel_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.14159264;

float PID, error, pitch_error, prev_roll_error, prev_pitch_error;
float P = 0;
float I = 0;
float D = 0;

double kp_roll = 100;
double ki_roll = 0.0;
double kd_roll = 0;
double kp_pitch = 100;
double ki_pitch = 0.0;
double kd_pitch = 0;

float desired_roll = 0;
float desired_pitch = 0;
int output_roll = 0;
int output_roll_neg = 0;
int output_pitch = 0;


void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  elevon1.attach(9);
  elevon2.attach(10);
  time = millis(); 
}


void loop() {
  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev)/1000;

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);

  Acc_rawX = Wire.read()<<8|Wire.read();
  Acc_rawY = Wire.read()<<8|Wire.read();
  Acc_rawZ = Wire.read()<<8|Wire.read();

  Accel_angle[0] = atan( (Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2))) *rad_to_deg;
  Accel_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2)+ pow((Acc_rawZ/16384.0),2)))*rad_to_deg;

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,4,true);

  Gyr_rawX = Wire.read()<<8|Wire.read();
  Gyr_rawY = Wire.read()<<8|Wire.read();

  Gyro_angle[0] = Gyr_rawX/131.0;
  Gyro_angle[1] = Gyr_rawY/131.0;

  Total_angle[0] = 0.98 * (Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Accel_angle[0];
  Total_angle[1] = 0.98 * (Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Accel_angle[1];

  if((updateInt%500)==0){
    Serial.print("roll :");
    Serial.print(Total_angle[0]);
    Serial.print("\n");
    Serial.print("output :");
    Serial.print(output_roll);
    Serial.print("\n");
    Serial.print("P values correspodingly");
    Serial.print(P);
  }
  updateInt++;

  error = Total_angle[0] - desired_roll;

  P = kp_roll*error;
  I += (ki_roll*error);
  D = kd_roll*((error-prev_roll_error)/elapsedTime);

  PID = P + I + D;

  if (PID> elev_max)
  {
    PID = elev_max;
  }
  if (PID < -elev_max)
  {
    PID = -elev_max;
  }
  output_roll = elev_neutral + PID;
  output_roll_neg = elev_neutral - PID;

  elevon1.write(output_roll);
  elevon2.write(output_roll);
  prev_roll_error = error;

if ((Total_angle[1] < -30)  || (Total_angle[1] > 30)) {

    Serial.print("pitch :");
    Serial.print(Total_angle[1]);
    Serial.print("\n");
    Serial.print("output :");
    Serial.print(output_pitch);
    Serial.print("\n");
 

    pitch_error = Total_angle[1] - desired_pitch;

    P = kp_pitch*error;
    I += (ki_pitch*error);
    D = kd_pitch*((error-prev_pitch_error)/elapsedTime);
    PID = P + I + D;

    if (PID> elev_max)
    {
      PID = elev_max;
    }
    if (PID < -elev_max)
    {
      PID = -elev_max;
    }
    output_pitch = elev_neutral + PID;
    int output_pitch_neg = elev_neutral - PID;
    elevon1.write(output_pitch);
    elevon2.write(output_pitch_neg);
    prev_pitch_error = pitch_error;
  }
}




