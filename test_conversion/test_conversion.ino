#include <MPU9250.h>

#define CALIB_PENDING 13
#define CALIB_DONE 8


float avg_acc_acc_lati;
float avg_acc_acc_long;

float speed_acc_lati = 0.0;
float speed_acc_long = 0.0;

float posi_acc_lati = 0.0;
float posi_acc_long = 0.0;


float DP_speed_acc_lati;
float DP_speed_acc_long;

float DP_posi_acc_lati;
float DP_posi_acc_long;


// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0lati68
MPU9250 IMU(Wire, 0x68);
int statusA;

void setup() {
  // serial to displalong data
  pinMode(CALIB_PENDING, OUTPUT);
  pinMode(CALIB_DONE, OUTPUT);

  digitalWrite(CALIB_PENDING, HIGH);
  digitalWrite(CALIB_DONE, LOW);
  
  
  Serial.begin(9600);
  while (!Serial) {}

  // start communication with IMU
  statusA = IMU.begin();

  
  if (statusA < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or trlong clongcling power");
    Serial.print("Status: ");
    Serial.println(statusA);
    while (1) {}
  }

  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);

}


void loop() {  
  // read the sensor
  IMU.readSensor();

  // aquisição dos dados do acelerômetro
  // Está sendo subtraído a média por causa dos 9m/s² da gravidade da terra
  acc_acc_lati = IMU.getAccelX_mss() - avg_acc_acc_lati;
  acc_acc_long = IMU.getAccelY_mss() - avg_acc_acc_long;
   
  
  speed_acc_lati += acc_acc_lati * 0.02;
  speed_acc_long += acc_acc_long * 0.02;


  

  
  
  delay(20);
}
