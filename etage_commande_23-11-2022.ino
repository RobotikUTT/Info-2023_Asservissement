#include <MPU6050.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#define RX_Wheels_Serial       8
#define TX_Wheels_Serial       9
#define SDA_MPU6050_Serial       A4
#define SCL_MPU6050_Serial       A5
MPU6050 mpu;
unsigned long prevT = 0;
int dt = 10;
float xCons = 0;
float yCons = 0;
bool enMvmt = false;
bool endMove = false;

SoftwareSerial wheelsSerial(RX_Wheels_Serial, TX_Wheels_Serial);
String inputString = "";

void setup() {
  Serial.begin(500000);   // Démarrage port série
  Serial.setTimeout(1);   // Délai d'attente port série 1ms
  wheelsSerial.begin(9600);
  wheelsSerial.setTimeout(10);
  initMPU();

  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);

  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
  Serial.println(F("Initialisé"));
}

float phi = 0;
float prevPhi = 0

void loop() {
  CheckSerial();
  CheckWheelsSerial();
  if (millis() - prevT >= dt) {
    prevT = millis();
    getRot();
  }
}

void getRot() {
  phi += mpu.readNormalizeGyro().ZAxis * dt / 1000.;
}

void CheckWheelsSerial() { // Récupération des consignes via le port SWSerial (RX TX)
  if (wheelsSerial.available() > 0) { //pour recuperer des valeurs du moniteur
    //Serial.print("Reception consigne : ");
    int command = wheelsSerial.parseInt();
    //Serial.println(command);
    if (command == 0) {
      enMvmt = false;
      Ancrage(false);
      
    }
    if (command == 1) {  // Deplacement
      enMvmt = true;
      float poubelle = wheelsSerial.parseFloat();
    }
  }
}

void CheckSerial() { // Récupération des consignes
  if (Serial.available() > 0) { //pour recup des valeurs du moniteur
    Rotation(0, 0.5, PI);
    while (Serial.available() > 0) {
      Serial.parseFloat();
    }
  }
}

void Arret() {
  wheelsSerial.println((int)0);
}

void Deplacement(float x, float y) {
  wheelsSerial.print((String) "1" + " " +  x + " " + y);
  prevPhi = phi;
  endMove = false;
  Serial.println("Consigne 1 envoyée");
}

void Rotation(float xCentre, float yCentre, float angle) {
  wheelsSerial.print((String) "2" + " " +  xCentre + " " + yCentre + " " + angle);
  Serial.println("Consigne 2 envoyée");
}

void Ancrage(bool a){
  a ? wheelsSerial.println((String)3 + " " + 0) : wheelsSerial.println((String)3 + " " + 1);
}

//-------------------------------------------------------------------
//////////////////////////      MPU       ///////////////////////////
//-------------------------------------------------------------------

void initMPU() {
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.setAccelPowerOnDelay(MPU6050_DELAY_3MS);
  mpu.setIntFreeFallEnabled(false);
  mpu.setIntZeroMotionEnabled(false);
  mpu.setIntMotionEnabled(false);
  mpu.setDHPFMode(MPU6050_DHPF_5HZ);
  mpu.setMotionDetectionThreshold(2);
  mpu.setMotionDetectionDuration(5);
  mpu.setZeroMotionDetectionThreshold(4);
  mpu.setZeroMotionDetectionDuration(2);
  mpu.calibrateGyro();
  mpu.setThreshold(3);
  //checkMPUSettings();
}

void checkMPUSettings() {
  Serial.print(" * Sleep Mode:                ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  Serial.print(" * Motion Interrupt:     ");
  Serial.println(mpu.getIntMotionEnabled() ? "Enabled" : "Disabled");
  Serial.print(" * Zero Motion Interrupt:     ");
  Serial.println(mpu.getIntZeroMotionEnabled() ? "Enabled" : "Disabled");
  Serial.print(" * Free Fall Interrupt:       ");
  Serial.println(mpu.getIntFreeFallEnabled() ? "Enabled" : "Disabled");
  Serial.print(" * Motion Threshold:          ");
  Serial.println(mpu.getMotionDetectionThreshold());
  Serial.print(" * Motion Duration:           ");
  Serial.println(mpu.getMotionDetectionDuration());
  Serial.print(" * Zero Motion Threshold:     ");
  Serial.println(mpu.getZeroMotionDetectionThreshold());
  Serial.print(" * Zero Motion Duration:      ");
  Serial.println(mpu.getZeroMotionDetectionDuration());
  Serial.print(" * Clock Source:              ");
  switch (mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  Serial.print(" * Accelerometer:             ");
  switch (mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;

      Serial.print(" * Accelerometer offsets:     ");
      Serial.print(mpu.getAccelOffsetX());
      Serial.print(" / ");
      Serial.print(mpu.getAccelOffsetY());
      Serial.print(" / ");
      Serial.println(mpu.getAccelOffsetZ());

      Serial.print(" * Accelerometer power delay: ");
      switch (mpu.getAccelPowerOnDelay())
      {
        case MPU6050_DELAY_3MS:            Serial.println("3ms"); break;
        case MPU6050_DELAY_2MS:            Serial.println("2ms"); break;
        case MPU6050_DELAY_1MS:            Serial.println("1ms"); break;
        case MPU6050_NO_DELAY:             Serial.println("0ms"); break;
      }
      Serial.println();
  }
}
