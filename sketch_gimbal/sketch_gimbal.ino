#include <Servo.h>
#include <MPU9250_WE.h>

#define pitchPin 3
#define yawPin 4
#define rollPin 2
#define joystickXPin A1
#define joystickYPin A2
#define joystickClickPin 5

MPU9250_WE myMPU9250 = MPU9250_WE();
Servo pitchServo;
Servo yawServo;
Servo rollServo;

float targetPitch = 90;
float targetYaw = 90;
float targetRoll = 90; 

float currentPitch = 90;
float currentYaw = 90;
bool is3Axis = false; 

int maxPitch = 135;
int minPitch = 45;
int minYaw = 0;
int maxYaw = 180;
int minRoll = 45;
int maxRoll = 135;

int deadzone = 32;
int status;

float lastRoll, lastPitch, lastYaw; 
float rollSmoothness = 1;
float pitchSmoothness = 1;

float lastX, lastY; 
float sensorJitter = 1; 
void setup() {
  // put your setup code here, to run once:
  
  pitchServo.attach(pitchPin);
  yawServo.attach(yawPin);
  rollServo.attach(rollPin);
  
  pitchServo.write(targetPitch);
  yawServo.write(targetYaw);
  rollServo.write(targetRoll);
  
  Serial.begin(115200);

  Wire.begin();
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }

  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(2500);
  myMPU9250.autoOffsets();
  Serial.println("Done!");
  
  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
}

void loop() {
  // put your main code here, to run repeatedly:
  int y = analogRead(joystickXPin);
  int x = analogRead(joystickYPin);

  if (x > 512 + deadzone || x < 512 - deadzone) {
    targetYaw -= ((float)(x) - 512.0) / 100.0; 
    if (targetYaw > maxYaw) targetYaw = maxYaw;
    if (targetYaw < minYaw) targetYaw = minYaw;
    delay(5);
  }

  if (y > 512 + deadzone || y < 512 - deadzone) {
    targetPitch -= ((float)(y) - 512.0) / 300.0; 
    if (targetPitch > maxPitch) targetPitch = maxPitch;
    if (targetPitch < minPitch) targetPitch = minPitch; 
    delay(5);
  }

/*
  pitchServo.write(targetPitch);
  rollServo.write(targetRoll); */
  yawServo.write(targetYaw);


  
  xyzFloat angles = myMPU9250.getAngles();
  
/* This method provides quite precise values for x/y 
   angles up 60°. */
  Serial.print("Angle x = ");
  Serial.print(angles.x);
  Serial.print("  |  Angle y = ");
  Serial.print(angles.y);
  Serial.print("  |  Angle z = ");
  Serial.println(angles.z);

/* Pitch and roll consider all axes for calculation. According to my experience
   it provides more reliable results at higher angles (>60°) */
  float pitch = myMPU9250.getPitch();
  float roll  = myMPU9250.getRoll();

  Serial.print("Pitch   = "); 
  Serial.print(pitch); 
  Serial.print("  |  Roll    = "); 
  Serial.println(roll); 
  Serial.print("Orientation of the module: ");
  Serial.println(myMPU9250.getOrientationAsString());
  
  Serial.println();
  

  if (abs((float)angles.x - lastX) > sensorJitter) lastX = angles.x;
  if (abs((float)angles.y - lastY) > sensorJitter) lastY = angles.y; 

  
  float newRoll = lastX + targetRoll;
  float newPitch = lastY*-1 + targetPitch;
  if (newRoll - lastRoll > rollSmoothness) newRoll = lastRoll + rollSmoothness;
  if (newRoll - lastRoll < -rollSmoothness) newRoll = lastRoll - rollSmoothness;
  if (newPitch - lastPitch > pitchSmoothness) newPitch = lastPitch + pitchSmoothness;
  if (newPitch - lastPitch < -pitchSmoothness) newPitch = lastPitch - pitchSmoothness; 
  
  rollServo.write(newRoll);
  pitchServo.write(newPitch);
  lastRoll = newRoll;
  lastPitch = newPitch;
  
  
}
