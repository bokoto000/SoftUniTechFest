#include <QTRSensors.h> // library used for sensors
#define NUM_SENSORS   6     
#define TIMEOUT       2500  
#define EMITTER_PIN   2     

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {
  A0, A1, A2, A3, A4, A5
},
NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
int p=3;//takes p sensor readings before calculating position for precise results

double Kp = 0.030, Ki = 0, Kd = 0.89; //PID constants

int pwma = 5; //motor A speed  pin
int pwmb = 6; //motor B speed pin
int ain2 = 8; //direction of motor A
int ain1 = 7; //direction of motor A
int stby = 9; //standby pin; spira motorite
int bin1 = 11; //direction of motor B
int bin2 = 10; //direction of motor B
int bx=100,by=100; //base speed for motor A(x) and B(y)
int x=bx, y=by; //base speed for motor A(x) and B(y)
int rightMaxSpeed = 130, leftMaxSpeed =130; //maximal speed for motors
int minx=x-(rightMaxSpeed-x)/4,miny=y-(leftMaxSpeed-y)/4; //minimal speed for motors
int error = 0; //error calculated by IR sensors
int integral = 0; 
int call[16], sensors[16]; 
int lastError = 0; //last error calculated by PID algorithm
int mid[16] = {0, 0, 0, 0, 0, 0, 0, 0};
int position=0;
void setup()
{
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission; needed to check sensors value through the serial monitor

  pinMode(LED_BUILTIN, OUTPUT);//sets pinmode for the led
  digitalWrite(LED_BUILTIN, HIGH);//indicates that the calibration has stated
  //callib();
  manual_calibration();//the calibration function
  digitalWrite(LED_BUILTIN, LOW);//indicates that the calibration has ended
  delay(2000);
  //sets pin modes for the motors
  pinMode(pwma, OUTPUT); 
  pinMode(pwmb, OUTPUT);
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);
  pinMode(stby, OUTPUT);
  digitalWrite(ain1, HIGH);
  digitalWrite(ain2, LOW);
  digitalWrite(bin1, HIGH);
  digitalWrite(bin2, LOW);
  digitalWrite(stby, HIGH);
}

void loop()
{
  sensors_reading();//get input from sensors
  if(isThereVase())
  {
    Stop();//stops the motors
    checkGround();//checks the moisture percentage
    sendData();//sends the data and the number  of the vase to station
    Start();//Starts the motors
  }
  
  PID();//starts PID algorithm
  
}
void sensors_reading()
{
  int position=0;//the current position variable
  //takes p sensor readings before calculating position for precise results
  for(int i=1;i<=p;i++){
  position += qtrrc.readLine(sensorValues);}
  position/=p;
  //more input from sensors that checks if there is a vase
}
bool isThereVase()
{
  return true;
}
void checkGround()
{
  //checks water % in soil
}
void sendData()
{
  //sends data to station
}
void Start()
{
  digitalWrite(stby,HIGH);
}
void Stop()
{
  digitalWrite(stby,LOW);
}
void PID()
{
  error = position - 2500; //calculates error
  Serial.println(error); // prints current error to serial monitor
  
  double motorSpeed = Kp * error + Ki * integral + Kd * (error - lastError); //calculates the motorSpeed using PID algorithm
  integral += error; //adds current error to the integral
  lastError = error; //remembers current error
  motorspeedup(motorSpeed); //changes motor speed 
}
void manual_calibration() {
  int i;
  for (i = 0; i < 250; i++)  // the calibration will 5 seconds
  {
    qtrrc.calibrate(QTR_EMITTERS_ON);
    delay(20);
  }
}
void speedWrite(int dqsno , int lqvo)
{
  analogWrite(pwma, dqsno);
  analogWrite(pwmb, lqvo);
}
void motorspeedup(int motorSpeed)
{
  int rightMotorSpeed = 0, leftMotorSpeed = 0;
  rightMotorSpeed = x + motorSpeed;
  leftMotorSpeed  = y - motorSpeed;
  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed )   leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed <= 0) rightMotorSpeed = minx; // keep the motor speed positive
  if (leftMotorSpeed <= 0)  leftMotorSpeed = miny; // keep the motor speed positive
  speedWrite(leftMotorSpeed, rightMotorSpeed);
}

