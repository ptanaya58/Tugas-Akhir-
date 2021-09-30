//LIBRARY DISINI//
#include <Fuzzy.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>


//PIN INITIALIZATION DAN VARIABEL DISINI//
//Motor pin initialization
int motor1pin1 = 26;
int motor1pin2 = 27;

int motor2pin1 = 14;
int motor2pin2 = 12;

//Enable pin initialization
int ENA = 21;
int ENB = 19;

// Setting PWM properties
const int freq = 30000;
const int pwmChannelA = 0;
const int pwmChannelB = 1;
const int resolution = 8;

//Sensor TCR5000RT Pin Initialization
const int IR1 = 25;
const int IR2 = 33;
const int IR3 = 32;
const int IR4 = 35;
const int IR5 = 34;

//Value Error Initialization
int sensor[5] = {0, 0, 0, 0, 0};

// PID Constanta
float Kp = 8;
float Ki = 0.1;
float Kd = 13;


// Initial motor speed
int initial_motor_speed = 145;
float left_motor_speed = 0;
float right_motor_speed = 0;

//Variable initialization
float error = 0, P = 0, I = 0, D = 0, PID_value = 0, pos = 0;
float previous_error = 0, previous_I = 0;
float setpoint = 0;


//FUNGSI-FUNGSI DISINI//
void forward() {
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void backward() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}

void left() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void right() {
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}

void sharpLeftTurn() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void sharpRightTurn() {
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}

void stop_bot() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}

void motor_control() {
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed - PID_value;
  int right_motor_speed = initial_motor_speed + PID_value;

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);

  //  Serial.println(left_motor_speed);
  //  Serial.println(right_motor_speed);
  /*Serial.print(PID_value);
    //    Serial.print("\t");
    //    Serial.print(left_motor_speed);
    //    Serial.print("\t");
    //    Serial.println(right_motor_speed);*/
  ledcWrite(pwmChannelA, left_motor_speed); //Left Motor Speed
  ledcWrite(pwmChannelB, right_motor_speed); //Right Motor Speed *

  //following lines of code are to make the bot move forward
  forward();
}

void calculate_pid() {
  P = error;
  I = I + error;
  D = error - previous_error;
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
  previous_error = error;
  
//  Serial.println(PID_value);
}

void read_sensor_values() {
  sensor[0] = !digitalRead(IR1);
  sensor[1] = !digitalRead(IR2);
  sensor[2] = !digitalRead(IR3);
  sensor[3] = !digitalRead(IR4);
  sensor[4] = !digitalRead(IR5);

  //  Serial.print(sensor[0]);
  //  Serial.print("\t");
  //  Serial.print(sensor[1]);
  //  Serial.print("\t");
  //  Serial.print(sensor[2]);
  //  Serial.print("\t");
  //  Serial.print(sensor[3]);
  //  Serial.print("\t");
  //  Serial.println(sensor[4]);
  if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
    error = 4;
  }
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
    error = 3;
  }
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
    error = 2;
  }
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] = 1) && (sensor[3] == 0) && (sensor[4] == 0)) {
    error = 1;
  }
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)) {
    error = 0;
  }
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0)) {
    error = -1;
  }
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0)) {
    error = -2;
  }
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1)) {
    error = -3;
  }
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1)) {
    error = -4;
  }
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) {
    error = -5;
    stop_bot;
  }
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) {
    error = 5;
    stop_bot;
  }
}

// FUNGSI PEMBACAAN RSSI DISINI//
Fuzzy *fuzzy = new Fuzzy();

String knownBLEAddresses[] = {"f4:5e:ab:bb:78:7c","64:69:4e:3e:04:66", "64:69:4e:25:71:c6"};
int best = -100;
int largest_rssi = -100;
bool device_found;
int scanTime = 1; //In seconds
BLEScan* pBLEScan;
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      for (int i = 0; i < (sizeof(knownBLEAddresses) / sizeof(knownBLEAddresses[0])); i++)
      {
        //Uncomment to Enable Debug Information
        //Serial.println("*************Start**************");
        //Serial.println(sizeof(knownBLEAddresses));
        //Serial.println(sizeof(knownBLEAddresses[0]));
        //Serial.println(sizeof(knownBLEAddresses)/sizeof(knownBLEAddresses[0]));
        //Serial.println(advertisedDevice.getAddress().toString().c_str());
        //Serial.println(knownBLEAddresses[i].c_str());
        //Serial.println("*************End**************");
        if (strcmp(advertisedDevice.getAddress().toString().c_str(), knownBLEAddresses[i].c_str()) == 0)
        {
          device_found = true;
          break;
        }
        else
          device_found = false;
      }
//      Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
    }
};

//TASK 1 DISINI//
TaskHandle_t FirstTask;
TaskHandle_t SecondTask;
void pid_garis( void * pvParameters ){
  while (true){
    read_sensor_values();
    Serial.print(",");
    Serial.print(error);
    Serial.print(",");
    Serial.println(largest_rssi);
    Serial.print(",");
    Serial.println(initial_motor_speed);
    if (error == 5 || error == -5){
      stop_bot;
    }
    else{
      calculate_pid();
      motor_control(); 
    }
    delay(50);
  }
}

//void pid_kecepatan( void * pvParameters ){
//  while (true){
//    delay(50);
//
//  }
//}

void setup()
{
  Serial.begin(9600);
   //SET UP MOTOR
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(ENB, OUTPUT);

  // configure LED PWM functionalitites
  ledcSetup(pwmChannelA, freq, resolution);
  ledcSetup(pwmChannelB, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ENA, pwmChannelA);
  ledcAttachPin(ENB, pwmChannelB);

  //SET UP IR SENSOR
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);

  //FUZZY LOGIC SET UP//
    //FUZZY LOGIC INPUT SET UP
  FuzzyInput *rssival = new FuzzyInput(1);
  FuzzySet *dekat = new FuzzySet(-77, -65, -65, -65);
  rssival->addFuzzySet(dekat);
  FuzzySet *sedang = new FuzzySet(-95, -82, -82, -70);
  rssival->addFuzzySet(sedang);
  FuzzySet *jauh = new FuzzySet(-100, -100, -100, -87);
  rssival->addFuzzySet(jauh);
  fuzzy->addFuzzyInput(rssival);

    //FUZZY LOGIC OUTPUT SET UP
  FuzzyOutput *speed = new FuzzyOutput(1);
  FuzzySet *slow = new FuzzySet(70, 70, 70, 122);
  speed->addFuzzySet(slow);
  FuzzySet *average = new FuzzySet(105, 127, 127, 149);
  speed->addFuzzySet(average);
  FuzzySet *fast = new FuzzySet(133, 155, 155, 155);
  speed->addFuzzySet(fast);
  fuzzy->addFuzzyOutput(speed);
  
    //FUZZY LOGIC RULE SET UP
  FuzzyRuleAntecedent *ifDistanceDekat = new FuzzyRuleAntecedent();
  ifDistanceDekat->joinSingle(dekat);
  FuzzyRuleConsequent *thenSpeedSlow = new FuzzyRuleConsequent();
  thenSpeedSlow->addOutput(slow);
  FuzzyRule *fuzzyRule01 = new FuzzyRule(1, ifDistanceDekat, thenSpeedSlow);
  fuzzy->addFuzzyRule(fuzzyRule01);


  FuzzyRuleAntecedent *ifDistanceSedang = new FuzzyRuleAntecedent();
  ifDistanceSedang->joinSingle(sedang);
  FuzzyRuleConsequent *thenSpeedAverage = new FuzzyRuleConsequent();
  thenSpeedAverage->addOutput(average);
  FuzzyRule *fuzzyRule02 = new FuzzyRule(2, ifDistanceSedang, thenSpeedAverage);
  fuzzy->addFuzzyRule(fuzzyRule02);


  FuzzyRuleAntecedent *ifDistanceJauh = new FuzzyRuleAntecedent();
  ifDistanceJauh->joinSingle(jauh);
  FuzzyRuleConsequent *thenSpeedFast = new FuzzyRuleConsequent();
  thenSpeedFast->addOutput(fast);
  FuzzyRule *fuzzyRule03 = new FuzzyRule(3, ifDistanceJauh, thenSpeedFast);
  fuzzy->addFuzzyRule(fuzzyRule03);

  //SET UP READ BEACON//
  Serial.println("Scanning..."); // Print Scanning
  BLEDevice::init("");

  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks()); //Init Callback Function
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100); // set Scan interval
  pBLEScan->setWindow(99);  // less or equal setInterval value  

  Serial.println("Date & Time, RSSI");

  delay(20000);
  
  //create a task that will be executed in the pid_garis() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
     pid_garis,    //Task function.
     "FirstTask", //name of task
     10000, //Stack size of task
     NULL, //parameter of the task
     1, //priority of the task
     &FirstTask, //Task handle to keep track of created task
     0); //pin task to core 0
     
//  //create a task that will be executed in the pid_kecepatan() function, with priority 1 and executed on core 0
//  xTaskCreatePinnedToCore(
//     pid_kecepatan,    //Task function.
//     "SecondTask", //name of task
//     10000, //Stack size of task
//     NULL, //parameter of the task
//     1, //priority of the task
//     &SecondTask, //Task handle to keep track of created task
//     0); //pin task to core 0

}


void loop()
{
  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  largest_rssi = best ;
  for (int i = 0; i < foundDevices.getCount(); i++){
    BLEAdvertisedDevice device = foundDevices.getDevice(i);
    int rssi = device.getRSSI();
//    Serial.print("RSSI: ");
//    Serial.println(rssi);
    if (rssi > largest_rssi){
      largest_rssi = rssi;
    }
  }
//  Serial.println(largest_rssi);

  fuzzy->setInput(1, largest_rssi);
  fuzzy->fuzzify();
  float output = fuzzy->defuzzify(1);
//  Serial.println("Result: ");
//  Serial.print("\t\t\tSpeed: ");
//  Serial.println(output);
  
  initial_motor_speed = output;
//  Serial.print("initmotor = ");
//  Serial.println(initial_motor_speed);

  pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
  delay(1);

}
