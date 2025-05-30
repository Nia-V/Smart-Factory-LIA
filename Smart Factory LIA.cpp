/*
This code,,,, sure is something
If anyone is reading this, know that this code was made in like 2 seconds at the last minute
when I somehow managed to trigger a WDT bug in micropython for ESP32 ;-;
It was originally coded in python until then
*/

#define HX711_DOUT 15 //scale broke so not used ;-;
#define HX711_SCK 16//scale broke so not used ;-;
#define FLOW_SENSOR 27
#define PUMP_PWM_PIN 14
#define BORAX_FWD_PIN 23
#define BORAX_REV_PIN 22
#define HEATER_PIN 21
#define MIXER_PIN 19
#define GLUE_PIN 18
#define SLIME_PIN 17
#define TEMP_ADC_PIN 25
#define TORQ_ADC_PIN 35
#define RESET_PIN 32
//RPM pulse increment 
volatile int pulses = 0;
void IRAM_ATTR handlePulse() {
  pulses++;
}

//Flow pulse increment
volatile int pulseCount = 0;
void IRAM_ATTR onPulse() {
  pulseCount++;
}

//Flow variable definition
int initialFlowValue = 0;
int currentFlowValue = 0;

float flowRate;
float flowSensor;

unsigned long totalMilliLitres;
unsigned long oldTime;
unsigned long sampleTime;

const float CALIBRATION_FACTOR = 73;

//MQTT stuff that never got implemented finally

const char* ssid = "****";
const char* password = "****";
const char* mqtt_server = "192.168.0.49";

WiFiClient espClient;
PubSubClient client(espClient);

//Run once then stop variable
bool first = true;

//PID Function Variables
float pidIntegral = 0;
float pidPreviousError = 0;
int pumpPWM = 0;


//Blocking Flow condition function
void waitFlowCondition(float targetLiters) {
  analogWrite(PUMP_PWM_PIN, 255);
  Serial.println("Pumping");
  pulseCount = 0;
// while total flow is not reached keep pump on
  int neededPulses = targetLiters * CALIBRATION_FACTOR;
  
  while (pulseCount < neededPulses) {
    float vol = pulseCount / CALIBRATION_FACTOR;
    Serial.printf("[FLOW] %.2f L (%d/%d pulses)\n", vol, pulseCount, neededPulses);
    delay(100);
  }

  analogWrite(PUMP_PWM_PIN, 0);
  Serial.println("[FLOW]");
}


void setup() {
//Monitor init
  Serial.begin(9600); 
//pin init
  pinMode(BORAX_FWD_PIN, OUTPUT);
  pinMode(BORAX_REV_PIN, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(MIXER_PIN, OUTPUT);
  pinMode(GLUE_PIN, OUTPUT);
  pinMode(SLIME_PIN, OUTPUT);
  pinMode(PUMP_PWM_PIN, OUTPUT);
  pinMode(FLOW_SENSOR, INPUT_PULLUP);
  pinMode(TORQ_ADC_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);
// bring all pins high (because of relay wiring, this actaully means off)
  digitalWrite(BORAX_FWD_PIN, 1);
  digitalWrite(BORAX_REV_PIN, 1);
  digitalWrite(HEATER_PIN, 1);
  digitalWrite(MIXER_PIN, 1);
  digitalWrite(GLUE_PIN, 1);
  digitalWrite(SLIME_PIN, 1);
  
//Make sure water pump pin is set to zero at start
  analogWrite(PUMP_PWM_PIN, 0);
  
  //init  for pulse count interrupt functions (peed encoder and flow sensor)
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR), onPulse, RISING);

  attachInterrupt(digitalPinToInterrupt(TORQ_ADC_PIN), handlePulse, RISING);
}
//Blocking temp condition function (lol why would I ever create a private variable, thats for losers, and me I'm bad at programming (⌐■_■))
void waitTempCondition(int threshold) {
  Serial.println(analogRead(TEMP_ADC_PIN));
  // while temp is not reached keep heater on
  while (analogRead(TEMP_ADC_PIN) <= threshold) {
    Serial.println(String(analogRead(TEMP_ADC_PIN)));
    delay(300);
    digitalWrite(HEATER_PIN, 0);
  }
  Serial.println("[TEMP] Reached: " + String(analogRead(TEMP_ADC_PIN)));
  digitalWrite(HEATER_PIN, 1);
  return;
}

//Calculate speed encoder pulse rate per second
/*this wasnt working so i did some wacky stuff to try and make it work so this function is probably balls,
i think it was bad wiring in the end, but it works so im not changing it*/
float readRPM() {  
  static unsigned long lastPrint = 0;
  //get the pulses every second and save to count, then reset pulses
  if (millis() - lastPrint > 1000) {  
    noInterrupts();
    unsigned long count = pulses;
    pulses = 0;  
    interrupts();

    Serial.println(count);
    lastPrint = millis();
    return count;
  }
}
/*
PID function for controlling the viscocity of the slime
PV = torque measured by speed encoder and NOT the current sensor
CV = PWM for Water pump voltage
It will add more and more water to the slime until the desired slime consistency is reached
*/


struct PID {
  float Kp, Ki, Kd;
  float prevError = 0;
  float integral = 0;
  float outputMin = 0;
  float outputMax = 255;
  float epsilon = 0.01;  
  float dt = 0.05;       

  PID(float kp, float ki, float kd, float minOut = 0, float maxOut = 255, float eps = 0.01)
    : Kp(kp), Ki(ki), Kd(kd), outputMin(minOut), outputMax(maxOut), epsilon(eps) {}

  void runUntilSetpointReached(float setpoint) {
    prevError = 0;
    integral = 0;

    while (true) {
      float measured = readRPM() / 20; //20 pulses per rotation on our encoder!!
      float error = setpoint - measured;

    //if the error is small enough, stop
      if (abs(error) <= epsilon) {
        Serial.println("PID Function Done");
        analogWrite(PUMP_PWM_PIN, 0);  
        break;
      }
    // otherwise calculate PID water pump pwm
      integral += error * dt;
      float derivative = (error - prevError) / dt;
      float output = Kp * error + Ki * integral + Kd * derivative;
      prevError = error;

      
      if (output > outputMax) output = outputMax;
      if (output < outputMin) output = outputMin;

     
      analogWrite(PUMP_PWM_PIN, output);

      
      Serial.print("Setpoint: ");
      Serial.print(setpoint);
      Serial.print(" Nm, Measured: ");
      Serial.print(measured);
      Serial.print(" Nm, Error: ");
      Serial.print(error);
      Serial.print(" Nm, Pump Voltage (PWM): ");
      Serial.println((int)output);

      delay(dt * 10000);  
    }
  }
};




/*3D PRinted Rotary valve for Borax dosing was stiff and difficult
to rotate so we had to pulse the motor a little, in the end it still didnt work properly
(NEVER use WD40 on a 3D print)*/
void Borax(int doses) {
  for (int x = 0; x < doses; x++) {
    digitalWrite(BORAX_REV_PIN, 1);
    digitalWrite(BORAX_FWD_PIN, 1);
    digitalWrite(BORAX_FWD_PIN, LOW);
    delay(800);
    digitalWrite(BORAX_FWD_PIN, HIGH);
    delay(800);
    digitalWrite(BORAX_FWD_PIN, LOW);
    delay(800);
    digitalWrite(BORAX_FWD_PIN, HIGH);
    delay(800);
    digitalWrite(BORAX_FWD_PIN, LOW);
    delay(800);
    digitalWrite(BORAX_FWD_PIN, HIGH);
    delay(800);
    digitalWrite(BORAX_FWD_PIN, LOW);
    delay(800);
    digitalWrite(BORAX_FWD_PIN, HIGH);
    delay(800);
    digitalWrite(BORAX_REV_PIN, LOW);
    delay(800);
    digitalWrite(BORAX_REV_PIN, HIGH);
    delay(800);
    digitalWrite(BORAX_REV_PIN, LOW);
    delay(800);
    digitalWrite(BORAX_REV_PIN, HIGH);
    delay(800);
    digitalWrite(BORAX_REV_PIN, LOW);
    delay(800);
    digitalWrite(BORAX_REV_PIN, HIGH);
    delay(800);
  }
}
void loop() {
  float setpointTorque = 5.0;        
  float measuredTorque = readRPM();  
  float dt = 5;                      
  bool reset = digitalRead(RESET_PIN);
  PID torquePID(2.0, 0.5, 0.0);
  // put your main code here, to run repeatedly:
  if (first == 1 && reset == 1) {
    waitTempCondition(500);
    Borax(1);
    waitFlowCondition(15);
    digitalWrite(MIXER_PIN, 0);//MIXER ON
    delay(120000);//MIX 1 MIN
    digitalWrite(GLUE_PIN, 0);//Start adding glue while mixing
    delay(120000);
    digitalWrite(GLUE_PIN, 1);//stop adding glue
    Serial.println("Stopped");
    delay(15000);//mix 15 more secs for it to fully combine
    torquePID.runUntilSetpointReached(10);// Start PID function (Pumping water and measuring torque)
    digitalWrite(MIXER_PIN, 1);//Turn off mixer
    digitalWrite(SLIME_PIN, 0);//Turn on slime pump to dispense 
    delay(120000);
    digitalWrite(SLIME_PIN, 1); //Turn off
    first = false;//Process does not repeat
  } else {
    delay(300);
  }
  if (reset == 0) {
    digitalWrite(SLIME_PIN, 0);//Reset and purge bad batch using slime pump
  }
}
