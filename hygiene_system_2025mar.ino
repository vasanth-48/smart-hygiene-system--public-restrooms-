#define BLYNK_TEMPLATE_ID "TMPL3805U44uf"
#define BLYNK_TEMPLATE_NAME "smart toilet"
#include <Servo.h>
#include <BlynkSimpleEsp8266.h>
#include <ESP8266WiFi.h>
#define BLYNK_TEMPLATE_ID "TMPL3805U44uf"
#define BLYNK_TEMPLATE_NAME "smart toilet"
// WiFi & Blynk Credentials
char auth[] = "q5a2pq7HRMzejyBpKavd69cOV9hpP1cu";
char ssid[] = "OnePlus Nord CE 2";
char pass[] = "v@t10343";

// Define pin connections
#define FLOW_SENSOR_PIN 5  // D1 (Flow sensor)
#define TRIG_PIN 4         // D2 (Ultrasonic sensor TRIG)
#define ECHO_PIN 0         // D3 (Ultrasonic sensor ECHO)
#define SERVO_PIN 2        // D4 (Servo motor)
#define IR_SENSOR_PIN 14   // D5 (IR sensor)

// DC Motor Control Pins
#define MOTOR_IN1 12   // D6 (Motor driver IN1)
#define MOTOR_IN2 13   // D7 (Motor driver IN2)
#define MOTOR_ENA 15   // D8 (Motor driver ENA, PWM)

// Constants
const float FLOW_THRESHOLD = 0.1;      // Flow rate threshold in L/min
const float DISTANCE_THRESHOLD = 8.5;  // Distance threshold in cm
const unsigned long INTERVAL = 1000;   // 1s interval for flow calculation
const unsigned long MOTOR_DELAY = 3000; // 3s delay before DC motor starts/stops
const unsigned long DOOR_CLOSE_DELAY = 6000; // 6s delay before closing door
const unsigned long MOTOR_DISABLE_TIME = 30000; // 30s motor disable time
const int COUNT_LIMIT = 20; // Max count before triggering delay sequence

// Variables
volatile int flowPulseCount = 0;
float flowRate = 0;
unsigned long previousMillis = 0;
bool waterLevelOkay = false;
int count = 0; // Count variable to track state
bool messageSent = false;
bool motorDisabled = false; // Flag to prevent motor operation
unsigned long motorDisableStartTime = 0; // Timer for motor disable duration

Servo doorServo;

// Interrupt service routine for flow sensor
void IRAM_ATTR flowPulseCounter() {
  flowPulseCount++;
}

// **BLYNK BUTTON TO UNLOCK DOOR**
BLYNK_WRITE(V1) {  
    if (param.asInt() == 1 && count >= COUNT_LIMIT) {
        Serial.println("Blynk Button Pressed! Unlocking door...");
        doorServo.write(90);  // Open door
        count = 0;  // Reset count after opening the door
        messageSent = false; // Allow message to be sent again later
        Blynk.virtualWrite(V2, "Door Unlocked!");
    }
}

void setup() {
    Serial.begin(115200);
    Blynk.begin(auth, ssid, pass);

    pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(IR_SENSOR_PIN, INPUT);
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_ENA, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowPulseCounter, RISING);
    
    doorServo.attach(SERVO_PIN);
    doorServo.write(0); // Start with door closed

    Serial.println("Restroom Control System Started");
}

void loop() {
    Blynk.run();  // Run Blynk processes

    int irSensorState = digitalRead(IR_SENSOR_PIN);

    if (irSensorState == LOW) { // Object detected
        count++;
        Serial.print("Object Detected! Count: ");
        Serial.println(count);
        delay(500); // Prevent multiple rapid increments
    }

    if (count >= COUNT_LIMIT && !messageSent) {
        Serial.println("Count limit exceeded! Sending Blynk alert...");
        Blynk.virtualWrite(V2, "Unlock via Blynk.");
        messageSent = true;  // Prevent multiple messages
    }

    if (motorDisabled) {
        Serial.println("Motor is disabled. No operation allowed.");
        delay(1000);
        return;
    }

    if (count % 2 == 0) {
        // Even count: Door open → Monitor ultrasonic sensor
        float distance = measureDistance();
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
        delay(500);

        if (distance <= DISTANCE_THRESHOLD) {
            if (!waterLevelOkay) {
                Serial.println("Water Level OK! Opening Door...");
                
                if (!motorDisabled) startMotor();
                delay(MOTOR_DELAY);
                doorServo.write(90); // Open door
                waterLevelOkay = true;
            }
        } else {
            if (waterLevelOkay) {
                Serial.println("Low Water Level! Stopping Motor...");
                
                doorServo.write(0); // Close door
                Serial.println("Closing Door...");
                delay(DOOR_CLOSE_DELAY);
                stopMotor();
                waterLevelOkay = false;
            }
        }
    } else {
        // Odd count: Door closed → Monitor flow sensor only
        Serial.println("Monitoring Flow Sensor Only...");
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= INTERVAL) {
            previousMillis = currentMillis;
            flowRate = (flowPulseCount / 7.5);
            flowPulseCount = 0;

            Serial.print("Flow Rate: ");
            Serial.print(flowRate);
            Serial.println(" L/min");

            if (flowRate > FLOW_THRESHOLD) {
                Serial.println("Water is flowing... Starting DC motor!");
                if (!motorDisabled) startMotor();
            } else {
                Serial.println("No water flow detected. DC motor remains off.");
                stopMotor();
            }
        }
    }
    delay(500);
}

// Function to measure distance
float measureDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    float distance = (duration * 0.034) / 2;
    delay(200);
    return distance;
}

// Function to start DC motor
void startMotor() {
    if (!motorDisabled) {
        Serial.println("Starting DC Motor...");
        digitalWrite(MOTOR_IN1, HIGH);
        digitalWrite(MOTOR_IN2, LOW);
        analogWrite(MOTOR_ENA, 200);
        delay(500);
    } else {
        Serial.println("Motor is disabled. Start command ignored.");
    }
}

// Function to stop DC motor
void stopMotor() {
    Serial.println("Stopping DC Motor...");
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    delay(500);
}
