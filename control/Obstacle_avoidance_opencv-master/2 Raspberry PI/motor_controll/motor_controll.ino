// Use Mega's hardware serial ports
#define HOVER_RIGHT Serial2 // Right wheels on TX2/RX2
#define HOVER_LEFT Serial3  // Left wheels on TX3/RX3

// Hoverboard communication settings
#define HOVER_SERIAL_BAUD   115200      // Baud rate for hoverboard communication
#define START_FRAME         0xABCD      // Start frame for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX           1000        // Maximum speed for testing

// Cytron motor driver pins
#define MOTOR1_PWM 5        // PWM pin for Motor 1
#define MOTOR1_DIR 6        // Direction pin for Motor 1
#define MOTOR2_PWM 9        // PWM pin for Motor 2
#define MOTOR2_DIR 10       // Direction pin for Motor 2

// DC motor settings
#define MOTOR_SPEED         180      // Speed for raising/lowering (0-255)
#define LOWER_DURATION      700      // Time to lower DC motors (ms)

// Brushless motor (ESC) settings
#include <Servo.h>
Servo Esc;
const int escPin = 7;
const int pulseMin = 1000; // Minimum pulse width for ESC (1000 µs)
const int pulseMax = 2000; // Maximum pulse width for ESC (2000 µs)
const int minThrottle = 0;   // Minimum throttle (stop)
const int maxThrottle = 180; // Maximum throttle (full speed)
bool isCutterRunning = false; // Tracks whether the cutter is running

// Data structures for commands and feedback
typedef struct {
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct {
   uint16_t start;
   int16_t  speedLeft;
   int16_t  speedRight;
   int16_t  currentLeft;
   int16_t  currentRight;
   int16_t  temperature;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;

// ########################## SETUP ##########################
void setup() {
  Serial.begin(115200);          // Debugging via USB

  // Initialize serial ports for hoverboard communication
  HOVER_RIGHT.begin(HOVER_SERIAL_BAUD);    // Right wheels
  HOVER_LEFT.begin(HOVER_SERIAL_BAUD);     // Left wheels

  // Initialize Cytron motor driver pins
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);

  // Initialize ESC
  Esc.attach(escPin, pulseMin, pulseMax);
  calibrateESCs();

  Serial.println("Setup complete. Ready for commands.");
}

// ########################## SEND ##########################
bool Send(HardwareSerial &port, int16_t uSteer, int16_t uSpeed) {
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  size_t bytesWritten = port.write((uint8_t *) &Command, sizeof(Command));
  if (bytesWritten != sizeof(Command)) {
    Serial.println("Error: Failed to send command!");
    return false;
  }
  return true;
}

// ########################## RECEIVE FEEDBACK ##########################
bool ReceiveFeedback(HardwareSerial &port, SerialFeedback &feedback) {
  if (port.available() >= sizeof(SerialFeedback)) {
    port.readBytes((uint8_t *)&feedback, sizeof(SerialFeedback));

    // Verify start frame and checksum
    if (feedback.start == START_FRAME && 
        feedback.checksum == (feedback.start ^ feedback.speedLeft ^ feedback.speedRight ^ feedback.currentLeft ^ feedback.currentRight ^ feedback.temperature)) {
      return true; // Valid feedback received
    }
  }
  return false; // No valid feedback
}

// ########################## LOOP ##########################
void loop() {
  static unsigned long lastSendTime = 0;
  unsigned long timeNow = millis();

  // Check for commands from Raspberry Pi
  if (Serial.available()) {
    char cmd = Serial.read();

    // Process command
    bool commandSuccess = false;
    switch (cmd) {
      case 'F': // Forward
        commandSuccess = Send(HOVER_RIGHT, 0, SPEED_MAX) && Send(HOVER_LEFT, 0, SPEED_MAX);
        LowerDCMotors(); // Lower DC motors
        StartCutter();   // Start brushless motor
        break;
      case 'B': // Backward
        commandSuccess = Send(HOVER_RIGHT, 0, -SPEED_MAX) && Send(HOVER_LEFT, 0, -SPEED_MAX);
        break;
      case 'L': // Left turn
        commandSuccess = Send(HOVER_RIGHT, 0, SPEED_MAX) && Send(HOVER_LEFT, 0, 0);
        break;
      case 'R': // Right turn
        commandSuccess = Send(HOVER_RIGHT, 0, 0) && Send(HOVER_LEFT, 0, SPEED_MAX);
        break;
      case 'S': // Stop
        commandSuccess = Send(HOVER_RIGHT, 0, 0) && Send(HOVER_LEFT, 0, 0);
        StopCutter();    // Stop brushless motor
        RaiseDCMotors(); // Raise DC motors
        break;
      default:
        Serial.println("Error: Invalid command!");
        break;
    }

    // Log command success or failure
    if (commandSuccess) {
      Serial.println("Command executed successfully.");
    } else {
      Serial.println("Error: Command execution failed!");
    }
  }

  // Receive feedback from motors
  SerialFeedback feedbackRight, feedbackLeft;

  if (ReceiveFeedback(HOVER_RIGHT, feedbackRight)) {
    Serial.print("Right Motor - Speed: ");
    Serial.print(feedbackRight.speedRight);
    Serial.print(", Current: ");
    Serial.print(feedbackRight.currentRight);
    Serial.print(", Temp: ");
    Serial.println(feedbackRight.temperature);
  }

  if (ReceiveFeedback(HOVER_LEFT, feedbackLeft)) {
    Serial.print("Left Motor - Speed: ");
    Serial.print(feedbackLeft.speedLeft);
    Serial.print(", Current: ");
    Serial.print(feedbackLeft.currentLeft);
    Serial.print(", Temp: ");
    Serial.println(feedbackLeft.temperature);
  }

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow % 2000) < 1000);
}

// ########################## HELPER FUNCTIONS ##########################
void LowerDCMotors() {
  Serial.println("Lowering DC motors...");
  digitalWrite(MOTOR1_DIR, LOW); // Set direction for lowering
  digitalWrite(MOTOR2_DIR, LOW);
  analogWrite(MOTOR1_PWM, MOTOR_SPEED); // Set speed
  analogWrite(MOTOR2_PWM, MOTOR_SPEED);
  delay(LOWER_DURATION); // Wait for motors to lower
  analogWrite(MOTOR1_PWM, 0); // Stop motors
  analogWrite(MOTOR2_PWM, 0);
  Serial.println("DC motors lowered.");
}

void RaiseDCMotors() {
  Serial.println("Raising DC motors...");
  digitalWrite(MOTOR1_DIR, HIGH); // Set direction for raising
  digitalWrite(MOTOR2_DIR, HIGH);
  analogWrite(MOTOR1_PWM, MOTOR_SPEED); // Set speed
  analogWrite(MOTOR2_PWM, MOTOR_SPEED);
  delay(LOWER_DURATION); // Wait for motors to raise
  analogWrite(MOTOR1_PWM, 0); // Stop motors
  analogWrite(MOTOR2_PWM, 0);
  Serial.println("DC motors raised.");
}

void StartCutter() {
  if (!isCutterRunning) {
    Serial.println("Starting brushless motor...");
    Esc.write(maxThrottle); // Start the motor
    isCutterRunning = true;
  }
}

void StopCutter() {
  if (isCutterRunning) {
    Serial.println("Stopping brushless motor...");
    Esc.write(minThrottle); // Stop the motor
    isCutterRunning = false;
  }
}

void calibrateESCs() {
  Serial.println("Calibrating ESC...");
  Esc.write(maxThrottle); // Send maximum throttle signal
  delay(5000);            // Wait for ESC to recognize max throttle
  Esc.write(minThrottle); // Send minimum throttle signal
  delay(5000);            // Wait for ESC to recognize min throttle
  Serial.println("ESC calibration complete.");
}
