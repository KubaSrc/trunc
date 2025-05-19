const int pwmPin = 11;    // PWM-capable pin
const int in1 = 10;
const int in2 = 9;
const unsigned long delayDuration = 5000;  // Maximum ramp-up time
unsigned long pwm_start = 200;
unsigned long pwm_end  = 255;

float commandNum;      // Duration in ms (change as needed)
float absCommandNum;   // Absolute value (for calculations)
bool run = false;      // Set to true to run the motor command once
bool motorState = false;  // Tracks whether the motor is on or off

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(pwmPin, 0);
  pinMode(pwmPin, OUTPUT);
  Serial.begin(9600);
  Serial.println("setup done");
}

void loop() {
  if (Serial.available()) {
    commandNum = Serial.parseFloat();
    Serial.println(commandNum);

    while (Serial.available()) {
      Serial.read();
    }

    if (commandNum == 0) { // Toggle motor state if MATLAB sends 0
      motorState = !motorState;
      if (motorState) {
        Serial.println("Motor turned ON");
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        analogWrite(pwmPin, pwm_end);
      } else {
        Serial.println("Motor turned OFF");
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        analogWrite(pwmPin, 0);
      }
      return;
    }
    
    absCommandNum = abs(commandNum);
    Serial.println(commandNum);
    Serial.println(absCommandNum);
    run = true;
  }

  if (run) {
    unsigned long maxDuration = min((unsigned long)absCommandNum, delayDuration);
    const int increment = 15;
    unsigned long delayTime = maxDuration / increment;
    unsigned long pwmIncrement = (pwm_end - pwm_start) / increment;
    if (pwmIncrement == 0) pwmIncrement = 1;
    long finalDelay = absCommandNum - maxDuration;

    Serial.print("finalDelay: ");
    Serial.println(finalDelay);
    Serial.print("absCommandNum: ");
    Serial.println(absCommandNum);

    if (commandNum > 0) {
      Serial.println("Running forward.");
      int pwmValue = pwm_start;
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      for (; pwmValue <= pwm_end; pwmValue += pwmIncrement) {
        analogWrite(pwmPin, pwmValue);
        delay(delayTime);
      }
      if (finalDelay > 0) delay(finalDelay);
    } 
    else if (commandNum < 0) {
      Serial.println("Running backward.");
      int pwmValue = pwm_end - pwm_start;
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      for (; pwmValue >= 0; pwmValue -= pwmIncrement) {
        analogWrite(pwmPin, pwm_end - pwmValue);
        delay(delayTime);
      }
      if (finalDelay > 0) delay(finalDelay);
    }

    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
    run = false;
  }
}