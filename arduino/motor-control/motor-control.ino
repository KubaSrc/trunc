const int pwmPin = 11;    // PWM-capable pin
const int in1 = 10;
const int in2 = 9;
const unsigned long delayDuration = 5000;  // Maximum ramp-up time
unsigned long pwm_start = 200;
unsigned long pwm_end  = 255;

// Hard-coded command value (duration) and flag; positive value means forward direction.
float commandNum;      // Duration in ms (change as needed)
float absCommandNum;   // Absolute value (for calculations)
bool run = false;              // Set to true to run the motor command once

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
  absCommandNum = max(commandNum, commandNum * -1);
  Serial.println(commandNum);
  Serial.println(absCommandNum);
  run = true;}

  if (run) {
    // Determine the ramp duration (the smaller of the command or delayDuration)
    unsigned long maxDuration = min((unsigned long)absCommandNum, delayDuration);
    // Define how many steps the ramp should have.
    const int increment = 15;  
    unsigned long delayTime = maxDuration / increment;
    unsigned long pwmIncrement = (pwm_end - pwm_start) / increment;
    // Ensure that the PWM increment is at least 1 to avoid an infinite loop.
    if (pwmIncrement == 0) {
      pwmIncrement = 1;
    }
    long finalDelay = absCommandNum - maxDuration;
    
    Serial.print("finalDelay: ");
    Serial.println(finalDelay);

    Serial.print("absCommandNum: ");
    Serial.println(absCommandNum);

    // Check the command sign to set the motor direction.
    if (commandNum > 0) {
      Serial.println("Running forward.");
      // Forward direction: in1 LOW, in2 HIGH
      int pwmValue = pwm_start;
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      for (; pwmValue <= pwm_end; pwmValue += pwmIncrement) {
        analogWrite(pwmPin, pwmValue);
        delay(delayTime);
      }
      if (finalDelay > 0) {
        delay(finalDelay);
      }
      // Turn the motor off.
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      analogWrite(pwmPin, 0);
    } 
    if (commandNum < 0) {
      Serial.println("Running backward.");
      // Reverse direction: in1 HIGH, in2 LOW
      int pwmValue = pwm_end - pwm_start;  
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      for (; pwmValue >= 0; pwmValue -= pwmIncrement) {
        analogWrite(pwmPin, pwm_end - pwmValue);
        delay(delayTime);
      }
      if (finalDelay > 0) {
        delay(finalDelay);
      }
      // Turn the motor off.
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      analogWrite(pwmPin, 0);
    }
    
    // Command processing complete; set run to false to prevent re-running.
    run = false;
  }
}
