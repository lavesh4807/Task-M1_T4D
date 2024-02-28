const int ledPin1 = 8;
const int ledPin2 = 9;
const int ledPin3 = 10;
const int ledTimerPin = 12;

const int pirInput1 = 2;
const int pirInput2 = 4;
const int pirInput3 = 3;

int pir1Detect = LOW;
int pir3Detect = LOW;
int pir2Detect = LOW;

void setup() {
  // Outputs from the LEDs
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);

  pinMode(ledTimerPin, OUTPUT);

  // Inputs from the sensors
  pinMode(pirInput1, INPUT);
  pinMode(pirInput2, INPUT);
  pinMode(pirInput3, INPUT);

  Serial.begin(9600);  // Initialize serial communication
  Serial.println("Setup complete.");

  // Attaching interruptions for the PIR sensors
  attachInterrupt(digitalPinToInterrupt(pirInput1), motionPir1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pirInput3), motionPir3, CHANGE);

  pciSetup(4);
  startTimer(1);
}

void loop() {
  // Your main loop code can go here
}

void pciSetup(byte pin) {
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));
  PCIFR |= bit(digitalPinToPCICRbit(pin));
  PCICR |= bit(digitalPinToPCICRbit(pin));
}

ISR(PCINT2_vect) {
  motionPir2();
}

void startTimer(double timerFrequency) {
  noInterrupts();

  uint16_t ocrValue = (uint16_t)(F_CPU / 1024.0 / timerFrequency - 1);

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = ocrValue;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);

  interrupts();
  Serial.println("Timer started.");
}

ISR(TIMER1_COMPA_vect) {
  digitalWrite(ledTimerPin, digitalRead(ledTimerPin) ^ 1);
  Serial.println("Timer interrupt!");
}

void motionPir1() {
  int val = digitalRead(pirInput1);
  if (val == HIGH) {
    digitalWrite(ledPin1, HIGH);
    if (pir1Detect == LOW) {
      pir1Detect = HIGH;
      Serial.println("Motion detected by PIR 1");
    }
  } else {
    digitalWrite(ledPin1, LOW);
    if (pir1Detect == HIGH) {
      pir1Detect = LOW;
      Serial.println("Motion stopped by PIR 1");
    }
  }
}

void motionPir2() {
  int val = digitalRead(pirInput2);
  if (val == HIGH) {
    digitalWrite(ledPin2, HIGH);
    if (pir2Detect == LOW) {
      pir2Detect = HIGH;
      Serial.println("Motion detected by PIR 2");
    }
  } else {
    digitalWrite(ledPin2, LOW);
    if (pir2Detect == HIGH) {
      pir2Detect = LOW;
      Serial.println("Motion stopped by PIR 2");
    }
  }
}

void motionPir3() {
  int val = digitalRead(pirInput3);
  if (val == HIGH) {
    digitalWrite(ledPin3, HIGH);
    if (pir3Detect == LOW) {
      pir3Detect = HIGH;
      Serial.println("Motion detected by PIR 3");
    }
  } else {
    digitalWrite(ledPin3, LOW);
    if (pir3Detect == HIGH) {
      pir3Detect = LOW;
      Serial.println("Motion stopped by PIR 3");
    }
  }
}
