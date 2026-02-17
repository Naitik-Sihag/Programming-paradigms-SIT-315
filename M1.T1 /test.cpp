

#define BUTTON_PIN 8
#define PIR_PIN    9

#define TRIG_PIN   3
#define ECHO_PIN   4

#define LED_EVENT  5
#define LED_TIMER  6

// -------------------- Shared Flags --------------------
volatile bool buttonFlag = false;
volatile bool pirFlag    = false;
volatile bool timerFlag  = false;

volatile uint8_t lastPortBState = 0;

// -------------------- System Variables --------------------
unsigned long lastUltrasonicRead = 0;
const unsigned long ultrasonicInterval = 200;  // ms

// -------------------- SETUP --------------------
void setup() {

  Serial.begin(9600);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PIR_PIN, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(LED_EVENT, OUTPUT);
  pinMode(LED_TIMER, OUTPUT);

  setupPCI();
  setupTimer1();

  Serial.println("System Started - PCI + Timer + Ultrasonic \n\nLogic: LED ON when PIR AND (Button OR Object<30cm )\n");
}

// -------------------- PCI SETUP --------------------
void setupPCI() {

  cli();

  PCICR |= (1 << PCIE0);      // Enable PORTB interrupts
  PCMSK0 |= (1 << PCINT0);    // D8
  PCMSK0 |= (1 << PCINT1);    // D9

  lastPortBState = PINB;

  sei();
}

// -------------------- TIMER1 SETUP --------------------
void setupTimer1() {

  cli();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 15624;  // 1 second interval

  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);

  TIMSK1 |= (1 << OCIE1A);

  sei();
}

// -------------------- PCI ISR --------------------
ISR(PCINT0_vect) {

  uint8_t currentState = PINB;
  uint8_t changed = currentState ^ lastPortBState;

  if (changed & (1 << PB0)) {
    if (!(currentState & (1 << PB0))) {
      buttonFlag = true;
    }
  }

  if (changed & (1 << PB1)) {
    if (currentState & (1 << PB1)) {
      pirFlag = true;
    }
  }

  lastPortBState = currentState;
}

// -------------------- TIMER ISR --------------------
ISR(TIMER1_COMPA_vect) {
  timerFlag = true;
}

// -------------------- MAIN LOOP --------------------
void loop() {

  handleButton();
  handlePIR();
  handleTimer();
  handleUltrasonic();
}

// -------------------- EVENT PROCESSING --------------------

void handleButton() {

  if (buttonFlag) {
    buttonFlag = false;

    Serial.println("Button Press Detected (PCI)");
    digitalWrite(LED_EVENT, !digitalRead(LED_EVENT));
  }
}

void handlePIR() {

  if (pirFlag) {
    pirFlag = false;

    Serial.println("Motion Detected (PCI)");
    digitalWrite(LED_EVENT, HIGH);
  }
}

void handleTimer() {

  if (timerFlag) {
    timerFlag = false;

    Serial.println("Timer Interrupt Triggered");
    digitalWrite(LED_TIMER, !digitalRead(LED_TIMER));
  }
}

// -------------------- ULTRASONIC PROCESSING --------------------

void handleUltrasonic() {

  if (millis() - lastUltrasonicRead >= ultrasonicInterval) {

    lastUltrasonicRead = millis();

    long duration;
    float distance;

    // Trigger pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30ms timeout

    distance = duration * 0.034 / 2;

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    if (distance > 0 && distance < 20) {
      Serial.println("Object Close - LED ON");
      digitalWrite(LED_EVENT, HIGH);
    }
  }
}
