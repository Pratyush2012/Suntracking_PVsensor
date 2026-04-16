#include <Servo.h>

const uint8_t PIN_LDR_TL     = A0;
const uint8_t PIN_LDR_TR     = A1;
const uint8_t PIN_LDR_BL     = A2;
const uint8_t PIN_LDR_BR     = A3;
const uint8_t PIN_SERVO_PAN  =  9;
const uint8_t PIN_SERVO_TILT = 10;

const int16_t  THRESHOLD     = 30;
const uint8_t  SERVO_STEP    = 1;
const uint16_t LOOP_DELAY_MS = 200;

const uint8_t PAN_MIN  =   0, PAN_MAX  = 180;
const uint8_t TILT_MIN =  30, TILT_MAX = 150;
const uint8_t PAN_INIT =  90, TILT_INIT = 90;

// ── Global state ───────────────────────────────────────────────────────────────
Servo servoPan;
Servo servoTilt;
int16_t panPos  = PAN_INIT;
int16_t tiltPos = TILT_INIT;

// Readings as plain globals — avoids struct-in-signature TinkerCad bug
int16_t ldr_tl, ldr_tr, ldr_bl, ldr_br;

// ── Helpers ────────────────────────────────────────────────────────────────────
int16_t median3(int16_t a, int16_t b, int16_t c) {
    if (a > b) { int16_t t = a; a = b; b = t; }
    if (b > c) { int16_t t = b; b = c; c = t; }
    if (a > b) { int16_t t = a; a = b; b = t; }
    return b;
}

int16_t sampleLDR(uint8_t pin) {
    return median3(analogRead(pin), analogRead(pin), analogRead(pin));
}

void readAllLDRs() {
    ldr_tl = sampleLDR(PIN_LDR_TL);
    ldr_tr = sampleLDR(PIN_LDR_TR);
    ldr_bl = sampleLDR(PIN_LDR_BL);
    ldr_br = sampleLDR(PIN_LDR_BR);
}

void updateServos() {
    int16_t diff_H = (ldr_tl + ldr_bl) - (ldr_tr + ldr_br);
    int16_t diff_V = (ldr_tl + ldr_tr) - (ldr_bl + ldr_br);

    if (diff_H >  THRESHOLD) panPos -= SERVO_STEP;
    if (diff_H < -THRESHOLD) panPos += SERVO_STEP;
    if (diff_V >  THRESHOLD) tiltPos += SERVO_STEP;
    if (diff_V < -THRESHOLD) tiltPos -= SERVO_STEP;

    panPos  = constrain(panPos,  PAN_MIN,  PAN_MAX);
    tiltPos = constrain(tiltPos, TILT_MIN, TILT_MAX);

    servoPan.write(panPos);
    servoTilt.write(tiltPos);
}

void printDebug() {
    int16_t dH = (ldr_tl + ldr_bl) - (ldr_tr + ldr_br);
    int16_t dV = (ldr_tl + ldr_tr) - (ldr_bl + ldr_br);

    Serial.print("TL="); Serial.print(ldr_tl);
    Serial.print(" TR="); Serial.print(ldr_tr);
    Serial.print(" BL="); Serial.print(ldr_bl);
    Serial.print(" BR="); Serial.print(ldr_br);
    Serial.print(" | dH="); Serial.print(dH);
    Serial.print(" dV="); Serial.print(dV);
    Serial.print(" | PAN="); Serial.print(panPos);
    Serial.print("deg TILT="); Serial.print(tiltPos);
    Serial.println("deg");
}

void setup() {
    Serial.begin(9600);
    Serial.println("Sun Tracker - booting...");
    servoPan.attach(PIN_SERVO_PAN);
    servoTilt.attach(PIN_SERVO_TILT);
    servoPan.write(PAN_INIT);
    servoTilt.write(TILT_INIT);
    delay(500);
    Serial.println("Ready. Adjust photoresistor sliders to simulate sunlight.");
}

void loop() {
    readAllLDRs();
    updateServos();
    printDebug();
    delay(LOOP_DELAY_MS);
}
