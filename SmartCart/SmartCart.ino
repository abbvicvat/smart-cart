#include <Wire.h>

const byte ENA = 13;
const byte IN1 = 5;
const byte IN2 = 4;

const byte ENB = 14;
const byte IN3 = 0;
const byte IN4 = 2;

const byte PWMPins[] = {ENA, ENB};
const byte dirPins[] = {IN1, IN2, IN3, IN4};
const byte dirPinsA[] = {IN1, IN2};
const byte dirPinsB[] = {IN3, IN4};

template <class T, size_t N> constexpr size_t len(const T(&)[N]) { return N; }

int speedA = 0, speedB = 0;

void setup() {
  Serial.begin(115200);
  for (byte pin : PWMPins) pinMode(pin, OUTPUT);
  for (byte pin : dirPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
}

int R = 1, L = -1;

void loop() {
  drive(200, 200);
  delay(5000);
  drive(-200, -200);
  delay(5000);
}

void drive(int mspeedA, int mspeedB){
  //for (byte pin : PWMPins) analogWrite(pin, abs(mspeed));
  analogWrite(ENA, abs(mspeedA));
  analogWrite(ENB, abs(mspeedB));
  //for (int i = 0; i<len(dirPins); i++) digitalWrite(dirPins[i], mspeed>0 ? !(i%2):i%2);
  for (int i = 0; i<len(dirPinsA); i++) digitalWrite(dirPinsA[i], mspeedA>0 ? !(i%2):i%2);
  for (int i = 0; i<len(dirPinsB); i++) digitalWrite(dirPinsB[i], mspeedB>0 ? !(i%2):i%2);
}


void stopMoving(){
  for (byte pin : dirPins) digitalWrite(pin, LOW);
}