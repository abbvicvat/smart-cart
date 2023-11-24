#include <Wire.h>
#include "FirebaseESP8266.h" // Includerar ett biblotek som gör att ESP'n kan komunisera med FireBase
#include <ESP8266WiFi.h> // Includerar ett biblotek som gör att ESP'n ska kunna använda WiFi

#define FIREBASE_HOST "https://smart-cart-374c8-default-rtdb.europe-west1.firebasedatabase.app/" // Definerar var datan ska skickas
#define FIREBASE_AUTH "WvYh2kcbqrnsQRiGs8lAe9QZTzNS2AZaR23v63F4" // Lösenord för att se att du är ägaren av FireBasen
#define WIFI_SSID "ABB_Indgym_Guest" // Namnet på wifiet som esp'n ska använda
#define WIFI_PASSWORD "Welcome2abb" // Lösenordet till WiFiet

FirebaseData firebaseData1;

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
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  for (byte pin : PWMPins) pinMode(pin, OUTPUT);
  for (byte pin : dirPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  Serial.println();// printar en ny linje i serial monitorn
  Serial.print("Connected with IP: "); // Printar att WiFi'et har connectat
  Serial.println(WiFi.localIP()); // Skriver ut den locala IP adressen
  Serial.println(); // Skriver ut en ny linje

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH); // Startar FireBase och skickar med var den ska skickas med HOST och AUTH för att veta att det är du som är ägare
  Firebase.reconnectWiFi(true); // Reconnnectar WiFi'et med Firebase (kolla om det är det den gör och skriv bättre!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)

  delay(5000); // Väntar 5000 milisekunder eller 5 sekunder för att vara säker att fire base och wifi'et är kopplade innan vi börjar skicka information

}

int R = 1, L = -1;

void loop() {
  L = Firebase.getInt(firebaseData1, "/left");
  R = Firebase.getInt(firebaseData1, "/right"); 

  drive(L, R);
  
  /*
  drive(200, 200);
  delay(5000);
  drive(-200, -200);
  delay(5000);*/

  
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