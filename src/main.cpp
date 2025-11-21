#include <Arduino.h>
#include <Servo.h>
#include <DHT.h>

// --- Sensor Ultrasónico ---
int inches = 0;
int cm = 0;
int posServo = 0;

// --- Pines PIR, LED, Servo ---
const int PIR_PIN = 5;
const int LED_PIN = 2;

// --- Sensor DHT11 en PIN 4 ---
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// --- Speaker (alarma) ---
const int SPEAKER_PIN = 8;

// Servo
Servo servo_9;

// --- Pines Sensor vibraciones ---
const int pinVibracion = 12;
unsigned long lastVibrationTime = 0;
const unsigned long vibrationDebounce = 500;

// --- Variables para envío a ESP32 ---
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 2000; // Enviar cada 2 segundos

// Función para medir distancia
long readUltrasonicDistance(int triggerPin, int echoPin) {
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  pinMode(echoPin, INPUT);
  return pulseIn(echoPin, HIGH);
}

// Función para enviar datos JSON al ESP32
void enviarDatosESP32(float temp, int distancia, int pir, int vibracion) {
  // Formato JSON simple para facilitar el parsing
  Serial.print("{\"temp\":");
  Serial.print(temp, 1); // 1 decimal
  Serial.print(",\"dist\":");
  Serial.print(distancia);
  Serial.print(",\"pir\":");
  Serial.print(pir);
  Serial.print(",\"vib\":");
  Serial.print(vibracion);
  Serial.println("}");
}

void setup() {
  Serial.begin(9600); // Comunicación con ESP32

  servo_9.attach(9, 600, 1750);
  servo_9.write(0);

  pinMode(PIR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SPEAKER_PIN, OUTPUT);

  dht.begin();

  pinMode(pinVibracion, INPUT);

  Serial.println("Sistema iniciado...");
  delay(1000);
}

void loop() {
  // --- Sensor ultrasónico ---
  cm = 0.01723 * readUltrasonicDistance(7, 6);
  inches = cm / 2.54;

  // Movimiento del servo según distancia
  if (cm > 0 && cm <= 70) {
    for (posServo = 0; posServo <= 90; posServo++) {
      servo_9.write(posServo);
      delay(40);
    }
    delay(2500);
    for (posServo = 90; posServo >= 0; posServo--) {
      servo_9.write(posServo);
      delay(40);
    }
  } else {
    servo_9.write(0);
  }

  // --- Sensor PIR ---
  int estadoPIR = digitalRead(PIR_PIN);

  if (estadoPIR == HIGH) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  // --- Temperatura con DHT11 ---
  float temperaturaC = dht.readTemperature();

  // --- Alarma por temperatura ---
  if (temperaturaC >= 27.0) {
    for (int i = 0; i < 3; i++) {
      tone(SPEAKER_PIN, 1000);
      delay(400);
      noTone(SPEAKER_PIN);
      delay(150);
      tone(SPEAKER_PIN, 800);
      delay(400);
      noTone(SPEAKER_PIN);
      delay(200);
    }
  } else {
    noTone(SPEAKER_PIN);
  }

  // --- Alarma por vibracion (SW-420) ---
  int estadoVibracion = digitalRead(pinVibracion);
  unsigned long currentTime = millis();
  
  if (estadoVibracion == HIGH && (currentTime - lastVibrationTime) > vibrationDebounce) {
    lastVibrationTime = currentTime;
    
    // Alarma: 3 pulsos
    for (int i = 0; i < 3; i++) {
      tone(SPEAKER_PIN, 1000);
      delay(400);
      noTone(SPEAKER_PIN);
      delay(150);
      tone(SPEAKER_PIN, 800);
      delay(400);
      noTone(SPEAKER_PIN);
      delay(200);
    }
  }

  // --- ENVÍO DE DATOS AL ESP32 ---
  // Enviar datos periódicamente al ESP32
  if (currentTime - lastSendTime >= sendInterval) {
    lastSendTime = currentTime;
    
    // Validar que la temperatura sea válida
    if (!isnan(temperaturaC)) {
      enviarDatosESP32(temperaturaC, cm, estadoPIR, estadoVibracion);
    }
  }

  delay(200);
}