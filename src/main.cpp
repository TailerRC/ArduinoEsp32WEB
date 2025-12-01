#include <Arduino.h>
#include <Servo.h>
#include <DHT.h>

// --- Sensor Ultrasónico ---
int inches = 0;
int cm = 0;
int posServo = 0;

// --- Sensor LDR (luz) ---
const int LDR_PIN = A0;  // Pin analógico para el LDR
const int UMBRAL_LUZ = 500;  // Umbral para determinar día/noche (ajustar según necesidad)

// --- LEDs controlados por ESP32 ---
const int LED1_PIN = 3;  // LED 1 en pin 3
const int LED2_PIN = 5;  // LED 2 en pin 5
bool led1Estado = false;
bool led2Estado = false;

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
bool vibracionActiva = false; // Mantener estado de vibración para el ESP32
unsigned long vibracionActivaHasta = 0; // Tiempo hasta que se desactiva
const unsigned long vibracionDuracion = 5000; // Mostrar vibración activa por 5 segundos

// --- Variables para envío a ESP32 ---
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 2000;

// --- Variables de control de aprobación ---
bool esperandoAprobacion = false;
bool servoAprobado = false;

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
void enviarDatosESP32(float temp, int distancia, int luz, int esDeNoche, int vibracion, int solicitud) {
  Serial.print("{\"temp\":");
  Serial.print(temp, 1);
  Serial.print(",\"dist\":");
  Serial.print(distancia);
  Serial.print(",\"luz\":");
  Serial.print(luz);
  Serial.print(",\"noche\":");
  Serial.print(esDeNoche);  // 1 = noche, 0 = día
  Serial.print(",\"vib\":");
  Serial.print(vibracion);
  Serial.print(",\"solicitud\":");
  Serial.print(solicitud);
  Serial.println("}");
}

// Función para verificar respuesta del ESP32
void verificarRespuestaESP32() {
  if (Serial.available()) {
    String respuesta = Serial.readStringUntil('\n');
    respuesta.trim();
    
    if (respuesta == "APROBAR") {
      servoAprobado = true;
      esperandoAprobacion = false;
    } else if (respuesta == "RECHAZAR") {
      servoAprobado = false;
      esperandoAprobacion = false;
    } else if (respuesta == "LUZ1_ON") {
      digitalWrite(LED1_PIN, HIGH);
      led1Estado = true;
    } else if (respuesta == "LUZ1_OFF") {
      digitalWrite(LED1_PIN, LOW);
      led1Estado = false;
    } else if (respuesta == "LUZ2_ON") {
      digitalWrite(LED2_PIN, HIGH);
      led2Estado = true;
    } else if (respuesta == "LUZ2_OFF") {
      digitalWrite(LED2_PIN, LOW);
      led2Estado = false;
    }
  }
}

// Función para mover el servo
void moverServo() {
  for (posServo = 10; posServo <= 90; posServo++) {
    servo_9.write(posServo);
    delay(40);
  }
  delay(2500);
  for (posServo = 90; posServo >= 10; posServo--) {
    servo_9.write(posServo);
    delay(40);
  }
  servoAprobado = false; // Resetear después de mover
}

void setup() {
  Serial.begin(9600);

  servo_9.attach(9, 600, 1750);
  servo_9.write(10);

  pinMode(LDR_PIN, INPUT);
  pinMode(SPEAKER_PIN, OUTPUT);
  
  // Inicializar LEDs
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);

  dht.begin();

  pinMode(pinVibracion, INPUT);

  delay(1000);
}

void loop() {
  // Verificar respuesta del ESP32
  verificarRespuestaESP32();
  
  // --- Sensor ultrasónico ---
  cm = 0.01723 * readUltrasonicDistance(7, 6);
  inches = cm / 2.54;

  // --- Lógica del servo con aprobación ---
  if (cm > 0 && cm <= 20) {
    if (!esperandoAprobacion && !servoAprobado) {
      // Solicitar aprobación
      esperandoAprobacion = true;
    }
  } else {
    // Si el objeto se aleja, cancelar solicitud
    if (esperandoAprobacion) {
      esperandoAprobacion = false;
    }
    servo_9.write(10);
  }
  
  // Si fue aprobado, mover el servo
  if (servoAprobado) {
    moverServo();
  }

  // --- Sensor LDR (luz) ---
  int valorLuz = analogRead(LDR_PIN);
  int esDeNoche = (valorLuz < UMBRAL_LUZ) ? 1 : 0;  // Menor luz = noche
  
  // Apagar LEDs automáticamente cuando es de día
  if (esDeNoche == 0 && (led1Estado || led2Estado)) {
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    led1Estado = false;
    led2Estado = false;
  }

  // --- Temperatura con DHT11 ---
  float temperaturaC = dht.readTemperature();

  // --- Alarma por temperatura ---
  if (temperaturaC >= 30.0) {
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

  // --- Alarma por vibración ---
  int estadoVibracion = digitalRead(pinVibracion);
  unsigned long currentTime = millis();
  
  // Detectar nueva vibración
  if (estadoVibracion == HIGH && (currentTime - lastVibrationTime) > vibrationDebounce) {
    lastVibrationTime = currentTime;
    vibracionActiva = true;
    vibracionActivaHasta = currentTime + vibracionDuracion; // Mantener activa por 5 segundos
    
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
  
  // Desactivar vibración después del tiempo establecido
  if (vibracionActiva && currentTime > vibracionActivaHasta) {
    vibracionActiva = false;
  }

  // --- ENVÍO DE DATOS AL ESP32 ---
  if (currentTime - lastSendTime >= sendInterval) {
    lastSendTime = currentTime;
    
    if (!isnan(temperaturaC)) {
      // Enviar con estado de solicitud, luz y vibración
      enviarDatosESP32(temperaturaC, cm, valorLuz, esDeNoche, vibracionActiva ? 1 : 0, esperandoAprobacion ? 1 : 0);
    }
  }

  delay(200);
}