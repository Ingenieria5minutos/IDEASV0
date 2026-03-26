/*
  IDEAS v1 adaptado a INA240
  ---------------------------------
  Modo A: t_ms,rpm,current_mA
  Modo B: t_ms,rpm

  Configuracion por serial:
    A
    B,10
    B,20
    B,50

  Hardware:
    - INA240 OUT -> A0
    - Encoder canal A -> D2
*/

#include <Arduino.h>

// ===================== CONFIG FIJA ======================
const int ENCODER_PIN = 2;     // D2 = interrupcion externa en Uno/Nano
const int PPR = 70;           // ajustar si hace falta

const unsigned long TS_MODO_A_MS = 100;   // como en la primera version (~10 Hz)

// limites para modo B
const unsigned long TS_B_MIN_MS = 5;
const unsigned long TS_B_MAX_MS = 200;

// ===================== INA240 ===========================
const int PIN_INA240 = A0;

const float VREF = 5.0;        // ADC referencia Arduino UNO/Nano
const int ADC_MAX = 1023;

// Ajusta estas dos segun tu hardware real
const float R_SHUNT = 0.1;     // ohm
const float GAIN = 20.0;       // INA240A1 = 20 V/V
                               // A2 = 50, A3 = 100, A4 = 200

const int N_CALIB = 200;       // muestras para cero
const int N_SAMPLES = 8;       // promedio por lectura

float VZERO = 0.0;

// ===================== ENCODER ==========================
volatile unsigned long pulseCount = 0;

void encoderISR() {
  pulseCount++;
}

// ===================== ESTADO ===========================
char modo = 'A';               // 'A' o 'B'
unsigned long ts_ms = 100;     // periodo actual
unsigned long lastSample = 0;
bool configurado = false;

// =======================================================
// Lee voltaje promedio del INA240
float leerVoltajePromedio(int pin, int muestras) {
  unsigned long suma = 0;

  for (int i = 0; i < muestras; i++) {
    suma += analogRead(pin);
  }

  float adcProm = suma / (float)muestras;
  return adcProm * VREF / ADC_MAX;
}

// =======================================================
// Calibracion de cero del INA240
void calibrarCeroINA240() {
  Serial.println("Calibrando INA240...");
  Serial.println("Deja el motor sin alimentacion.");
  delay(3000);

  VZERO = leerVoltajePromedio(PIN_INA240, N_CALIB);

  Serial.print("VZERO=");
  Serial.println(VZERO, 4);
}

// =======================================================
// Convierte lectura analogica a corriente en mA
float leerCorriente_mA() {
  float vout = leerVoltajePromedio(PIN_INA240, N_SAMPLES);
  float deltaV = vout - VZERO;

  // pequeña zona muerta para evitar ruido
  if (deltaV < 0.003f && deltaV > -0.003f) {
    deltaV = 0.0f;
  }

  float corriente_A = (deltaV / GAIN) / R_SHUNT;
  float corriente_mA = corriente_A * 1000.0f;

  // para esta practica normalmente solo interesa corriente positiva
  if (corriente_mA < 0.0f) corriente_mA = 0.0f;

  return corriente_mA;
}

// =======================================================
void setup() {
  Serial.begin(115200);
  analogReference(DEFAULT);

  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);

  delay(1500); // tiempo para que Python abra el puerto y Arduino reinicie

  calibrarCeroINA240();

  // Esperar configuracion desde Python
  // Formatos:
  // A
  // B,10
  // B,20
  // B,50
  esperarConfiguracion();

  if (modo == 'A') {
    ts_ms = TS_MODO_A_MS;
    Serial.println("t_ms,rpm,current_mA");
  } else {
    Serial.println("t_ms,rpm");
  }
}

// =======================================================
void loop() {
  if (!configurado) return;

  unsigned long now = millis();

  if (now - lastSample >= ts_ms) {
    lastSample = now;

    noInterrupts();
    unsigned long count = pulseCount;
    pulseCount = 0;
    interrupts();

    float dt_s = ts_ms / 1000.0;
    float revs = (float)count / (float)PPR;
    float rpm = (revs / dt_s) * 60.0;

    if (modo == 'A') {
      float current_mA = leerCorriente_mA();

      Serial.print(now);
      Serial.print(",");
      Serial.print(rpm, 2);
      Serial.print(",");
      Serial.println(current_mA, 2);
    } else {
      Serial.print(now);
      Serial.print(",");
      Serial.println(rpm, 2);
    }
  }
}

// =======================================================
void esperarConfiguracion() {
  String linea = "";
  unsigned long t0 = millis();

  while (!configurado) {
    while (Serial.available() > 0) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        if (linea.length() > 0) {
          procesarConfiguracion(linea);
          linea = "";
        }
      } else {
        linea += c;
      }
    }

    // Si pasan muchos segundos sin configuracion, usar A por defecto
    if (millis() - t0 > 8000) {
      modo = 'A';
      ts_ms = TS_MODO_A_MS;
      configurado = true;
    }
  }
}

// =======================================================
void procesarConfiguracion(String s) {
  s.trim();

  if (s.length() == 0) return;

  if (s == "A") {
    modo = 'A';
    ts_ms = TS_MODO_A_MS;
    configurado = true;
    return;
  }

  if (s.charAt(0) == 'B') {
    modo = 'B';

    int coma = s.indexOf(',');
    if (coma > 0) {
      String tsStr = s.substring(coma + 1);
      unsigned long val = tsStr.toInt();

      if (val < TS_B_MIN_MS) val = TS_B_MIN_MS;
      if (val > TS_B_MAX_MS) val = TS_B_MAX_MS;

      ts_ms = val;
    } else {
      ts_ms = 10; // default para modo B
    }

    configurado = true;
    return;
  }
}