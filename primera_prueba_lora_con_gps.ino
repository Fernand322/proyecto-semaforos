#include <TaskScheduler.h>
#include <SPI.h>
#include <SX127XLT.h>
#include <string.h>
#include "ACS712-SOLDERED.h"
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

SX127XLT LT;

#define NSS 10
#define NRESET 9
#define DIO0 2
#define LORA_DEVICE DEVICE_SX1278
#define TXpower 17
#define RXtimeout 1000

const uint8_t RXBUFFER_SIZE = 251;
uint8_t RXBUFFER[RXBUFFER_SIZE];
uint8_t TXBUFFER[RXBUFFER_SIZE];
uint8_t RXPacketL;
uint8_t RXPayloadL;
uint8_t TXPayloadL;
const uint16_t NetworkID = 0x3211;
const uint16_t NetworkID2 = 0x3212;

Scheduler runner;

void loop1Callback();
void loop2Callback();

Task task1(0, TASK_FOREVER, &loop1Callback);
Task task2(0, TASK_FOREVER, &loop2Callback);


ACS712 ACS1(A0, ACS712_5A);
ACS712 ACS2(A1, ACS712_5A);
ACS712 ACS3(A2, ACS712_5A);

int valorRojo = 0;
int valorAmarillo = 0;
int valorverde = 0;

int promRojoON = 0;
int promRojoOFF = 0;
int sumRojoOFF = 0;
int sumRojoON = 0;
int sumRojoOnMax = 0;

bool flag1 = false;
bool flag2 = false;

int mA1 = 0;
int mA2 = 0;
int mA3 = 0;

int ii = 0;
int i = 0;

int counter = 0;

bool envRojo = false;
bool envAmarillo = false;
bool envVerde = false;

const unsigned long tiempoEspera = 50000;
unsigned long tiempoTranscurrido;
unsigned long tiempoActual;
unsigned long tiempoInicio;

// Definir pines para el GPS
SoftwareSerial gpsSerial(4, 3);  // RX, TX
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  SPI.begin();
  while (!Serial) {
    ;
  }

  gpsSerial.begin(9600);

  if (LT.begin(NSS, NRESET, DIO0, LORA_DEVICE)) {
    Serial.println(F("El dispositivo arranco bien!"));
    delay(1000);
  } else {
    Serial.println(F("No se pudo arrancar..."));
    while (1)
      ;
  }

  LT.setupLoRa(434000000, 0, LORA_SF7, LORA_BW_125, LORA_CR_4_5, LDRO_AUTO);

  Serial.print(F("Lora configurado!"));
  Serial.println();
  Serial.println();

  Serial.println("Iniciando tareas...");

  runner.init();
  runner.addTask(task1);
  runner.addTask(task2);
  task1.enable();
  task2.enable();


  Serial.println("Tareas habilitadas.");
}

void loop() {
  runner.execute();
}

void loop1Callback() {

  updateGPS();

  if (LT.receiveReliable(RXBUFFER, RXBUFFER_SIZE, NetworkID, 250, WAIT_RX)) {
    RXPacketL = LT.readRXPacketL();
    RXPayloadL = RXPacketL - 4;
    packet_is_OK();
    Serial.println();

    Serial.print("Hora en Argentina: ");
    Serial.println(getArgentinaTime());

    strcat((char*)RXBUFFER, (char*)&NetworkID2);

    if (LT.transmitReliable(RXBUFFER, RXPayloadL, NetworkID2, 10, TXpower, WAIT_TX)) {
      Serial.print(F("Se envio con exito la cosa"));
      Serial.println();
    }
  } else {
  }

  /*  // Actualizar datos del GPS

  static unsigned long lastTimeCheck = 0;
  if (millis() - lastTimeCheck > 10000) { // Cada 10 segundos
    Serial.print("Hora en Argentina: ");
    Serial.println(getArgentinaTime());
    lastTimeCheck = millis();
  } */
}

void loop2Callback() {
  updateGPS();
  /* static unsigned long lastTimeCheck = 0;
  if (millis() - lastTimeCheck > 1000) {  // Cada 10 segundos
    Serial.print("Hora en Argentina: ");
    Serial.println(getArgentinaTime());
    lastTimeCheck = millis();
  } */
}

void loop3Callback() {
}

// Función para actualizar datos del GPS
void updateGPS() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
}

// Función para obtener la hora en Argentina
String getArgentinaTime() {
  if (gps.time.isValid()) {
    // Obtener hora UTC del GPS
    int hour = gps.time.hour();
    int minute = gps.time.minute();
    int second = gps.time.second();

    // Ajustar a la hora de Argentina (UTC-3)
    hour = (hour - 3 + 24) % 24;

    // Formatear la hora con ceros a la izquierda si es necesario
    char buffer[10];
    snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d", hour, minute, second);

    return String(buffer);
  } else {
    return "Hora no válida";
  }
}

void packet_is_OK() {
  Serial.print(F("PAYLOAD RECIBIDO OK"));
  Serial.println();
  LT.printASCIIPacket(RXBUFFER, RXPayloadL);

  if (LT.getReliableConfig(NoReliableCRC)) {
    Serial.println(F("Payload CRC check disabled"));
  }
  Serial.println();
}