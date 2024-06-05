#include <TaskScheduler.h>
#include <SPI.h>       //the lora device is SPI based so load the SPI library
#include <SX127XLT.h>  //include the appropriate library
#include <string.h>
#include "ACS712-SOLDERED.h"

SX127XLT LT;  //create a library class instance called LT

#define NSS 10                     //select pin on LoRa device
#define NRESET 9                   //reset pin on LoRa device
#define DIO0 2                     //DIO0 pin on LoRa device, used for sensing RX and TX done
#define LORA_DEVICE DEVICE_SX1278  //we need to define the device we are using
#define TXpower 17

//ACA SETEO TODO LO DE RECEPCION

#define RXtimeout 1000  //receive timeout in mS.

const uint8_t RXBUFFER_SIZE = 251;  //RX buffer size, set to max payload length of 251, or maximum expected payload length
uint8_t RXBUFFER[RXBUFFER_SIZE];    //create the buffer that received packets are copied into
uint8_t TXBUFFER[RXBUFFER_SIZE];
uint8_t RXPacketL;              //stores length of packet received
uint8_t RXPayloadL;             //stores length of payload received
uint8_t PacketOK;               //set to > 0 if packetOK
int16_t PacketRSSI;             //stores RSSI of received packet
uint16_t LocalPayloadCRC;       //locally calculated CRC of payload
uint16_t RXPayloadCRC;          //CRC of payload received in packet
uint16_t TransmitterNetworkID;  //the NetworkID from the transmitted and received packet

const uint16_t NetworkID = 0x3210;  //NetworkID identifies this connection, needs to match value in transmitter
const uint16_t NetworkID2 = 0x3211;


//ACA SE CONFIGURA LOOP EN PARALELO

Scheduler runner;

// Declaración de las funciones de callback
void loop1Callback();
void loop2Callback();

// Crear tareas con intervalos específicos y el número de ejecuciones (TASK_FOREVER para ejecutarlas indefinidamente)
Task task1(0, TASK_FOREVER, &loop1Callback);
Task task2(5000, TASK_FOREVER, &loop2Callback);

// Declare a ACS712 object
ACS712 ACS1(A0, ACS712_5A);
ACS712 ACS2(A1, ACS712_5A);
ACS712 ACS3(A2, ACS712_5A);

//HASTA ACA VARIABLES DE LORA

//----------------Declaramos variables propias de la deteccion de corriente.-------------
int valorRojo = 0;
int valorAmarillo = 0;
int valorverde = 0;

int countR = 0;
int countA = 0;
int countV = 0;

int promRojoON = 0;
int promAmarilloON = 0;
int promVerdeON = 0;

int promRojoOFF = 0;
int promAmarilloOFF = 0;
int promVerdeOFF = 0;

int sumRojoOFF = 0;
int sumAmarilloOFF = 0;
int sumVerdeOFF = 0;

int sumRojoON = 0;
int sumAmarilloON = 0;
int sumVerdeON = 0;

int sumRojoOnMax = 0;

bool flag1 = false;
bool flag2 = false;
bool flag3 = false;

int ii = 0;
int i = 0;

//---------------Declaramos variables propias de la logica de transmision---------------------

int counter = 0;  //Cantidad de veces que entra al loop

//Banderas que indican los estados de los focos.
bool flag12 = false;
bool flag22 = false;
bool envRojo = false;
bool envVerde = false;
bool salida = false;
bool rojo;

//Variable de error de lampara.
int lamp = 0;
char n_lamp[10];

//Definimos varibles de corriente para cada sensor.
float I1;
float I2;
float I3;

// Variable para almacenar el tiempo transcurrido
unsigned long tiempoInicio;
unsigned long tiempoInicioRojo;
unsigned long tiempoInicioVerde;

const unsigned long tiempoEspera = 80000;  // 130 segundos en milisegundos

//Definimos los tiempos de espera de los colores principales.
const unsigned long tiempoRojo = 40000;
const unsigned long tiempoVerde = 15000;

void setup() {
  Serial.begin(115200);
  SPI.begin();
  while (!Serial) {
    ;  // Espera a que el puerto serial se conecte. Solo necesario para placas como Arduino Leonardo
  }

  if (LT.begin(NSS, NRESET, DIO0, LORA_DEVICE)) {
    Serial.println(F("El dispositivo arranco bien!"));
    delay(1000);
  } else {
    Serial.println(F("No se pudo arrancar..."));
    while (1)
      ;
  }

  LT.setupLoRa(434000000, 0, LORA_SF7, LORA_BW_125, LORA_CR_4_5, LDRO_AUTO);  //configure frequency and LoRa settings

  Serial.print(F("Lora configurado!"));
  Serial.println();
  Serial.println();

  Serial.println("Iniciando tareas...");

  // Agregar tareas al scheduler
  runner.init();
  runner.addTask(task1);
  runner.addTask(task2);

  // Habilitar tareas
  task1.enable();
  task2.enable();

  Serial.println("Tareas habilitadas.");
}

void loop() {
  runner.execute();
}

// Definición de las funciones de callback
void loop1Callback() {

  Serial.println(F("Esperando algun paquete..."));
  if (LT.receiveReliable(RXBUFFER, RXBUFFER_SIZE, NetworkID, RXtimeout, WAIT_RX))  //Espera el paquete por el tiempo del RXTimeout
  {
    RXPacketL = LT.readRXPacketL();    //get the received packet length
    RXPayloadL = RXPacketL - 4;        //payload length is always 4 bytes less than packet length
    PacketRSSI = LT.readPacketRSSI();  //read the received packets RSSI value
    packet_is_OK();
    Serial.println();

    //COMO SE RECIBIO OKEY SE VA A TRANSMITIR EL PAQUETE
    Serial.print(F("Se intenta transmitir"));

    strcat((char*)RXBUFFER, NetworkID2);

    if (LT.transmitReliable(RXBUFFER, RXPayloadL, NetworkID2, RXtimeout, TXpower, WAIT_TX)) {
      Serial.print(F("Se envio con exito la cosa"));
    }

  } else {
    Serial.print(F("No se pudo recibir nada: "));
    LT.printIrqStatus();  //print IRQ status, why did packet receive fail
    Serial.println();
  }
}

void loop2Callback() {
  Serial.println("Loop 2 ejecutándose");

  unsigned long tiempoActual = millis();
  // Calculamos cuánto tiempo ha transcurrido desde el inicio
  unsigned long tiempoTranscurrido = tiempoActual - tiempoInicio;
  unsigned long tiempoTranscurridoRojo = tiempoActual - tiempoInicioRojo;
  unsigned long tiempoTranscurridoVerder = tiempoActual - tiempoInicioVerde;
  Serial.println(F("Entro a la otra funcion"));
  Serial.println(F(""));
  int mA1 = ACS1.mA_AC();
  //---------------------------------------Calculamos valores de encendido y apagado de las lamparas-------------------------------------
  if (!flag1 && ii <= 6) {
    Serial.println("Grabando valor apagado");
    if (mA1 < 70) {
      ii = ii + 1;
      sumRojoOFF = sumRojoOFF + mA1;
      if (ii == 6) {
        flag1 = true;
        promRojoOFF = ((sumRojoOFF / 6) * 1.3);
        Serial.println("Sacamos el valor promedio de apagado");
        Serial.println(promRojoOFF);
      }
    }
  }

  if (!flag2 && i <= 6) {
    Serial.println("Grabando valor encendido");
    if (mA1 > 80) {
      i = i + 1;
      sumRojoON = sumRojoON + mA1;
      if (i == 6) {
        flag2 = true;
        promRojoON = (sumRojoON / 6);
        Serial.println("Sacamos el valor promedio prendido");
        Serial.println(promRojoON);
        sumRojoOnMax = mA1 - (mA1 * 0.15);
      }
    }
  }
  if (flag1 && flag2) {

    if (!salida) {
      if (mA1 >= promRojoON) {
        flag12 = true;
        rojo = true;
        if (envRojo == false) {
          strcpy((char*)TXBUFFER, "N1-R1");
          Serial.println("N1-R1");
          if (LT.transmitReliable(TXBUFFER, RXPayloadL, NetworkID2, RXtimeout, TXpower, WAIT_TX)) {
            Serial.print(F("Se envio con exito la cosa"));
          }

          envRojo = true;
        }
        /*if (tiempoTranscurrido >= tiempoRojo) {
          strcpy((char*)buff, "N1-R0");
          Serial.println("N1-R0");
          main_lora_send();
          envRojo = false;
          // Reiniciamos el tiempo de inicio para comenzar el conteo nuevamente
          tiempoInicioRojo = millis();
          salida = true;
        }*/
      } else if (mA1 < promRojoOFF) {
        Serial.println("Rojo apagado");
        strcpy((char*)TXBUFFER, "N1-R0");
        Serial.println("N1-R0");
        if (LT.transmitReliable(TXBUFFER, RXPayloadL, NetworkID2, RXtimeout, TXpower, WAIT_TX)) {
          Serial.print(F("Se envio con exito la cosa"));
        }
        envRojo = false;
        salida = true;
        rojo = false;
      }

      if (mA1 < sumRojoOnMax && mA1 > promRojoOFF) {
        Serial.println("Foco quemado");
        Serial.println("-");
        strcpy((char*)TXBUFFER, "-");
        // main_lora_send();
      }
    }
  }
  //-------------------------------Logica de varificacion de encendido de lamparas------------------------------------------------------

  // Verificamos si ha pasado el tiempo establecido
  if (tiempoTranscurrido >= tiempoEspera) {
    if (flag12 == false) {
      Serial.println("Lampara 1 no funciona");
      lamp = 1;
    }
    if (flag22 == false) {
      Serial.println("Lampara 2 no funciona");
      lamp = 2;
    }
    /*if (flag32 == false) {
      Serial.println("Lampara 3 no funciona");
      lamp = 3;
    }*/

    if (lamp != 0) {
      sprintf(n_lamp, "%d", lamp);
      //aca se envia por lora la informacion de la lampara quemada.
      Serial.println("alguna no enciende");
      Serial.println(n_lamp);
      strcpy((char*)TXBUFFER, n_lamp);
      if (LT.transmitReliable(TXBUFFER, RXPayloadL, NetworkID2, RXtimeout, TXpower, WAIT_TX)) {
        Serial.print(F("Se envio con exito la cosa"));
      }
    }
    // reseteamos valores de banderas
    flag12 = false;
    flag22 = false;
    salida = false;
    //flag3 = false;
    lamp = 0;

    // Reiniciamos el tiempo de inicio para comenzar el conteo nuevamente
    tiempoInicio = millis();
  }
}

void packet_is_OK() {
  Serial.print(F("PAYLOAD RECIBIDO OK"));
  Serial.println();
  LT.printASCIIPacket(RXBUFFER, RXPayloadL);

  if (LT.getReliableConfig(NoReliableCRC)) {
    Serial.println(F("Payload CRC check disabled"));
  }
  //printPacketDetails();
  Serial.println();
}
