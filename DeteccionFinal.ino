#include <SPI.h>
#include "ACS712-SOLDERED.h"
#include <SX127XLT.h>
#include <string.h>

//ESTO ES LAS VARIABLES DEL LORA

SX127XLT LT;  //create a library class instance called LT

#define NSS 10                     //select pin on LoRa device
#define NRESET 9                   //reset pin on LoRa device
#define DIO0 2                     //DIO0 pin on LoRa device, used for sensing RX and TX done
#define LED1 8                     //LED used to indicate transmission
#define LORA_DEVICE DEVICE_SX1278  //we need to define the device we are using
#define TXpower 2                  //LoRa transmit power in dBm
#define TXtimeout 5000             //transmit timeout in mS. If 0 return from transmit function after send.

//uint8_t buff[]="hola";  //the payload to send
uint8_t buff[2];
uint16_t PayloadCRC;
uint8_t TXPayloadL;  //this is the payload length sent
uint8_t TXPacketL;

const uint16_t NetworkID = 0x3210;  //NetworkID identifies this connection, needs to match value in receiver

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
bool flag22 = true;
bool envRojo = false;
bool envVerde = false;
bool salida = false;

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
  // Initialize the serial port
  Serial.begin(115200);

  //Setup de lora a continuacion
  lora_setup();

  // Determine midpoint of the ACS712 and noise
  ACS1.autoMidPoint();
  Serial.print("MidPoint: ");
  Serial.print(ACS1.getMidPoint());
  Serial.print(". Noise mV: ");
  Serial.println(ACS1.getNoisemV());
}

void loop() {
  int mA1 = ACS1.mA_AC();
  // Obtenemos el tiempo actual
  unsigned long tiempoActual = millis();

  // Calculamos cuánto tiempo ha transcurrido desde el inicio
  unsigned long tiempoTranscurrido = tiempoActual - tiempoInicio;
  unsigned long tiempoTranscurridoRojo = tiempoActual - tiempoInicioRojo;
  unsigned long tiempoTranscurridoVerder = tiempoActual - tiempoInicioVerde;

  //Obtenemos el valor de la corriente de cada sensor.
  // I1 = sensor1.getCurrentAC();
  //I2 = sensor1.getCurrentAC();
  //I3 = sensor1.getCurrentAC();

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
  //-------------------------------Logica de informacion de encendido de colores------------------------------------------------------


  if (flag1 && flag2) {
    if (!salida) {
      if (mA1 >= promRojoON) {
        flag12 = true;
        if (envRojo == false) {
          strcpy((char*)buff, "R1");
          Serial.println("R1");
          main_lora_send();
          envRojo = true;
        }
        if (tiempoTranscurrido >= tiempoRojo) {
          strcpy((char*)buff, "R0");
          Serial.println("R0");
          main_lora_send();
          envRojo = false;
          // Reiniciamos el tiempo de inicio para comenzar el conteo nuevamente
          tiempoInicioRojo = millis();
          salida = true;
        }
      } /*else if (mA1 < promRojoOFF) {
      Serial.println("Rojo apagado");
      Serial.println("00");
      // buff = "Rojo apagado";
      strcpy((char*)buff, "00");
      //main_lora_send();
      }*/
      
      if (mA1 < sumRojoOnMax && mA1 > promRojoOFF) {
        Serial.println("Foco quemado");
        Serial.println("-");
        strcpy((char*)buff, "-");
        // main_lora_send();
      }
    }
  }

  /*if (I2 > 0.08) {
    flag2 = true;
    //VEr si es necesario controlar el amarillo.
  }

  if (I3 > 0.08) {
    flag3 = true;
    if (envVerde == false) {
      Serial.println("Lampara verde encendida");

      //enviamos tiempo de encendido

      envVerde = true;
    }
    if (tiempoTranscurrido >= tiempoVerde) {
      Serial.println("Lampara roja Apagada");
      //enviamos tiempo de encendido

      envVerde = false;
      // Reiniciamos el tiempo de inicio para comenzar el conteo nuevamente
      tiempoInicioVerde = millis();
    }
  }*/

  //-------------------------------Logica de varificacion de encendido de lamparas------------------------------------------------------

  // Verificamos si ha pasado el tiempo establecido
  if (tiempoTranscurrido >= tiempoEspera) {
    Serial.println(flag12);
    Serial.println(flag22);
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
      //TO DO

      sprintf(n_lamp, "%d", lamp);
      //aca se envia por lora la informacion de la lampara quemada.
      Serial.println("alguna no enciende");
      Serial.println(n_lamp);
      strcpy((char*)buff, n_lamp);
      main_lora_send();
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

  delay(1000);
}

//FUNCIONES DE LORA

void lora_setup() {

  Serial.println();
  Serial.println(F("201_Basic_Reliable_Transmitter Starting"));

  pinMode(LED1, OUTPUT);
  //led_Flash(2, 125);  //two quick LED flashes to indicate program start

  SPI.begin();

  if (LT.begin(NSS, NRESET, DIO0, LORA_DEVICE)) {
    Serial.println(F("LoRa Device found"));
    delay(1000);
  } else {
    Serial.println(F("No LoRa device responding"));
    do {
      digitalWrite(LED1, HIGH);
      // delay(50);
      digitalWrite(LED1, LOW);
      // delay(50);
    } while (1);
  }

  LT.setupLoRa(434000000, 0, LORA_SF7, LORA_BW_125, LORA_CR_4_5, LDRO_AUTO);  //configure frequency and LoRa settings

  Serial.println();
  LT.printModemSettings();  //reads and prints the configured LoRa settings, useful check
  Serial.println();
  LT.printOperatingSettings();  //reads and prints the configured operating settings, useful check
  Serial.println();
  Serial.println();
  LT.printRegisters(0x00, 0x4F);  //print contents of device registers, normally 0x00 to 0x4F
  Serial.println();
  Serial.println(F("Transmitter ready"));
  Serial.println();
}

void main_lora_send() {

  // Serial.println(message);

  Serial.print(F("Entro a enviar el msg"));

  Serial.print(F("Transmit Payload > "));
  TXPayloadL = sizeof(buff);
  Serial.println(TXPayloadL);
  LT.printASCIIArray(buff, TXPayloadL);  //print the payload buffer as ASCII
  Serial.println();
  if (LT.getReliableConfig(NoReliableCRC)) {
    Serial.println(F("Payload CRC check disabled"));
  }
  Serial.flush();

  //now transmit the packet
  digitalWrite(LED1, HIGH);
  //LED on to indicate transmit
  TXPacketL = LT.transmitReliable(buff, TXPayloadL, NetworkID, TXtimeout, TXpower, WAIT_TX);  //will return packet length > 0 if sent OK, otherwise 0 if transmit error

  if (TXPacketL > 0) {
    //if transmitReliable() returns > 0 then transmit was OK
    PayloadCRC = LT.getTXPayloadCRC(TXPacketL);  //read the actual transmitted CRC from the LoRa device buffer
    packet_is_OK();
    Serial.println();
  } else {
    //if transmitReliable() returns 0 there was an error
    packet_is_Error();
    Serial.println();
  }

  digitalWrite(LED1, LOW);
  Serial.println();
}


void packet_is_OK() {
  Serial.print(F("LocalNetworkID,0x"));
  Serial.print(NetworkID, HEX);
  Serial.print(F(",TransmittedPayloadCRC,0x"));  //print CRC of transmitted payload
  Serial.print(PayloadCRC, HEX);
}


void packet_is_Error() {
  Serial.print(F("SendError"));
  LT.printIrqStatus();       //prints the text of which IRQs set
  LT.printReliableStatus();  //print the reliable status
}


void led_Flash(uint16_t flashes, uint16_t delaymS) {
  uint16_t index;
  for (index = 1; index <= flashes; index++) {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}
