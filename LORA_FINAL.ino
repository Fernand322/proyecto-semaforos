#include <SPI.h>                                //the lora device is SPI based so load the SPI library                                         
#include <SX127XLT.h>                           //include the appropriate library  
#include <string.h>

SX127XLT LT;                                    //create a library class instance called LT

#define NSS 10                                  //select pin on LoRa device
#define NRESET 9                                //reset pin on LoRa device
#define DIO0 2                                  //DIO0 pin on LoRa device, used for sensing RX and TX done 
#define LORA_DEVICE DEVICE_SX1278               //we need to define the device we are using
#define TXpower 17 

//ACA SETEO TODO LO DE RECEPCION

#define RXtimeout 1000                         //receive timeout in mS.  

const uint8_t RXBUFFER_SIZE = 251;              //RX buffer size, set to max payload length of 251, or maximum expected payload length
uint8_t RXBUFFER[RXBUFFER_SIZE];                //create the buffer that received packets are copied into

uint8_t RXPacketL;                              //stores length of packet received
uint8_t RXPayloadL;                             //stores length of payload received
uint8_t PacketOK;                               //set to > 0 if packetOK
int16_t PacketRSSI;                             //stores RSSI of received packet
uint16_t LocalPayloadCRC;                       //locally calculated CRC of payload
uint16_t RXPayloadCRC;                          //CRC of payload received in packet
uint16_t TransmitterNetworkID;                  //the NetworkID from the transmitted and received packet

const uint16_t NetworkID = 0x3210;              //NetworkID identifies this connection, needs to match value in transmitter
const uint16_t NetworkID2 = 0x3211;


const int ledPin = 13;      // Pin del LED
const int sensorPin = A0;   // Pin del sensor analógico

unsigned long previousMillisLED = 0;      // Almacena el tiempo de la última actualización del LED
const long intervalLED = 1000;            // Intervalo para parpadear el LED (1 segundo)

void setup()
{
  Serial.begin(115200);

  SPI.begin();

  if (LT.begin(NSS, NRESET, DIO0, LORA_DEVICE))
  {
    Serial.println(F("El dispositivo arranco bien!"));
    delay(1000);
  }
  else
  {
    Serial.println(F("No se pudo arrancar..."));
    while (1);
  }

  LT.setupLoRa(434000000, 0, LORA_SF7, LORA_BW_125, LORA_CR_4_5, LDRO_AUTO); //configure frequency and LoRa settings

  pinMode(ledPin, OUTPUT);

  Serial.print(F("Lora configurado!"));
  Serial.println();
  Serial.println();
}

void loop() {
  unsigned long currentMillis = millis();

  // Tarea continua: leer el sensor y mostrar el valor
  Serial.println(F("Esperando algun paquete..."));
  if (LT.receiveReliable(RXBUFFER, RXBUFFER_SIZE, NetworkID, RXtimeout, WAIT_RX))      //Espera el paquete por el tiempo del RXTimeout
    {
      RXPacketL = LT.readRXPacketL();               //get the received packet length
      RXPayloadL = RXPacketL - 4;                   //payload length is always 4 bytes less than packet length
      PacketRSSI = LT.readPacketRSSI();             //read the received packets RSSI value
      packet_is_OK();
      Serial.println();


      //COMO SE RECIBIO OKEY SE VA A TRANSMITIR EL PAQUETE
      Serial.print(F("Se intenta transmitir"));

      strcat((char*)RXBUFFER, NetworkID2);

      if(LT.transmitReliable(RXBUFFER, RXPayloadL, NetworkID2, RXtimeout, TXpower, WAIT_TX)){
        Serial.print(F("Se envio con exito la cosa"));
      }

    }
    else
    {
      Serial.print(F("No se pudo recibir nada: "));
      LT.printIrqStatus();                                          //print IRQ status, why did packet receive fail
      Serial.println();
    }

    // Tarea con delay: manejar parpadeo del LED
    if (currentMillis - previousMillisLED >= intervalLED) {
      Serial.println(F("Entro a la otra funcion"));
      Serial.println(F(""));
    }
}

void packet_is_OK()
{
  Serial.print(F("PAYLOAD RECIBIDO OK"));
  Serial.println();
  LT.printASCIIPacket(RXBUFFER, RXPayloadL);
  
  if (LT.getReliableConfig(NoReliableCRC))
  {
    Serial.println(F("Payload CRC check disabled"));
  }
  //printPacketDetails();
  Serial.println();
}
