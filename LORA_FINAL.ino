#include <SPI.h>                                //the lora device is SPI based so load the SPI library                                         
#include <SX127XLT.h>                           //include the appropriate library  
#include <string.h>

SX127XLT LT;                                    //create a library class instance called LT

#define NSS 10                                  //select pin on LoRa device
#define NRESET 9                                //reset pin on LoRa device
#define DIO0 2                                  //DIO0 pin on LoRa device, used for sensing RX and TX done 
#define LORA_DEVICE DEVICE_SX1278               //we need to define the device we are using
#define TXpower 17                              //LoRa transmit power in dBm

//ACA SETEO TODO LO DE RECEPCION

#define RXtimeout 10000                         //receive timeout in mS.  

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

//ACA SETEO TODO LO DE EMISOR


void loop()
{
  Serial.print(F("Entro en el loop"));
  Serial.flush();

  //LT.printASCIIPacket(RXBUFFER, sizeof(RXBUFFER));                        //print the buffer (the sent packet) as ASCII

  /* //Transmit a packet
  if (!LT.transmit(RXBUFFER, sizeof(RXBUFFER), 10000, TXpower, WAIT_TX))  //will return 0 if transmit error
  {
    Serial.print(F("FALLO LA TRANSMICION!"));
    LT.printIrqStatus();                                          //print IRQ status, indicates why packet transmit fail
    Serial.println();
  } */

  Serial.println();
  Serial.print(F("ESPERANDO ALGUN PAQUETE..."));
  Serial.println();

  //ACA ESTA EL RECEPTOR
  if (LT.receiveReliable(RXBUFFER, RXBUFFER_SIZE, NetworkID, RXtimeout, WAIT_RX))      //wait for a packet to arrive with 2 second (2000mS) timeout
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
    Serial.print(F("FALLO LA RECEPCION!"));
    LT.printIrqStatus();                                          //print IRQ status, why did packet receive fail
    Serial.println();
  }

  Serial.println();
  delay(500);                                                     //have a delay between packets
}


void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("Lora que envia y recibe"));

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

  Serial.print(F("Lora configurado!"));
  Serial.println();
  Serial.println();
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