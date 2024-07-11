

#include <SPI.h>                                //the LoRa device is SPI based so load the SPI library                                         
#include <SX127XLT.h>                           //include the appropriate library  

SX127XLT LT;                                    //create a library class instance called LT

#define NSS 10                                  //select pin on LoRa device
#define NRESET 9                                //reset pin on LoRa device
#define DIO0 2                                  //DIO0 pin on LoRa device, used for sensing RX and TX done 
#define LED1 8                                  //LED used to indicate transmission
#define LORA_DEVICE DEVICE_SX1278               //we need to define the device we are using
#define TXpower 2                               //LoRa transmit power in dBm
#define TXtimeout 5000                          //transmit timeout in mS. If 0 return from transmit function after send.  

uint8_t buff1[] = "Hello World_5";                 //the payload to send
uint8_t buff2[] = "Hello World_7";                 //the payload to send
uint8_t buff3[] = "Hello World_12";                 //the payload to send
uint16_t PayloadCRC;
uint8_t TXPayloadL;                                            //this is the payload length sent
uint8_t TXPacketL;

const uint16_t NetworkID = 0x3210;                             //NetworkID identifies this connection, needs to match value in receiver

unsigned long previousMillis = 0;
const unsigned long interval5 = 5000; // 5 segundos en milisegundos
const unsigned long interval7 = 7000; // 7 segundos en milisegundos
const unsigned long interval12 = 12000; // 12 segundos en milisegundos

unsigned long previousMillis5 = 0;
unsigned long previousMillis7 = 0;
unsigned long previousMillis12 = 0;

unsigned long flag = 1;

void loop() {
  unsigned long currentMillis = millis();
  
  // Primer intervalo: 5 segundos
  if (currentMillis - previousMillis >= interval5 && flag == 1) {
    Serial.println("Han pasado 5 segundos");
    previousMillis = currentMillis; // Reinicia el contador para el siguiente ciclo de 5 segundos
    flag = 2;
    hacer_esto(buff1);
  }
  
  // Segundo intervalo: 7 segundos
  if (currentMillis - previousMillis >= interval7 && flag == 2) {
    Serial.println("Han pasado 7 segundos");
    previousMillis = currentMillis; // Reinicia el contador para el siguiente ciclo de 7 segundos
    flag = 3;
    hacer_esto(buff2);
  }
  
  // Tercer intervalo: 12 segundos
  if (currentMillis - previousMillis >= interval12 && flag == 3) {
    Serial.println("Han pasado 12 segundos");
    previousMillis = currentMillis; // Reinicia el contador para el siguiente ciclo de 12 segundos
    flag = 1;
    hacer_esto(buff3);
  }
}

void hacer_esto(uint8_t *value){
  TXPayloadL = strlen((char*)value);   // Obtiene la longitud del payload
  LT.printASCIIArray(value, TXPayloadL); //print the payload buffer as ASCII
  Serial.println();
                                         //LED on to indicate transmit
  TXPacketL = LT.transmitReliable(value, TXPayloadL, NetworkID, TXtimeout, TXpower, WAIT_TX);  //will return packet length > 0 if sent OK, otherwise 0 if transmit error

  if (TXPacketL > 0)
  {
    //if transmitReliable() returns > 0 then transmit was OK
    PayloadCRC = LT.getTXPayloadCRC(TXPacketL);  //read the actual transmitted CRC from the LoRa device buffer
    packet_is_OK();
    Serial.println();
  }
  Serial.println();
}

void packet_is_OK() {
  Serial.print(F("LocalNetworkID,0x"));
  Serial.print(NetworkID, HEX);
  Serial.print(F(",TransmittedPayloadCRC,0x")); //print CRC of transmitted payload
  Serial.print(PayloadCRC, HEX);
}




void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("201_Basic_Reliable_Transmitter Starting"));
                                    //two quick LED flashes to indicate program start

  SPI.begin();

  if (LT.begin(NSS, NRESET, DIO0, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    delay(1000);
  }
  else
  {
    Serial.println(F("No LoRa device responding"));
    do
    {
      digitalWrite(LED1, HIGH);
      delay(50);
      digitalWrite(LED1, LOW);
      delay(50);
    } while (1);
  }

  LT.setupLoRa(434000000, 0, LORA_SF7, LORA_BW_125, LORA_CR_4_5, LDRO_AUTO); //configure frequency and LoRa settings


}
