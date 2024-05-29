/**
 **************************************************
 *
 * @file        ACS712_5_AC.ino
 * @brief       Example for measure AC current
 *              https://github.com/e-radionicacom/Soldered-ACS712-Current-Sensor-Arduino-Library
 *              
 *
 *
 *
 * @authors    Rob Tillaart
 * Modified by: Soldered for use with https://solde.red/333146
 *
 ***************************************************/

//Connecting diagram
//Breakout      Arduino
//|-------------|
//OUT-----------A0
//GND-----------GND
//VCC-----------5V

#include "ACS712-SOLDERED.h"

// Declare a ACS712 object
ACS712 ACS1(A0, ACS712_5A);
ACS712 ACS2(A1, ACS712_5A);
ACS712 ACS3(A2, ACS712_5A);

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

int ii = 0;
int i = 0;

void setup() {
  // Initialize the serial port
  Serial.begin(115200);


  // Determine midpoint of the ACS712 and noise
  ACS1.autoMidPoint();
  Serial.print("MidPoint: ");
  Serial.print(ACS1.getMidPoint());
  Serial.print(". Noise mV: ");
  Serial.println(ACS1.getNoisemV());
}


void loop() {
  int mA1 = ACS1.mA_AC();
  // Serial.println(mA1);
  //Calculamos los promedios de encendido y apagado.
  if (!flag1 && ii <= 6) {
    Serial.println("Grabando valor apagado");
    //Serial.println("entro en la primer condicion");
    //65 es el ruido generado sin la circulacion de corriente.
    if (mA1 < 70) {
      ii = ii + 1;
     // Serial.println(ii);

      sumRojoOFF = sumRojoOFF + mA1;
      // int mA2 = ACS2.mA_AC();
      //sumAmarilloOFF = sumAmarilloOFF + mA2;
      //int mA3 = ACS3.mA_AC();
      //sumVerdeOFF = sumVerdeOFF + mA3;
      if (ii == 6) {
        flag1 = true;
        promRojoOFF = ((sumRojoOFF / 6) * 1.3);
        Serial.println("Sacamos el valor promedio de apagado");
        Serial.println(promRojoOFF);
      }



      // promAmarilloOFF = sumAmarilloOFF / 6;
      //promVerdeOFF = sumVverdeOFF / 6;
    }
  }

  if (!flag2 && i <= 6) {
    Serial.println("Grabando valor encendido");
    if (mA1 > 80) {
      i = i + 1;

       sumRojoON = sumRojoON + mA1;
      /* int mA2 = ACS2.mA_AC();
        sumAmarilloON = sumAmarilloON + mA2;
        int mA3 = ACS3.mA_AC();
        */
      //sumVerdeON = sumVerdeON + mA3;
      if (i == 6) {
        flag2 = true;
        promRojoON = (sumRojoON / 6) ;
        Serial.println("Sacamos el valor promedio prendido");
        Serial.println(promRojoON);
        sumRojoOnMax =  mA1 - (mA1 * 0.15);
      }



      /* promAmarilloON = sumAmarilloON / 6;
      promVerdeON = sumVverdeON / 6;*/
    }
  }
  
  //Serial.println(sumRojoOnMax);
  if (flag1 && flag2) {


    if (mA1 >= promRojoON) {
      Serial.println("Rojo encendido");
     // Serial.println(promRojoON);
    } else if (mA1 < promRojoOFF) {
      Serial.println("Rojo apagado");
     // Serial.println(promRojoOFF);
    }
    if (mA1 < sumRojoOnMax  && mA1 > promRojoOFF) {
      Serial.println("Foco quemado");
     // Serial.println(sumRojoOnMax);
      //Serial.println(mA1);
    }
  }
  delay(1000);
}


// -- END OF FILE --