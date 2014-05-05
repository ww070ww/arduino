/*
commande du moteur brushless
 control du moteur brushless a l'aide d'un potentiométre
 
 il faut brancher un potentiomètre sur la broche analogique A0
 il faut brancher la broche 5  sur le PWM du variateur
 
 le programme attend 2s le potentiometre a zero 
 
 */

#include <Servo.h> 

const int analogPin = 7;
const int pwmPin = 5;
const int seuil = 10;


Servo moteur;

int valAnalog = 0;


void setup() {
  Serial.begin(38400);
  boolean tempo=true;
  unsigned long time = millis();
  
  moteur.attach(pwmPin);
  moteur.writeMicroseconds(1000);
  


/* attente de 2s potentiometre a zero pour la securite */
  
  
  while (tempo) {
    
    valAnalog = analogRead(analogPin);
    unsigned long maintenant = millis();
    
    if (valAnalog<seuil) {
      if (maintenant-time>2000) {
        tempo=false;
      } 
    }

    else {
      time = millis();
    }
    delay(5);


  }

}

void loop() {
  valAnalog = analogRead(analogPin);                     // lecture du potentiometre
  int impulsion = map(valAnalog,0,1023,800,2000);       // conversion de cette valeur en temp d'impulsion
  moteur.writeMicroseconds(impulsion);                   // generation de l'impulsion
  Serial.println(impulsion);
  delay(10);
}




