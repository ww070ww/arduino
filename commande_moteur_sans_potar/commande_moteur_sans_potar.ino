/*
commande du moteur brushless
 control du moteur brushless a l'aide d'un potentiométre
 
 il faut brancher un potentiomètre sur la broche analogique A0
 il faut brancher la broche 5  sur le PWM du variateur
 
 le programme attend 2s le potentiometre a zero 
 
 */

#include <Servo.h> 

#define P_MOTEUR_INIT 800 // impulsion minimum du variateur en us
#define PWM_PIN 5
Servo moteur;

int impulsion = P_MOTEUR_INIT;
bool etat_moteur=false;
bool etat_moteur_precedent=false;
String mon_buffer_serie="";

int selection_coeff=0;

void setup() {
  Serial.begin(115200);
  moteur.attach(PWM_PIN);
  moteur.writeMicroseconds(P_MOTEUR_INIT);
  delay(10);


}

void loop() {


  /*
   commande du moteur en fonction de l'erreur mesurée
   */
  if (etat_moteur) {
    if (!etat_moteur_precedent) {              // on va a la vitesse de consigne en utilisant une rampe
      int impulsion_temp=P_MOTEUR_INIT; 
      while (impulsion_temp<=impulsion) {
        moteur.writeMicroseconds(impulsion_temp);
        impulsion_temp+=50;
        delayMicroseconds(1000);
      }

    }

    moteur.writeMicroseconds(impulsion);
    etat_moteur_precedent=true;                      // mise a jour de l'état du moteur
  }
  else {
    etat_moteur_precedent=false;                     // mise a jour de l'état du moteur
    moteur.writeMicroseconds(P_MOTEUR_INIT);       // arret du moteur
  }




  /*
     changement a la volée des paramétres Kp Ki Kd etat_moteur
   */

  if (Serial.available() > 0) {
    // read the incoming byte:

    int incomingByte = Serial.read();

    switch ((char)incomingByte) {


    case 'v':
      selection_coeff=0;                          // réglage de la vitesse de base du moteur
      break;
    case 'o':
      etat_moteur=!etat_moteur;                     // mise en marche  et arrêt du système
      if (etat_moteur)
        Serial.println("etat moteur : ON");        // "on" "off"
      else
        Serial.println("etat moteur : OFF");                                
      break;                                      
    case '0':
      mon_buffer_serie+='0';                      // mise en mémoire des charactères "123456789.-"
      break;
    case '1':
      mon_buffer_serie+='1';
      break;
    case '2':
      mon_buffer_serie+='2';
      break;
    case '3':
      mon_buffer_serie+='3';
      break;
    case '4':
      mon_buffer_serie+='4';
      break;
    case '5':
      mon_buffer_serie+='5';
      break;
    case '6':
      mon_buffer_serie+='6';
      break;
    case '7':
      mon_buffer_serie+='7';
      break;
    case '8':
      mon_buffer_serie+='8';
      break;
    case '9':
      mon_buffer_serie+='9';
      break;
    case '.':
      mon_buffer_serie+='.';
      break;
    case '-':
      mon_buffer_serie+='-';
      break;
    case 'm':                                 // montrer l'état du buffer
      Serial.println(mon_buffer_serie);
      break;  
    case 's': // supprimer
      mon_buffer_serie="";                    // remise a zero du buffer
      break;  
    case'w':                                  // écrit le coeff courant "write"
      {
        mon_buffer_serie+="y";

        // conversion  de la chaine de caractère en double
        char *endp;
        char buf[mon_buffer_serie.length()];
        mon_buffer_serie.toCharArray(buf,mon_buffer_serie.length());
        double coeff=strtod(buf,&endp);       

        switch (selection_coeff) {

        case 0:
          Serial.println("vitesse moteur:"+mon_buffer_serie);
          impulsion=coeff;
          break;
        default:
          delayMicroseconds(1);
        }
        mon_buffer_serie="";         // remise à zéro du buffer
      }        
      break;
    default:
      delayMicroseconds(1);
    }


  }


}

