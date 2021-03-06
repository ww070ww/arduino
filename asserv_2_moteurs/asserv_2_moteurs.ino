#include <PID_v1.h>

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include <Servo.h> 
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
 NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
 depends on the MPU-6050's INT pin being connected to the Arduino's
 external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
 digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
 NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
 when using Serial.write(buf, len). The Teapot output uses this method.
 The solution requires a modification to the Arduino USBAPI.h file, which
 is fortunately simple, but annoying. This will be fixed in the next IDE
 release. For more info, see these links:
 
 http://arduino.cc/forum/index.php/topic,109987.0.html
 http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */







#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// mes variables
double pitch_mesure = 0; //input PID
double pitch_consigne= 0; // setpoint PID

Servo moteur_1;             // commande des moteurs
Servo moteur_2;

int p_moteurs=1000;   // valeur de la largeur impulsion entre 1000 et 2000us
double deltaP_moteur=0;  // valeur de la correction si erreur ouput PID


const int pin_potar_1 = 15;
const int pin_potar_2 = 14;

const int seuil=5;

int tension_1 = 0;  // tension aux bornesdes CAN
int tension_2 = 0;


double Kp = 0.7;
double Ki = 0.2;
double Kd = 0.2;
// reglage offset vitesse moteur

int offset=-34;

boolean reglage = false;

// variables permettant le changement des coefficients du correcteur

String mon_buffer_serie="";
int selection_coeff=0;

int count_println =0;
int nb_println=100;
//Specify the links and initial tuning parameters
PID myPID(&pitch_mesure, &deltaP_moteur, &pitch_consigne,Kp,Ki,Kd, DIRECT);


/*
// ================================================================
 // ===               INTERRUPT DETECTION ROUTINE                ===
 // ================================================================
 
 volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
 void dmpDataReady() {
 mpuInterrupt = true;
 }
 */


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {


  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device

  // mes initialisations

  moteur_1.attach(5);        // decision arbitraire du moteur 1
  moteur_2.attach(6);
  moteur_1.writeMicroseconds(p_moteurs);
  moteur_2.writeMicroseconds(p_moteurs+offset);

  delay(100);

  // initialisation PID

  myPID.SetOutputLimits(-50, 50);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);


  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  /*
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
*/
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(83);
  mpu.setYGyroOffset(-83);
  mpu.setZGyroOffset(-8);
  mpu.setXAccelOffset(-1385); 
  mpu.setYAccelOffset(1524);
  mpu.setZAccelOffset(1475);


  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    Serial.println("Mettre la puissance au minimum");

    int val =analogRead(pin_potar_2);
    while(val>seuil) 
      val =analogRead(pin_potar_2);


    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  mon_buffer_serie="";
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  /*  // wait for MPU interrupt or extra packet(s) available
   while (!mpuInterrupt && fifoCount < packetSize) {
   // other program behavior stuff here
   // .
   Serial.println("boucle while");
   // .
   // if you are really paranoid you can frequently test in between other
   // stuff to see if mpuInterrupt is true, and if so, "break;" from the
   // while() loop to immediately process the MPU datam
   // .
   // .
   // .
   }
   
   // reset interrupt flag and get INT_STATUS byte
   mpuInterrupt = false;*/
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;


    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //Serial.print("ypr\t");
    //Serial.print(ypr[0] * 180/M_PI);

    //Serial.print("\t");
    //Serial.println(ypr[2] * 180/M_PI);
    pitch_mesure=ypr[1]* 180/M_PI;

    p_moteurs=map(analogRead(pin_potar_2),0,1023,1000,2000);

    myPID.SetTunings(Kp, Ki, Kd);    
    myPID.Compute();

    count_println++;

    if (count_println>nb_println) {
      Serial.print("Kp :\t");
      Serial.print(Kp,6);
      Serial.print("\tKi :\t");
      Serial.print(Ki,6);
      Serial.print("\tKd :\t");
      Serial.println(Kd,6);

      Serial.print("Pitch :\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\tdelta :\t");
      Serial.print(deltaP_moteur);
      Serial.print("\terreur :\t");
      Serial.println(pitch_consigne-pitch_mesure);
      count_println=0;
    }
    // 2 modes de fonctionnement, le mode réglage et le mode normale

    if (reglage) {
      moteur_1.writeMicroseconds(p_moteurs+offset);
      moteur_2.writeMicroseconds(p_moteurs);
    }
    else {


      // commande du moteur suivant l'erreur
      if (deltaP_moteur>0) {
        moteur_1.writeMicroseconds(p_moteurs+offset+abs(deltaP_moteur));
        moteur_2.writeMicroseconds(p_moteurs);
      }
      else {
        moteur_1.writeMicroseconds(p_moteurs+offset);
        moteur_2.writeMicroseconds(p_moteurs+abs(deltaP_moteur));
      }
    }
    /*

     changement a la volee de coefficient Kp Ki et Kd
     */

    if (Serial.available() > 0) {
      // read the incoming byte:

      int incomingByte = Serial.read();
      //if (((char)incomingByte)=='p') {
      //Serial.println("totot");
      //}
      //delay(1000);

      switch ((char)incomingByte) {

      case 'p':        
        selection_coeff=0;  
        break;
      case 'i':
        selection_coeff=1;
        break;
      case 'd':
        selection_coeff=2;
        break;
      case 'o':
        selection_coeff=3;
        break;
      case 'r':
        selection_coeff=4;
        break;
      case '0':
        mon_buffer_serie+='0';
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
      case 'm':   // montrer
        Serial.println(mon_buffer_serie);
        break;  
      case 's': // supprimer
        mon_buffer_serie="";                    // remise a zero du buffer
        break;  
      case'w':                // write                // envois du coeff
        {
          mon_buffer_serie+="y";
          char *endp;
          char buf[mon_buffer_serie.length()];
          mon_buffer_serie.toCharArray(buf,mon_buffer_serie.length());
          double coeff=strtod(buf,&endp);       

          switch (selection_coeff) {
          case 0:
            Serial.println("Kp:"+mon_buffer_serie);
            Kp=coeff;
            break;
          case 1:
            Serial.println("Ki:"+mon_buffer_serie);
            Ki=coeff;
            break;
          case 2: 
            Serial.println("Kd:"+mon_buffer_serie);
            Kd=coeff;
            break;
          case 3:
            Serial.println("offset:"+mon_buffer_serie);
            offset=coeff;
            break;
          case 4:
            Serial.println("changement du mode de fonctionnement");
            reglage=!reglage;
            break;
          default:
            delayMicroseconds(1);
          }
          mon_buffer_serie="";
        }        
        break;
      default:
        delayMicroseconds(1);
      }


    }




    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}








