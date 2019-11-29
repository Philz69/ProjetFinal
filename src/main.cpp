#include <Arduino.h>
#include <LibRobus.h>
#include "Adafruit_TCS34725.h"

int Mouvement(float dist);
int Tourner(int dir, int Angle);
float FonctionPID(float distMotDroite, float distMotGauche);
void Suivre();

void SonnerAlarme();
void Detection();
void FaireParcours(int nbTours);

int lireCouleur();
float LireDistance(int capteur);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

int StrobeEffect(int PulseParSec, int Duree); //Duree est en secondes
void nonBlockingBuzzer(uint32_t *lastChangeTime, int changeFrequency, int *buzzerState);
void nonBlockingStrobe(uint32_t *lastChangeTime, int  PulsePerSecond, int *lightState); //Changes light state if lastChangeTime is high enough
void Action(uint32_t duration, int lightFrequency, int soundFrequency, uint32_t pumpDuration);
void LightCTRL(bool OnOff, int PinOut);
//=============================================================================================

//constante
#define ROUGE 0
#define VERT 1
#define BLEU 2
#define JAUNE 3
#define NOIR 4

#define redpin 3
#define greenpin 5
#define bluepin 6

#define BUZZER 8
#define POMPE 22
#define CAPTEUR_GAUCHE 14
#define CAPTEUR_MILIEU 15
#define CAPTEUR_DROIT 16

#define commonAnode true

#define ON true
#define OFF false
#define LumOutput 13 // pour definir la sortie de l'Arduino pour la lumiere

float facteurAcceleration;

float distTotMotDroite = 0;
float distTotMotGauche = 0;

float vitesseTourner = 0.2;

bool OutputSetup = false; // pour ne pas definir continuellement l'entree de la lumiere

bool EnSuivi = false;

int capteurG;
int capteurM;
int capteurD;

int DetectionFaite = 0;

unsigned long temps_initial = 0;
#define DELAI_TEST 1500 //250 ms de delai entre les tonalites de lalarme

//=============================================================================================

void setup()
{
  // put your setup code here, to run once:
  BoardInit();
  Serial.begin(9600);
  if (tcs.begin())
  {
    Serial.println("Found sensor");
  }
  else
  {
    Serial.println("No TCS34725 found ... check your connections");
  }
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);

  pinMode(POMPE, OUTPUT);
  pinMode(CAPTEUR_GAUCHE, INPUT);
  pinMode(CAPTEUR_MILIEU, INPUT);
  pinMode(CAPTEUR_DROIT, INPUT);

  pinMode(BUZZER, OUTPUT);
  noTone(BUZZER);

  delay(1500);
  MOTOR_SetSpeed(0, 0); // Moteur gauche
  MOTOR_SetSpeed(1, 0); // Moteur droit

  LightCTRL(OFF,LumOutput);
}

void loop()
{
  if(ROBUS_IsBumper(0))
  {
    digitalWrite(POMPE, HIGH);
    delay(1000);
  digitalWrite(POMPE, LOW);
  }
  if(ROBUS_IsBumper(1))
  {
    SonnerAlarme();
  }
  if(ROBUS_IsBumper(3))
  {
    //FaireParcours(3);
    Detection();
  }

  if(ROBUS_IsBumper(2))
  {
    if(EnSuivi)
    {
      EnSuivi = false;
    }
    else
    {
      EnSuivi = true;
    }
    delay(250);
  }

  if(EnSuivi)
  {
    Suivre();

    //Test 90 pour detection
    if(millis() > temps_initial + DELAI_TEST && DetectionFaite == 0)
    {
      Tourner(1, 89);
      delay(500);
      Tourner(1, 89);

      DetectionFaite = 1;
    }
  }
  else
  {
    MOTOR_SetSpeed(0,0);
    MOTOR_SetSpeed(1,0);
  }
}

void SonnerAlarme()
{
  //Mettre la fonction tant que le robot est en detection et quil tire
  for (int i = 0; i < 6; i++)
  {
    tone(BUZZER, 1000);

    delay(250);
    tone(BUZZER, 2000);

    // noTone(BUZZER);
    // AX_BuzzerON();

    delay(250);
    // AX_BuzzerOFF();
    }

    noTone(BUZZER);
}

void FaireParcours(int nbTours)
{
    for (int i = 0; i < nbTours; i++)
    {
      facteurAcceleration = 0.5;
      Mouvement(120);
      Tourner(1, 89);
      Mouvement(100);
      Tourner(1, 89);
      Mouvement(120);
      Tourner(1, 88);
      Mouvement(100);
      Tourner(1, 89);
    }
}

float LireDistance(int capteur) //capteur 0 = GAUCHE. capteur 1 = AVANT
{
  float brut = ROBUS_ReadIR(capteur) / 200.0;
  float distance = pow(0.679454 * -1 * ((brut - 49.8115) / (brut - 0.230724)), (125000.0 / 139017.0));
  return distance;
}

void Detection(void)
{
  float DistAvant = LireDistance(1);
  float DistGauche = LireDistance(0);
  Serial.println(DistGauche);
  if(DistAvant >= 15 && DistAvant <= 60)
  {
    Action(2, 50, 4, 2);
  } 
  if(DistGauche >= 15 && DistGauche <= 60)
  {
    Tourner(-1, 90);
    DistAvant = LireDistance(1);
    if(DistAvant >= 15 && DistAvant <= 60)
    {
    Action(2, 50, 4, 2);
    } 
  } 
}

int Tourner(int dir, int Angle) //dir = -1 pour tourner a gauche et dir = 1 pour tourner à droite
{
  int AngleActuel = 0;
  int32_t EncoderG = 0;
  int32_t EncoderD = 0;

  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);

  while (AngleActuel <= Angle && !ROBUS_IsBumper(2))
  {
    EncoderG = ENCODER_Read(0);
    EncoderD = ENCODER_Read(1);

    AngleActuel = (EncoderG) / (22.0418) * dir;

    /*Serial.println(EncoderG);
    Serial.println(EncoderD);
    Serial.println("\n");*/

    if (dir < 0)
    {
      MOTOR_SetSpeed(0, -vitesseTourner); // Moteur gauche
      MOTOR_SetSpeed(1, vitesseTourner);  // Moteur droit
    }
    else
    {
      MOTOR_SetSpeed(0, vitesseTourner);  // Moteur gauche
      MOTOR_SetSpeed(1, -vitesseTourner); // Moteur droit
    }
    //delay(75);
  }

  MOTOR_SetSpeed(0, 0); // Moteur gauche
  MOTOR_SetSpeed(1, 0); // Moteur droit

  delay(50);
  return EncoderG;
}

int Mouvement(float dist)
{
  int32_t EncoderG = 0;
  int32_t EncoderD = 0;
  float Distactuel = 0;
  double accel = 0;
  double distAccel = 30;

  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);

  while (Distactuel <= dist && !ROBUS_IsBumper(2))
  {
    if (distAccel >= (dist / 2))
    {
      if (Distactuel < dist / 2)
      {
        accel = (Distactuel / distAccel) * facteurAcceleration;
        //Serial.println("1");
      }
      else
      {
        accel = ((dist - Distactuel) / distAccel) * facteurAcceleration;
        //Serial.println("2");
      }
    }
    else if (Distactuel < distAccel)
    {
      accel = (Distactuel / distAccel) * facteurAcceleration;
      //Serial.println("3");
    }
    else if (dist - Distactuel <= distAccel)
    {
      accel = ((dist - Distactuel) / distAccel) * facteurAcceleration;
      //Serial.println("4");
    }
    else
    {
      accel = facteurAcceleration;
    }
    EncoderG = ENCODER_Read(0);
    EncoderD = ENCODER_Read(1);
    Distactuel = (EncoderG) / (133.6675);
    // Serial.println(EncoderD);
    // Serial.println(EncoderG);
    //Serial.println("\n");

    MOTOR_SetSpeed(0, accel + 0.1 - FonctionPID(ENCODER_Read(0), ENCODER_Read(1))); // Moteur gauche
    MOTOR_SetSpeed(1, accel + 0.1 + FonctionPID(ENCODER_Read(0), ENCODER_Read(1))); // Moteur droit
    //delay(50);
  }

  MOTOR_SetSpeed(0, 0); // Moteur gauche
  MOTOR_SetSpeed(1, 0); // Moteur droit
  Distactuel = 0;

  delay(100);
  return EncoderG;
}

float FonctionPID(float distMotDroite, float distMotGauche)
{
  float kp = 0.001;
  float ki = 0.002;
  float diffDist = 0;
  float P = 0; // P = Produit de la difference dans un PID
  float diffDistTotal = 0;
  float I = 0; // I = Integrale de la difference dans un PID
  float vitMot1 = 0;
  float distMD = 0;
  float distMG = 0;

  distMD = distMotDroite - distTotMotDroite;
  distMG = distMotGauche - distTotMotGauche;

  diffDist = distMD - distMG;
  diffDistTotal = distMotDroite - distMotGauche;

  P = diffDist * kp;
  I = diffDistTotal * ki;
  vitMot1 = (P + I) / 2;

  distTotMotDroite = distMotDroite;
  distTotMotGauche = distMotGauche;

  return vitMot1;
}

int lireCouleur()
{
  uint16_t clear, red, green, blue;
  tcs.setInterrupt(false);
  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(true);
  float r, g, b, sum;
  sum = clear;
  r = red / sum;
  g = green / sum;
  b = blue / sum;
  r = r * 256.0;
  b = b * 256.0;
  g = g * 256.0;
  /*Serial.print("\tR:\t");   Serial.print((int)red);
 Serial.print("\tg:\t");   Serial.print((int)green);
 Serial.print("\tb:\t");   Serial.print((int)blue);
 Serial.print("\tclear:\t");   Serial.print((int)clear);
 Serial.println();*/

  if (clear < 650)
  {
    return 4;
  }
  if (r > 95 && r < 125 && g > 55 && g < 75 && b > 55 && b < 85)
  {
    return 0;
  }
  if (r > 40 && r < 60 && g > 85 && g < 105 && b > 80 && b < 100)
  {
    return 1;
  }
  if (r > 30 && r < 50 && g > 70 && g < 90 && b > 100 && b < 130)
  {
    return 2;
  }
  if (r > 90 && r < 130 && g > 80 && g < 100 && b > 40 && b < 60)
  {
    return 3;
  }

  return -1;
}

//Fonction suiveur de ligne
void Suivre()
{
  capteurG = digitalRead(CAPTEUR_GAUCHE);
  capteurM = digitalRead(CAPTEUR_MILIEU);
  capteurD = digitalRead(CAPTEUR_DROIT);

  Serial.println("Capteur g : ");
  Serial.print(capteurG);
  Serial.println("Capteur m : ");
  Serial.print(capteurM);
  Serial.println("Capteur d : ");
  Serial.print(capteurD);

  if (capteurG == HIGH)
  {
    MOTOR_SetSpeed(0, 0.1);
    MOTOR_SetSpeed(1, 0.3);
  }
  else if (capteurD == HIGH)
  {
    MOTOR_SetSpeed(0, 0.3);
    MOTOR_SetSpeed(1, 0.1);
  }
  else
  {
    MOTOR_SetSpeed(0, 0.1);
    MOTOR_SetSpeed(1, 0.1);
  }

  // if (Capteur >= 5)
  // {
  //   MOTOR_SetSpeed(0, -0.5); // Faire des tests pour voir quel angle fonctionne mieux.
  //   MOTOR_SetSpeed(1, 0);    // 111 -> Pas de ligne
  // }
  // else if (Capteur >= 4.28) //110 -> Tourne a gauche
  // {
  //   MOTOR_SetSpeed(0, -0.3); //Gauche
  //   MOTOR_SetSpeed(1, 0.7);  //Droite
  // }
  // else if (Capteur >= 3.57) //101 -> Avance
  // {
  //   MOTOR_SetSpeed(0, 0.7);
  //   MOTOR_SetSpeed(1, 0.7);
  // }
  // else if (Capteur >= 2.86) //100 -> Tourne a gauche un peu
  // {
  //   MOTOR_SetSpeed(0, 0.3);
  //   MOTOR_SetSpeed(1, 0.7);
  // }
  // else if (Capteur >= 2.14) //011 -> Tourne a droite
  // {
  //   MOTOR_SetSpeed(0, 0.7);
  //   MOTOR_SetSpeed(1, -0.3);
  // }
  // else if (Capteur >= 1.42) //010 -> Intersection tourne a droite
  // {
  //   MOTOR_SetSpeed(0, 0.7);
  //   MOTOR_SetSpeed(1, 0.3);
  // }
  // else if (Capteur >= 0.72) //001 -> Tourne a droite un peu
  // {
  //   MOTOR_SetSpeed(0, 0.7);
  //   MOTOR_SetSpeed(1, 0.3);
  // }
  // else if (Capteur >= 0) //000 -> Intersection milieu stop prendre une decision. peut etre ajouter return.
  // {
  //   MOTOR_SetSpeed(0, 0);
  //   MOTOR_SetSpeed(1, 0);
  // }
  // else //Par defaut
  // {
  //   MOTOR_SetSpeed(0, 0.5);
  //   MOTOR_SetSpeed(1, 0.5);
  // }
}

//=============================================================================================
//fonctions controle des lumieres

void nonBlockingStrobe(uint32_t *lastChangeTime, int  PulsePerSecond, int *lightState)
{
  uint32_t lightDelay = 1000/(2*PulsePerSecond);

      if( ( millis() - *lastChangeTime ) > lightDelay)
      {
        if(*lightState)
        {
      LightCTRL(ON, LumOutput);
        *lightState = ON;
        }
        else
        {
      LightCTRL(OFF, LumOutput);
        *lightState = OFF;
        }
        *lastChangeTime = millis();
      }
}

void nonBlockingBuzzer(uint32_t *lastChangeTime, int changeFrequency, int *buzzerState)
{
  uint32_t buzzerDelay = 1/changeFrequency;

      if( ( millis() - *lastChangeTime ) >  buzzerDelay)
      {
        if(*buzzerState)
        {
        tone(BUZZER, 2000);
        *buzzerState = ON;
        }
        else
          {
        tone(BUZZER, 1000);
        *buzzerState = OFF;
        }
        *lastChangeTime = millis();
      }
}

int StrobeEffect(int PulseParSec, int Duree)
{
  if(! OutputSetup)
  {
    pinMode(LumOutput, OUTPUT);
    OutputSetup = true;  
    Serial.println("pinout defined \n");
  }

  LightCTRL(OFF, LumOutput);
  for(int t = 0; t < Duree; t++)
  {
    for(int i = 0; i < PulseParSec; i++)
    {
      LightCTRL(ON, LumOutput);
      Serial.println("strobe on \n");
      delay(1000/(2*PulseParSec));

      LightCTRL(OFF, LumOutput);
      Serial.println("strobe off \n");
      delay(1000/(2*PulseParSec));
    }
  }
}

void Action(uint32_t duration, int lightFrequency, int soundFrequency, uint32_t pumpDuration)
{
  int pumpFired = 0;
  int startTime = 0;

  uint32_t buzzerChangeTime = 0;
  int buzzerState = 0;

  int lightState = 0;
  uint32_t lightChangeTime = 0;

  digitalWrite(POMPE, HIGH);

  tone(BUZZER, 1000);
  buzzerChangeTime = millis();
  buzzerState = 1;

  LightCTRL(ON, LumOutput);
  lightChangeTime = millis();
  lightState = 1;

  startTime = millis();
  while( (millis() - startTime) < duration)
  {
      if( ( millis() - startTime ) > pumpDuration && !pumpFired)
      {
        digitalWrite(POMPE, LOW);
        pumpFired = 1;
      }

      nonBlockingBuzzer(&buzzerChangeTime, soundFrequency, &buzzerState);
      nonBlockingStrobe(&lightChangeTime, lightFrequency, &lightState);

  }

  LightCTRL(OFF, LumOutput);
  noTone(BUZZER);

}

void LightCTRL(bool OnOff, int PinOut){
  if (OnOff)
  {
    digitalWrite(PinOut, HIGH);
    //Serial.println("cmd lum on \n");
  } 
  else 
  {
    digitalWrite(PinOut, LOW);
    //Serial.println("cmd lum off \n");
  }
}

//=============================================================================================
