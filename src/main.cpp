#include <Arduino.h>
#include <LibRobus.h>
#include "Adafruit_TCS34725.h"

//=============================================================================================

void Suivre();
void Detection();

void Effrayer(uint32_t duree, uint32_t frequenceStrobe, uint32_t frequenceAlarme, uint32_t dureePompe);

void ClignoterDispositif(int dispositif, uint32_t *dernierTempsChangement, int frequence, int *etat);

// void nonBlockingStrobe(uint32_t *dernierTempsChangement, int pulseParSeconde, int *etatLumieres);
// void nonBlockingAlarme(uint32_t *lastChangeTime, int changeFrequency, int *buzzerState);

//Fonctions en Backup (pas utilisees)

void FaireParcours(int nbTours);
void StrobeEffect(int PulseParSec, int Duree); //Duree en secondes
void SonnerAlarme();

//Fonctions de base

float LireDistance(int capteur);

int Mouvement(float dist);
int Tourner(int dir, int Angle);
float FonctionPID(float distMotDroite, float distMotGauche);

//

// int lireCouleur();
// Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

//=============================================================================================

// #define ROUGE 0
// #define VERT 1
// #define BLEU 2
// #define JAUNE 3
// #define NOIR 4

// #define redpin 3
// #define greenpin 5
// #define bluepin 6
// #define commonAnode true

#define POMPE 22 //Sortie de l'Arduino pour la pompe

#define BUZZER 8    //Sortie de l'Arduino pour le buzzer
#define LUMIERES 24 //Sortie de l'Arduino pour la lumiere

//Entrees de l'Arduino pour les capteurs de ligne
#define CAPTEUR_GAUCHE 14
#define CAPTEUR_MILIEU 15
#define CAPTEUR_DROIT 16

#define ON true
#define OFF false

#define DIST_DETECTION_MIN 15 //La distance minimale de detection d intrus en centimetres
#define DIST_DETECTION_MAX 60 //La distance minimale de detection d intrus en centimetres

#define DUREE_EFFRAYER 2000 //Le temps pendant lequel le robot effraie un intrus avec lumieres et alarme sonore
#define DUREE_POMPE 2000 //Le temps pendant lequel le robot tire un jet d eau. <= DUREE_EFFRAYER
#define FREQUENCE_LUMIERES 5 //Frequence du clignotement des lumieres en Hz
#define FREQUENCE_ALARME 2 //Frequence du clignotement de l alarme sonore en Hz

//=============================================================================================

float facteurAcceleration;

float distTotMotDroite = 0;
float distTotMotGauche = 0;

float vitesseTourner = 0.2;

bool OutputSetup = false; // pour ne pas definir continuellement l'entree de la lumiere

bool EnPatrouille = false; //Si le robot patrouille le jardin presentement

//Capteurs de ligne
int capteurG;
int capteurM;
int capteurD;

//=============================================================================================

void setup()
{
  // put your setup code here, to run once:
  BoardInit();
  Serial.begin(9600);
  // if (tcs.begin())
  // {
  //   Serial.println("Found sensor");
  // }
  // else
  // {
  //   Serial.println("No TCS34725 found ... check your connections");
  // }
  // pinMode(redpin, OUTPUT);
  // pinMode(greenpin, OUTPUT);
  // pinMode(bluepin, OUTPUT);

  pinMode(CAPTEUR_GAUCHE, INPUT);
  pinMode(CAPTEUR_MILIEU, INPUT);
  pinMode(CAPTEUR_DROIT, INPUT);

  pinMode(POMPE, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LUMIERES, OUTPUT);

  digitalWrite(LUMIERES, LOW);
  noTone(BUZZER);

  MOTOR_SetSpeed(LEFT, 0);  // Moteur gauche
  MOTOR_SetSpeed(RIGHT, 0); // Moteur droit

  delay(1500);
}

//=============================================================================================

void loop()
{
  if (ROBUS_IsBumper(1))
  {
    Effrayer(DUREE_EFFRAYER, FREQUENCE_LUMIERES, FREQUENCE_ALARME, DUREE_POMPE);
  }

  if (ROBUS_IsBumper(3))
  {
    if (EnPatrouille)
    {
      EnPatrouille = false;
    }
    else
    {
      EnPatrouille = true;
    }
    delay(250);
  }

  if (EnPatrouille)
  {
    Suivre();
    Detection();
  }
  else
  {
    MOTOR_SetSpeed(0, 0);
    MOTOR_SetSpeed(1, 0);
  }
}

//=============================================================================================

//Fonction suiveur de ligne
void Suivre()
{
  capteurG = digitalRead(CAPTEUR_GAUCHE);
  // capteurM = digitalRead(CAPTEUR_MILIEU);
  capteurD = digitalRead(CAPTEUR_DROIT);

  Serial.println("Capteur g : ");
  Serial.print(capteurG);
  // Serial.println("Capteur m : ");
  // Serial.print(capteurM);
  Serial.println("Capteur d : ");
  Serial.print(capteurD);

  if (capteurG == HIGH)
  {
    MOTOR_SetSpeed(LEFT, 0.1);
    MOTOR_SetSpeed(RIGHT, 0.3);
  }
  else if (capteurD == HIGH)
  {
    MOTOR_SetSpeed(LEFT, 0.3);
    MOTOR_SetSpeed(RIGHT, 0.1);
  }
  else
  {
    MOTOR_SetSpeed(LEFT, 0.1);
    MOTOR_SetSpeed(RIGHT, 0.1);
  }
}

//=============================================================================================

//Fonction qui detecte si un intrus est proche du robot et qui enclenche l action
void Detection()
{
  float distAvant = LireDistance(1);
  float distGauche = LireDistance(0);

  Serial.println(distGauche);

  //Si l intrus est devant
  if (distAvant >= DIST_DETECTION_MIN && distAvant <= DIST_DETECTION_MAX)
  {
    Effrayer(DUREE_EFFRAYER, FREQUENCE_LUMIERES, FREQUENCE_ALARME, DUREE_POMPE);
  }

  //Si l intrus est a gauche
  if (distGauche >= DIST_DETECTION_MIN && distGauche <= DIST_DETECTION_MAX)
  {
    Tourner(-1, 90);
    delay(250);

    // DistAvant = LireDistance(1);
    // if (DistAvant >= 15 && DistAvant <= 60)
    // {
    Effrayer(DUREE_EFFRAYER, FREQUENCE_LUMIERES, FREQUENCE_ALARME, DUREE_POMPE);
    // }

    Tourner(1, 90);
    delay(250);
  }
}

//=============================================================================================

//Fonction qui actionne le jet deau, la lumiere et l alarme
void Effrayer(uint32_t duree, uint32_t frequenceLumieres, uint32_t frequenceAlarme, uint32_t dureePompe)
{
  //La pompe est activee
  bool pompeAretee = false;
  digitalWrite(POMPE, HIGH);

  //Les lumieres sont activees
  uint32_t tempsChangementLumieres = millis();
  int etatLumiere = 1;

  //L alarme est activee
  uint32_t tempsChangementAlarme = millis();
  int etatAlarme = 1;

  uint32_t tempsDepart = millis();

  while ((millis() - tempsDepart) < duree)
  {
    //Serial.println(millis() - tempsDepart);

    if ((millis() - tempsDepart) > dureePompe && !pompeAretee)
    {
      //Le jet d eau arrete
      digitalWrite(POMPE, LOW);
      pompeAretee = true;
    }

    ClignoterDispositif(LUMIERES , &tempsChangementLumieres, frequenceLumieres, &etatLumiere);
    ClignoterDispositif(BUZZER, &tempsChangementAlarme, frequenceAlarme, &etatAlarme);

    // nonBlockingStrobe(&tempsChangementLumieres, frequenceStrobe, &etatLumiere);
    // nonBlockingAlarme(&tempsChangementAlarme, frequenceAlarme, &etatAlarme);
  }

  //Les dispositifs sont desactives a la fin
  digitalWrite(POMPE, LOW);
  digitalWrite(LUMIERES, LOW);
  noTone(BUZZER);

  delay(500);
}

//=============================================================================================

//Fonction controle le clignotement d un dispositif de lumieres ou d alarme sonore
void ClignoterDispositif(int dispositif, uint32_t *dernierTempsChangement, int frequence, int *etat)
{
  uint32_t delai = 1000 / (2 * frequence); //La moitie d une periode en secondes avec frequence en Hz

  if ((millis() - *dernierTempsChangement) > delai)
  {
    if (!*etat)
    {
      switch (dispositif)
      {
      case LUMIERES : digitalWrite(LUMIERES, HIGH);
        break;
      
      case BUZZER : tone(BUZZER, 2000);
        break;
      }

      *etat = ON;
    }
    else
    {
      switch (dispositif)
      {
      case LUMIERES : digitalWrite(LUMIERES, LOW);
        break;
      
      case BUZZER : tone(BUZZER, 1000);
        break;
      }

      *etat = OFF;
    }

    *dernierTempsChangement = millis();
  }
}

//=============================================================================================

//Fonction controle des lumieres
// void nonBlockingStrobe(uint32_t *dernierTempsChangement, int frequenceLumiere, int *etatLumieres)
// {
//   uint32_t delaiLumieres = 1000 / (2 * frequenceLumiere);

//   if ((millis() - *dernierTempsChangement) > delaiLumieres)
//   {
//     if (!*etatLumieres)
//     {
//       digitalWrite(LUMIERES, HIGH);
//       *etatLumieres = ON;
//     }
//     else
//     {
//       digitalWrite(LUMIERES, LOW);
//       *etatLumieres = OFF;
//     }

//     *dernierTempsChangement = millis();
//   }
// }

//=============================================================================================

//Fonction de controle de l alarme
// void nonBlockingAlarme(uint32_t *dernierTempsChangement, int frequenceAlarme, int *etatBuzzer)
// {
//   uint32_t delaiAlarme = 1000 / (2 * frequenceAlarme);

//   if ((millis() - *dernierTempsChangement) > delaiAlarme)
//   {
//     if (!*etatBuzzer)
//     {
//       tone(BUZZER, 2000);
//       *etatBuzzer = ON;
//     }
//     else
//     {
//       tone(BUZZER, 1000);
//       *etatBuzzer = OFF;
//     }

//     *dernierTempsChangement = millis();
//   }
// }

//=============================================================================================

//Fonction de backup qui fait un parcours rectangle
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

//=============================================================================================

//Fonction de backup des lumieres
void StrobeEffect(int PulseParSec, int Duree)
{
  if (!OutputSetup)
  {
    pinMode(LUMIERES, OUTPUT);
    OutputSetup = true;
    Serial.println("pinout defined \n");
  }

  digitalWrite(LUMIERES, LOW);
  for (int t = 0; t < Duree; t++)
  {
    for (int i = 0; i < PulseParSec; i++)
    {
      digitalWrite(LUMIERES, HIGH);
      Serial.println("strobe on \n");
      delay(1000 / (2 * PulseParSec));

      digitalWrite(LUMIERES, LOW);
      Serial.println("strobe off \n");
      delay(1000 / (2 * PulseParSec));
    }
  }
}

//=============================================================================================

//Fonction de backup qui fait sonner l alarme repetitivement
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

//=============================================================================================

//Fonction qui retourne une distance lue en centimetres
float LireDistance(int capteur) //capteur 0 = GAUCHE. capteur 1 = AVANT
{
  float brut = ROBUS_ReadIR(capteur) / 200.0;
  float distance = pow(0.679454 * -1 * ((brut - 49.8115) / (brut - 0.230724)), (125000.0 / 139017.0));
  return distance;
}

//=============================================================================================

//Fonction de mouvement en ligne droite
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

//=============================================================================================

//Fonction de pivot central d'un angle desire
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

//=============================================================================================

//Fonction de PID pour un mouvement sans deviation
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

//=============================================================================================

//Fonction de detection de couleur avec capteurs de couleur
// int lireCouleur()
// {
//   uint16_t clear, red, green, blue;
//   tcs.setInterrupt(false);
//   tcs.getRawData(&red, &green, &blue, &clear);
//   tcs.setInterrupt(true);
//   float r, g, b, sum;
//   sum = clear;
//   r = red / sum;
//   g = green / sum;
//   b = blue / sum;
//   r = r * 256.0;
//   b = b * 256.0;
//   g = g * 256.0;
//   /*Serial.print("\tR:\t");   Serial.print((int)red);
//  Serial.print("\tg:\t");   Serial.print((int)green);
//  Serial.print("\tb:\t");   Serial.print((int)blue);
//  Serial.print("\tclear:\t");   Serial.print((int)clear);
//  Serial.println();*/

//   if (clear < 650)
//   {
//     return 4;
//   }
//   if (r > 95 && r < 125 && g > 55 && g < 75 && b > 55 && b < 85)
//   {
//     return 0;
//   }
//   if (r > 40 && r < 60 && g > 85 && g < 105 && b > 80 && b < 100)
//   {
//     return 1;
//   }
//   if (r > 30 && r < 50 && g > 70 && g < 90 && b > 100 && b < 130)
//   {
//     return 2;
//   }
//   if (r > 90 && r < 130 && g > 80 && g < 100 && b > 40 && b < 60)
//   {
//     return 3;
//   }

//   return -1;
// }

//=============================================================================================