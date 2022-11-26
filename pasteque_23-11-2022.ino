#include <SoftwareSerial.h>

//-------------------------------------------------------------------
////////////////////////   Pin CNC shield     ///////////////////////
//-------------------------------------------------------------------

// Controlleurs Pas a Pas
#define xDirPin           2  //PD2 -> set PORTD |= 0b00000100 ou 0x04   eteindre PORTE &= 0b11111011 ou 0xFB
#define yDirPin           3  //PD3 -> set PORTD |= 0b00001000 ou 0x08   eteindre PORTE &= 0b11110111 ou 0xF7
#define zDirPin           4  //PD4 -> set PORTD |= 0b00010000 ou 0x10   eteindre PORTG &= 0b11101111 ou 0xEF
#define xStepPin          5  //PD5 -> set PORTD |= 0b00100000 ou 0x20   eteindre PORTE &= 0b11011111 ou 0xDF
#define yStepPin          6  //PD6 -> set PORTD |= 0b01000000 ou 0x40   eteindre PORTH &= 0b10111111 ou 0xBF
#define zStepPin          7  //PD7 -> set PORTD |= 0b10000000 ou 0x80   eteindre PORTH &= 0b01111111 ou 0x7F
#define enablePin         8
// Communication
#define RXSWSerial       13
#define TXSWSerial       12
// Mesure tension
#define tensionBatteriePin A7 //Analogique 7
#define facteurTension     0.01659 //Pont diviseur de tension + lecture sur 10 bits.
#define seuilAlerteTension   11 //Volts
#define seuilCritiqueTension   10 //Volts
//Leds
#define powerLedPin        9
#define batteryLedPin      10

#define STEP_HIGH_X       PORTD |= 0x20  // Activation Pin Pas X
#define STEP_LOW_X        PORTD &= 0xDF  // Désactivation Pin Pas X
#define STEP_HIGH_Y       PORTD |= 0x40  // Activation Pin Pas Z
#define STEP_LOW_Y        PORTD &= 0xBF  // Désactivation Pin Pas Y
#define STEP_HIGH_Z       PORTD |= 0x80  // Activation Pin Pas Z
#define STEP_LOW_Z        PORTD &= 0x7F  // Désactivation Pin Pas Z
#define SENS_TRIGO_X      PORTD |= 0x04
#define SENS_HORRAIRE_X   PORTD &= 0xFB
#define SENS_TRIGO_Y      PORTD |= 0x08
#define SENS_HORRAIRE_Y   PORTD &= 0xF7
#define SENS_TRIGO_Z      PORTD |= 0x10
#define SENS_HORRAIRE_Z   PORTD &= 0xEF



//-------------------------------------------------------------------
///////////////////////////  Constantes  ////////////////////////////
//-------------------------------------------------------------------

// conversion pas par tour et par radians.
#define pasParMetre    4244 * 1.03 //facteur experimental
const float rRobot = 0.124; //0.1265 théoriquement

//Determinées
const PROGMEM long dtMaxSpeed = 400; // Microsecondes entre 2 pas à la vitesse maximale
const PROGMEM long tempsAcc = 0.5 * pow(10, 6); // Temps d'accélération en µs
const PROGMEM long tempsDec = 0.25 * pow(10, 6); // Temps de décélération en µs

//Calculées
const PROGMEM float ratioAcc = max(tempsAcc, tempsDec) / tempsAcc;
const PROGMEM float ratioDec = max(tempsAcc, tempsDec) / tempsDec;
const PROGMEM unsigned int  nbEtapesAcc = tempsAcc / (2 * dtMaxSpeed); // Nombres de pas parcourus pendant l'accelération (intégrale de la vitesse)
const PROGMEM unsigned int  nbEtapesDec = tempsDec / (2 * dtMaxSpeed); // Nombres de pas parcourus pendant la décélération (intégrale de la vitesse)
//const PROGMEM unsigned int  nbEtapesAcc = 0; // Nombres de pas parcourus pendant l'accelération (intégrale de la vitesse)
//const PROGMEM unsigned int  nbEtapesDec = 0; // Nombres de pas parcourus pendant la décélération (intégrale de la vitesse)


const float dtMaxAcc = 1. / sqrt(2. / (tempsAcc * dtMaxSpeed)); // Plus grand intervalle de temps entre deux pas à l'accélération
const float dtMaxDec = 1. / sqrt(2. / (tempsDec * dtMaxSpeed)); // Plus grand intervalle de temps entre deux pas à la décélération

//-------------------------------------------------------------------
//////////////////////  Variables Déplacement  //////////////////////
//-------------------------------------------------------------------

//Sens de rotation
bool sensX = false;             //
bool sensY = false;             // sens trigo robot : true, sens horraire robot : false
bool sensZ = false;             //
// Vitesse Relative des moteurs
float ratioVitX = 1;
float ratioVitY = 1;
float ratioVitZ = 1;
// Noubre de pas déja faits et restants sur chaque moteur
volatile unsigned long pasFaitsX = 0;
volatile unsigned long pasFaitsY = 0;
volatile unsigned long pasFaitsZ = 0;
volatile unsigned long pasRestantsX = 0;
volatile unsigned long pasRestantsY = 0;
volatile unsigned long pasRestantsZ = 0;
//booleen de controle
volatile bool enMvmt = false;    // Le robot est en mouvement
volatile bool enAcc = false;     // Le robot est en phase d'accélération
volatile bool enDec = false;     // Le robot est en phase de décélération
// Etapes
volatile unsigned long etapeTrajet = 0;
volatile unsigned long t = 0; // Temps actuel
volatile unsigned long prevT = 0;                // Stockage du temps en microseconde auquel s'est passé la dernière étape
volatile unsigned int newDt = 5000;     // Intervalle de temps en microseconde avant la prochaine étape
volatile unsigned long etapesRestantes;          // Nombres d'étapes restantes
unsigned long nbTotalEtapes;            // Nombres total d'étapes à réaliser

//-------------------------------------------------------------------
/////////////////////     Autres Variables     //////////////////////
//-------------------------------------------------------------------

//Comunication
SoftwareSerial SWSerial(RXSWSerial, TXSWSerial);
String inputString = "";

//Batterie
bool batterieAlerte = false;
bool batterieCritique = false;
const unsigned int delaiCheckBatterie = 5000; //milisecondes
unsigned long prevCheckBatterie;
int onSeBallade = 0;

//-------------------------------------------------------------------
///////////////////////    Initialisation    ////////////////////////
//-------------------------------------------------------------------

void setup() {
  pinMode(xStepPin, OUTPUT);  // Pin pas moteur x en mode OUTPUT
  pinMode(yStepPin, OUTPUT);  // Pin pas moteur y en mode OUTPUT
  pinMode(zStepPin, OUTPUT);  // Pin pas moteur z en mode OUTPUT
  pinMode(xDirPin, OUTPUT);   // Pin dir moteur x en mode OUTPUT
  pinMode(yDirPin, OUTPUT);   // Pin dir moteur y en mode OUTPUT
  pinMode(zDirPin, OUTPUT);   // Pin dir moteur z en mode OUTPUT
  pinMode(enablePin, OUTPUT); // Pin alimentation moteurs en mode OUTPUT
  pinMode(tensionBatteriePin, INPUT);
  pinMode(powerLedPin, OUTPUT);
  pinMode(batteryLedPin, OUTPUT);
  Serial.begin(500000);   // Démarrage port série
  Serial.setTimeout(1);   // Délai d'attente port série 1ms
  SWSerial.begin(9600);
  SWSerial.setTimeout(100);
  AlimMoteurs(false);         // On éteint les moteurs (par sécurité)
  analogWrite(powerLedPin, 25);
  delay(150);
  analogWrite(powerLedPin, 0);
  delay(150);
  analogWrite(powerLedPin, 25);
  delay(150);
  analogWrite(powerLedPin, 0);
  CheckBatterie();
  //Serial.println(F("Initialisé"));
}

//-------------------------------------------------------------------
///////////////////////////     Loop      ///////////////////////////
//-------------------------------------------------------------------

void loop() {  // boucle de déplacement
  CheckSWSerial(); // Reception de consignes de l'étage commande
  if (enMvmt) { // Si on est en mouvement
    Avancer();   // On passe à l'étape d'après (si le temps necessaire s'est écoulé)
  } else {
    if (millis() - prevCheckBatterie >= delaiCheckBatterie) {
      CheckBatterie();
      prevCheckBatterie = millis();
    }
    //ballade();
  }
}

//-------------------------------------------------------------------
//////////////////////////   Fonctions    ///////////////////////////
//-------------------------------------------------------------------

void ballade() {
  switch (onSeBallade) {
    case 0:
      CalculPasRotation(0, 0.35, PI);
      onSeBallade++;
      Demarrer();
      break;
    case 1:
      CalculPasRotation(0, -0.35, -PI);
      onSeBallade++;
      Demarrer();
      break;
    case 2:
      CalculPasRotation(0, 0.35, PI);
      onSeBallade++;
      Demarrer();
      break;
    case 3:
      CalculPasRotation(0, -0.35, -PI);
      onSeBallade++;
      Demarrer();
      break;
    case 4:
      CalculPasRotation(0, 0.35, 2 * PI);
      onSeBallade++;
      Demarrer();
      break;
    case 5:
      CalculPasRotation(0, - 0.35, -PI);
      onSeBallade++;
      Demarrer();
      break;
    case 6:
      CalculPasRotation(0, 0.35, PI);
      onSeBallade++;
      Demarrer();
      break;
    case 7:
      CalculPasRotation(0, -0.35, -PI);
      onSeBallade++;
      Demarrer();
      break;
    case 8:
      CalculPasRotation(0, 0.35, PI);
      onSeBallade = 0;
      Demarrer();
      break;
    default:
      delay(500);
      AlimMoteurs(false);
      break;
  }

}

void CheckSWSerial() { // Récupération des consignes via le port SWSerial (RX TX)
  if (SWSerial.available() > 0) { //pour recuperer des valeurs du moniteur
    Serial.print("Reception consigne : ");
    int command = SWSerial.parseInt();
    Serial.println(command);
    if (command == 0) {  // Arret d'urgence
      Serial.println("Consigne d'arret");
      enMvmt = false;
      AlimMoteurs(false);
      float poubelle = SWSerial.parseFloat();
    }
    if (command == 1) {  // déplacment XY
      Serial.println("Consigne de départ");
      float x = SWSerial.parseFloat();
      float y = SWSerial.parseFloat();
      Serial.println((String) x + " " + y);
      CalculPasDeplacement(x, y);
      Demarrer();
      float poubelle = SWSerial.parseFloat();
    }
    if (command == 2) {  // Rotation : Centre XY et angle
      Serial.println("Consigne de départ");
      float x = SWSerial.parseFloat();
      float y = SWSerial.parseFloat();
      float a = SWSerial.parseFloat();
      CalculPasRotation(x, y , a);
      Demarrer();
      float poubelle = SWSerial.parseFloat();
    }
    if (command == 3) {  // Ancrage
      Serial.println("Consigne d'Ancrage");
      AlimMoteurs(SWSerial.parseInt());
      float poubelle = SWSerial.parseFloat();
    }
  }
}

void Demarrer() { // Lancement du mouvement
  if (!batterieCritique) {
    InitVariablesMvmt();
    SWSerial.print((int) 1);
    AlimMoteurs(true);
    //Serial.println(F("C'est parti !"));
  }
}

void InitVariablesMvmt() { // Initialisation des variables
  enMvmt = true;
  enAcc = true;
  enDec = false;
  etapeTrajet = 0;
  etapesRestantes = nbTotalEtapes - etapeTrajet;
  pasFaitsX = 0;
  pasFaitsY = 0;
  pasFaitsZ = 0;
  prevT = 0;
}

double arctan(float x, float y) {
  if (abs(x) < 0.001) {
    return (y / abs(y)) * PI / 2;
  } else if (x > 0) {
    return atan(y / x);
  } else {
    return borner(atan(y / x) + PI);
  }
}

void CalculPasRotation(float xCentre, float yCentre, float angle) {
  if (sqrt(sq(xCentre) + sq(yCentre)) == rRobot) yCentre += 0.001;
  float distXX = rRobot * cos(-PI / 6) - xCentre;
  float distYX = rRobot * sin(-PI / 6) - yCentre;
  float distXY = rRobot * cos(-5 * PI / 6) - xCentre;
  float distYY = rRobot * sin(-5 * PI / 6) - yCentre;
  float distXZ = rRobot * cos(PI / 2) - xCentre;
  float distYZ = rRobot * sin(PI / 2) - yCentre;

  float angleX = arctan(distXX, distYX) + PI / 2;
  float angleY = arctan(distXY, distYY) + PI / 2;
  float angleZ = arctan(distXZ, distYZ) + PI / 2;
  float distAFaireX = angle * sqrt(sq(distXX) + sq(distYX)); // longueur d'arc de rayon centre-point de contact
  float distAFaireY = angle * sqrt(sq(distXY) + sq(distYY));
  float distAFaireZ = angle * sqrt(sq(distXZ) + sq(distYZ));

  long pasMoteurX = (float) distAFaireX * pasParMetre * cos(angleX + (2. / 3) * PI);     //
  long pasMoteurY = (float) distAFaireY * pasParMetre * cos(angleY - (2. / 3) * PI );    // Calcul des distances à parcourir
  long pasMoteurZ = (float) distAFaireZ * pasParMetre * cos(angleZ);                     //

  MouvementSynchrone(pasMoteurX, pasMoteurY, pasMoteurZ);
}

void CalculPasDeplacement(float x, float y) { // Calcul des pas à faire sur chaque moteurs
  float angle = arctan(x, y);
  //Serial.print("angle directionnel : ");
  //Serial.println(angleRelatif * 180 / PI);
  float distanceTotalePas = sqrt(sq(x) + sq(y)) * pasParMetre;

  long pasMoteurX = (float) distanceTotalePas * cos(borner(angle + (2. / 3) * PI));     //
  long pasMoteurY = (float) distanceTotalePas * cos(borner(angle - (2. / 3) * PI ));    // Calcul des distances à parcourir
  long pasMoteurZ = (float) distanceTotalePas * cos(borner(angle));                     //

  MouvementSynchrone(pasMoteurX, pasMoteurY, pasMoteurZ);
}

void MouvementSynchrone(long x, long y, long z) {
  SensMoteurs((x >= 0) ? sensX = 1 : sensX = 0, (y >= 0) ? sensY = 1 : sensY = 0, (z >= 0) ? sensZ = 1 : sensZ = 0);
  pasRestantsX = abs(x);
  pasRestantsY = abs(y);
  pasRestantsZ = abs(z);
  nbTotalEtapes = max(max(pasRestantsX, pasRestantsY), pasRestantsZ);                                                    //
  ratioVitX = (float) pasRestantsX / nbTotalEtapes;                                     // Calcul et Stockage du vecteur
  ratioVitY = (float) pasRestantsY / nbTotalEtapes;                                     //  directionnel souhaité
  ratioVitZ = (float) pasRestantsZ / nbTotalEtapes;                                     //
}

void Avancer() { // Passe à la prochaine étape du mouvement si l'intervalle de temps est respecté
  t = micros();
  if (enAcc) { // Passage a la prochaine étape pendant l'accélération
    if (t - prevT >= newDt) {
      Move();
      prevT = t;
      ActualiserEtapeTrajet();
      newDt = dtMaxAcc / Sqrt(etapeTrajet);
      if (etapeTrajet >= nbEtapesAcc) { //Fin de l'acc
        enAcc = false;
      }
      if (etapesRestantes <= nbEtapesDec) {
        enDec = true;
        enAcc = false;
      }
    }
  } else if (enDec) {
    if (t - prevT >= newDt) {
      Move();
      prevT = t;
      ActualiserEtapeTrajet();
      newDt = dtMaxDec / Sqrt(etapesRestantes);
      if (etapesRestantes <= 0) {
        enMvmt = false;
        SWSerial.println((int) 0);
      }
    }
  } else {
    if (t - prevT >= dtMaxSpeed) {
      Move();
      prevT = t;
      ActualiserEtapeTrajet();
      if (etapesRestantes <= nbEtapesDec) enDec = true; //Debut décélération
    }
  }
}

void ActualiserEtapeTrajet() {
  etapeTrajet++;
  etapesRestantes = nbTotalEtapes - etapeTrajet;
}

void Move() { // Commande des moteurs
  if (etapesRestantes >= 0) {     // >= 0 pour ne pas perdre les derniers pas
    if (ratioVitX * etapeTrajet >= pasFaitsX) {
      PasMoteurX();
      pasFaitsX++;
    }
    if (ratioVitY * etapeTrajet >= pasFaitsY) {
      PasMoteurY();
      pasFaitsY++;
    }
    if (ratioVitZ * etapeTrajet >= pasFaitsZ) {
      PasMoteurZ();
      pasFaitsZ++;
    }
  }
}


uint16_t Sqrt(uint16_t input) {// Fonction de calcul de racine plus rapide trouvée sur https://gist.github.com/devgru/5006529
  uint16_t result = 0;
  uint16_t one = 1u << 14;
  while (one > input) one /= 4;
  while (one != 0) {
    if (input >= result + one) {
      result += one;
      input -= result;
      result += one;
    }
    result /= 2;
    one /= 4;
  }
  return result;
}

float borner(float angle) {                     //
  //while (abs(angle) > PI) {                     //
  if (angle > PI) {                           //
    return angle - 2. * PI;                   //
  } else if (angle <= PI) {                   // On remet l'angle entre -180 et 180
    return angle + 2. * PI;                   //
  }                                           //
  //}                                             //
}                                               //

void CheckBatterie() {
  float u = map(analogRead(tensionBatteriePin), 0, 1023, 0, 17);
  if (u <= seuilCritiqueTension) {
    batterieCritique = true;
    analogWrite(batteryLedPin, 25);
  } else  if (u <= seuilAlerteTension) {
    batterieAlerte = true;
    analogWrite(batteryLedPin, 0);
  } else {
    analogWrite(batteryLedPin, 0);
  }
  //Serial.println((String) "Batterie : " + u + " Volts" + batterieCritique + batterieAlerte);
}

//-------------------------------------------------------------------
//////////////////  Fonctions Physiques Moteurs  ////////////////////
//-------------------------------------------------------------------

void AlimMoteurs(bool power) { // Alimentation ou coupure du courant des moteurs
  digitalWrite(enablePin, !power);
}

void SensMoteurs(bool X, bool Y, bool Z) {
  X ? SENS_TRIGO_X : SENS_HORRAIRE_X; //choix du sens
  Y ? SENS_TRIGO_Y : SENS_HORRAIRE_Y; //choix du sens
  Z ? SENS_TRIGO_Z : SENS_HORRAIRE_Z; //choix du sens
}

void PasMoteurX() {
  STEP_HIGH_X;                 //set la pin sur HIGH
  //delayMicroseconds(5);
  STEP_LOW_X;                  //set la pin sur LOW
}
void PasMoteurY() {
  STEP_HIGH_Y;                 //set la pin sur HIGH
  //delayMicroseconds(5);
  STEP_LOW_Y;                  //set la pin sur LOW
}
void PasMoteurZ() {
  STEP_HIGH_Z;                 //set la pin sur HIGH
  //delayMicroseconds(5);
  STEP_LOW_Z;                  //set la pin sur LOW
}
