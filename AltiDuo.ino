/*
  Model Rocket dual altimeter Ver 1.5
  Copyright Boris du Reau 2012-2013
  Améliorations apportées :
  1. Sécurité renforcée pour la détection d'apogée
  2. Gestion améliorée des états de vol
  3. Débogage plus structuré
  4. Gestion d'erreurs pour le capteur BMP085
*/

// ... (le reste des commentaires d'en-tête reste inchangé)

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_BMP085.h>

#define MAJOR_VERSION 1
#define MINOR_VERSION 5

// Définition des constantes pour un meilleur contrôle
#define DEBUG
#define USE_TONE
#define METRIC_UNIT  // Décommenter pour utiliser les mètres

// Constantes de sécurité
#define APOGEE_CONFIRMATION_COUNT 10  // Nombre de mesures descendantes pour confirmer l'apogée
#define LIFTOFF_ALTITUDE 20          // Altitude de décollage en mètres (AGL)
#define MIN_APOGEE_ALTITUDE 50       // Altitude minimale pour déclencher l'apogée (m)
#define MAIN_DEPLOY_DEFAULT 100      // Altitude par défaut pour le déploiement principal (m)
#define DEPLOYMENT_DURATION 2000     // Durée d'activation des allumeurs (ms)
#define KALMAN_INIT_SAMPLES 50       // Nombre d'échantillons pour l'initialisation Kalman
#define GROUND_ALT_SAMPLES 10        // Nombre d'échantillons pour calculer l'altitude initiale

// Définition des broches avec des noms plus explicites
enum {
  PIN_APOGEE_FIRE = 9,
  PIN_MAIN_FIRE = 13,  // Changé de 8 à 13 pour correspondre au schéma
  PIN_APOGEE_CONT = 10,
  PIN_MAIN_CONT = 11,
  PIN_SPEAKER = 12,
  PIN_ALT_SELECT1 = 6,  // Renommé pour plus de clarté
  PIN_ALT_SELECT2 = 7
};

// État du vol
enum FlightState {
  STATE_PRELAUNCH,
  STATE_ASCENT,
  STATE_APOGEE,
  STATE_DESCENT,
  STATE_MAIN_DEPLOYED,
  STATE_LANDED
};

// Structure pour la configuration stockée en EEPROM
struct ConfigStruct {
  char app[7] = "AltDuo";
  int majorVersion = MAJOR_VERSION;
  int minorVersion = MINOR_VERSION;
  int apogeeAltitude = 0;
  byte cksum = 0xBA;
  
  // Calcul du checksum pour validation
  bool isValid() const {
    byte calculatedCksum = 0xBA;  // Checksum de base
    for(size_t i = 0; i < sizeof(app); i++) {
      calculatedCksum ^= app[i];
    }
    calculatedCksum ^= (majorVersion & 0xFF);
    calculatedCksum ^= (minorVersion & 0xFF);
    calculatedCksum ^= (apogeeAltitude & 0xFF);
    calculatedCksum ^= ((apogeeAltitude >> 8) & 0xFF);
    return cksum == calculatedCksum;
  }
  
  void updateChecksum() {
    cksum = 0xBA;
    for(size_t i = 0; i < sizeof(app); i++) {
      cksum ^= app[i];
    }
    cksum ^= (majorVersion & 0xFF);
    cksum ^= (minorVersion & 0xFF);
    cksum ^= (apogeeAltitude & 0xFF);
    cksum ^= ((apogeeAltitude >> 8) & 0xFF);
  }
};

// Variables globales avec initialisation explicite
Adafruit_BMP085 bmp;
FlightState currentState = STATE_PRELAUNCH;

long initialAltitude = 0;
long currentAltitude = 0;
long apogeeAltitude = 0;
long mainDeployAltitude = 0;
long lastAltitude = 0;

unsigned int apogeeConfirmationCounter = APOGEE_CONFIRMATION_COUNT;
bool apogeeFired = false;
bool mainFired = false;
bool apogeeSaved = false;

// Variables du filtre Kalman
float kalman_q = 4.0001;
float kalman_r = 0.20001;
float kalman_x = 0;
float kalman_p = 0;
float kalman_x_last = 0;
float kalman_p_last = 0;
float kalman_k = 0;

void setup() {
  initializeHardware();
  initializeKalmanFilter();
  initializeAltitude();
  readConfiguration();
  announceVersion();
  checkContinuity();
}

void loop() {
  updateAltitude();
  
  switch(currentState) {
    case STATE_PRELAUNCH:
      handlePrelaunch();
      break;
    case STATE_ASCENT:
      handleAscent();
      break;
    case STATE_APOGEE:
      handleApogee();
      break;
    case STATE_DESCENT:
      handleDescent();
      break;
    case STATE_MAIN_DEPLOYED:
      handleMainDeployed();
      break;
    case STATE_LANDED:
      handleLanded();
      break;
  }
  
  #ifdef DEBUG
    logFlightData();
  #endif
}

// Initialisation du matériel
void initializeHardware() {
  #ifdef DEBUG
    Serial.begin(9600);
    Serial.println(F("Initialisation du système..."));
  #endif
  
  Wire.begin();
  
  if (!bmp.begin()) {
    #ifdef DEBUG
      Serial.println(F("ERREUR: Capteur BMP085 non détecté!"));
    #endif
    errorBeep(3);
    while(1);  // Arrêt du système
  }
  
  // Configuration des broches
  pinMode(PIN_APOGEE_FIRE, OUTPUT);
  pinMode(PIN_MAIN_FIRE, OUTPUT);
  pinMode(PIN_SPEAKER, OUTPUT);
  pinMode(PIN_ALT_SELECT1, INPUT_PULLUP);  // Activation des résistances pull-up
  pinMode(PIN_ALT_SELECT2, INPUT_PULLUP);
  pinMode(PIN_APOGEE_CONT, INPUT_PULLUP);
  pinMode(PIN_MAIN_CONT, INPUT_PULLUP);
  
  // Assurance que les sorties sont désactivées
  digitalWrite(PIN_APOGEE_FIRE, LOW);
  digitalWrite(PIN_MAIN_FIRE, LOW);
  digitalWrite(PIN_SPEAKER, LOW);
  
  // Configuration de l'altitude de déploiement principal
  mainDeployAltitude = readDeployAltitude();
}

// Lecture de l'altitude de déploiement depuis les cavaliers
int readDeployAltitude() {
  int val1 = digitalRead(PIN_ALT_SELECT1);
  int val2 = digitalRead(PIN_ALT_SELECT2);
  
  if(val1 == LOW && val2 == LOW) return 50;
  if(val1 == LOW && val2 == HIGH) return 100;
  if(val1 == HIGH && val2 == LOW) return 150;
  return 200;  // HIGH, HIGH
}

// Initialisation du filtre Kalman
void initializeKalmanFilter() {
  for (int i = 0; i < KALMAN_INIT_SAMPLES; i++) {
    kalmanFilter(bmp.readAltitude());
    delay(10);
  }
}

// Calcul du filtre Kalman
float kalmanFilter(float measurement) {
  // Prédiction
  kalman_x = kalman_x_last;
  kalman_p = kalman_p_last + kalman_r;
  
  // Mise à jour
  kalman_k = kalman_p / (kalman_p + kalman_q);
  kalman_x = kalman_x + kalman_k * (measurement - kalman_x);
  kalman_p = (1 - kalman_k) * kalman_p;
  
  // Sauvegarde pour l'itération suivante
  kalman_x_last = kalman_x;
  kalman_p_last = kalman_p;
  
  return kalman_x;
}

// Initialisation de l'altitude de référence
void initializeAltitude() {
  long sum = 0;
  
  for (int i = 0; i < GROUND_ALT_SAMPLES; i++) {
    sum += kalmanFilter(bmp.readAltitude());
    delay(50);
  }
  
  initialAltitude = sum / GROUND_ALT_SAMPLES;
  
  #ifdef DEBUG
    Serial.print(F("Altitude initiale: "));
    Serial.println(initialAltitude);
  #endif
}

// Mise à jour de l'altitude actuelle
void updateAltitude() {
  float rawAltitude = bmp.readAltitude();
  
  if (isnan(rawAltitude)) {
    #ifdef DEBUG
      Serial.println(F("ERREUR: Lecture altitude invalide"));
    #endif
    return;
  }
  
  currentAltitude = kalmanFilter(rawAltitude) - initialAltitude;
  lastAltitude = currentAltitude;
}

// Gestion de l'état pré-lancement
void handlePrelaunch() {
  // Vérification de continuité périodique
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 1000) {
    checkContinuity();
    lastCheck = millis();
  }
  
  // Détection du décollage
  if (currentAltitude > LIFTOFF_ALTITUDE) {
    currentState = STATE_ASCENT;
    #ifdef DEBUG
      Serial.println(F("DECOLLAGE DETECTE"));
    #endif
    beepSequence(2, 200);  // Deux bips courts pour indiquer le décollage
  }
}

// Gestion de l'ascension
void handleAscent() {
  if (currentAltitude > lastAltitude) {
    // Toujours en ascension
    lastAltitude = currentAltitude;
    apogeeConfirmationCounter = APOGEE_CONFIRMATION_COUNT;
  } else {
    // Début de la descente - confirmation de l'apogée
    if (--apogeeConfirmationCounter == 0) {
      if (currentAltitude >= MIN_APOGEE_ALTITUDE) {
        fireApogee();
      } else {
        #ifdef DEBUG
          Serial.println(F("Apogée trop basse, annulation"));
        #endif
      }
    }
  }
}

// Déclenchement de l'apogée
void fireApogee() {
  digitalWrite(PIN_APOGEE_FIRE, HIGH);
  delay(DEPLOYMENT_DURATION);
  digitalWrite(PIN_APOGEE_FIRE, LOW);
  
  apogeeAltitude = currentAltitude;
  apogeeFired = true;
  currentState = STATE_APOGEE;
  
  #ifdef DEBUG
    Serial.print(F("APOGEE: "));
    Serial.println(apogeeAltitude);
  #endif
}

// Gestion après apogée
void handleApogee() {
  if (currentAltitude <= mainDeployAltitude && !mainFired) {
    fireMain();
  }
  currentState = STATE_DESCENT;
}

// Déclenchement du parachute principal
void fireMain() {
  digitalWrite(PIN_MAIN_FIRE, HIGH);
  delay(DEPLOYMENT_DURATION);
  digitalWrite(PIN_MAIN_FIRE, LOW);
  
  mainFired = true;
  currentState = STATE_MAIN_DEPLOYED;
  
  #ifdef DEBUG
    Serial.println(F("Parachute principal deploye"));
  #endif
}

// Gestion après déploiement principal
void handleMainDeployed() {
  static bool altitudeReported = false;
  
  if (!altitudeReported) {
    saveApogee();
    reportAltitude();
    altitudeReported = true;
  }
  
  // Détection de l'atterrissage
  if (abs(currentAltitude) < 5) {  // À ±5m du sol
    currentState = STATE_LANDED;
  }
}

// Gestion après atterrissage
void handleLanded() {
  // Rien à faire pour le moment
  delay(1000);
}

// Lecture de la configuration depuis l'EEPROM
void readConfiguration() {
  ConfigStruct config;
  EEPROM.get(0, config);
  
  if (config.isValid() && config.apogeeAltitude > 0) {
    #ifdef DEBUG
      Serial.print(F("Apogee précédente: "));
      Serial.println(config.apogeeAltitude);
    #endif
    
    // Conversion et annonce de l'apogée précédente
    long previousApogee = config.apogeeAltitude * FEET_IN_METER;
    for(int i = 0; i < 2; i++) {
      beepAltitude(previousApogee);
      delay(1000);
    }
  }
}

// Sauvegarde de l'apogée dans l'EEPROM
void saveApogee() {
  if (!apogeeSaved && apogeeAltitude > 0) {
    ConfigStruct config;
    config.apogeeAltitude = apogeeAltitude;
    config.updateChecksum();
    
    EEPROM.put(0, config);
    apogeeSaved = true;
    
    #ifdef DEBUG
      Serial.println(F("Apogee sauvegardee en EEPROM"));
    #endif
  }
}

// Annonce de l'altitude par bips
void reportAltitude() {
  beginBeepSequence();
  delay(1000);
  beepAltitude(apogeeAltitude * FEET_IN_METER);
  delay(2000);
}

// Vérification de continuité des allumeurs
void checkContinuity() {
  checkSingleContinuity(PIN_APOGEE_CONT, 1);
  delay(200);
  checkSingleContinuity(PIN_MAIN_CONT, 2);
}

void checkSingleContinuity(int pin, int channel) {
  bool continuity = digitalRead(pin) == LOW;  // LOW = continuité (cavaliers fermés)
  
  #ifdef DEBUG
    Serial.print(F("Continuite "));
    Serial.print(channel);
    Serial.print(F(": "));
    Serial.println(continuity ? "OK" : "ERREUR");
  #endif
  
  if (!continuity) {
    errorBeep(channel);
  }
}

// Séquence de bips d'erreur
void errorBeep(int count) {
  for(int i = 0; i < count; i++) {
    tone(PIN_SPEAKER, 300, 500);
    delay(600);
  }
}

// Annonce de la version
void announceVersion() {
  beepSequence(MAJOR_VERSION, 1000);   // Bips longs pour version majeure
  delay(300);
  beepSequence(MINOR_VERSION, 250);    // Bips courts pour version mineure
  delay(1000);
}

// Séquence de bips générique
void beepSequence(int count, int duration) {
  for(int i = 0; i < count; i++) {
    if(i > 0) delay(250);
    tone(PIN_SPEAKER, 600, duration);
    delay(duration);
  }
}

// Log des données de vol (débogage)
void logFlightData() {
  static unsigned long lastLog = 0;
  
  if (millis() - lastLog > 1000) {
    Serial.print(F("Etat: "));
    Serial.print(currentState);
    Serial.print(F(" Alt: "));
    Serial.print(currentAltitude);
    Serial.print(F(" Apogee: "));
    Serial.print(apogeeAltitude);
    Serial.print(F(" Compteur: "));
    Serial.println(apogeeConfirmationCounter);
    lastLog = millis();
  }
}

// Les fonctions beepAltitude(), beginBeepSequence(), etc. restent similaires
// mais peuvent être optimisées selon le même modèle