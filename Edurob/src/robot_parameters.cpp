#include <string>
#include <robot_parameters.h>


// Globale Vektor-Definition
std::vector<Pose> punkte;

// Punkt hinzufügen
void addPunkt(double x, double y, double theta) {
    punkte.push_back({x, y, theta});
}

// Alle Punkte zurückgeben
const std::vector<Pose>& getPunkte() {
    return punkte;
}

// Punkte löschen (falls nötig z. B. bei Reset)
void clearPunkte() {
    punkte.clear();
}

// Globale Parameter (numerisch)
double radabstand = 0.1234;
double axenabstand = 0.5678;
double radradius = 1.2345;
double encoderaufloesung = 28;
double uebersetzung = 100;
int kinematik = 0;

double translationalSpeed = 0.15;
double translationalAcceleration = 0.05;
double rotationalSpeed = 0.2;
double rotationalAcceleration = 0.05;
int pointCount = 0;
            

// Setter
void setRadabstand(double value)        { radabstand = value; }
void setAxenabstand(double value)       { axenabstand = value; }
void setRadradius(double value)         { radradius = value; }
void setEncoderaufloesung(double value) { encoderaufloesung = value; }
void setUebersetzung(double value)      { uebersetzung = value; }
void setKinematik(int value) { kinematik = value; }


void setTranslationalSpeed(double value)         { translationalSpeed = value; }
void setTranslationalAcceleration(double value) { translationalAcceleration = value; }
void setRotationalSpeed(double value)      { rotationalSpeed = value; }
void setRotationalAcceleration(double value)      { rotationalAcceleration = value; }
void setPointCount(int value) { pointCount = value; }

// Getter
double getRadabstand()        { return radabstand; }
double getAxenabstand()       { return axenabstand; }
double getRadradius()         { return radradius; }
double getEncoderaufloesung() { return encoderaufloesung; }
double getUebersetzung()      { return uebersetzung; }
int getKinematik()    { return kinematik; }

double getTranslationalSpeed()         { return translationalSpeed; }
double getTranslationalAcceleration() { return translationalAcceleration; }
double getRotationalSpeed() { return rotationalSpeed; }
double getRotationalAcceleration()      { return rotationalAcceleration; }
int getPointCount()    { return pointCount; }
