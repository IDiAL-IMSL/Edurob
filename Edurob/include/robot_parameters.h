#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H


#include <vector>

struct Pose {
    double x;
    double y;
    double theta;
};

// Globale Variable zur Speicherung der Posen
extern std::vector<Pose> punkte;

// Setter & Getter
void addPunkt(double x, double y, double theta);
const std::vector<Pose>& getPunkte();
void clearPunkte();





// Globale Parameter-Deklarationen (extern)
extern double radabstand;
extern double axenabstand;
extern double radradius;
extern double encoderaufloesung;
extern double uebersetzung;
extern int kinematik;

extern double translationalSpeed;
extern double translationalAcceleration;
extern double rotationalSpeed;
extern double rotationalAcceleration;
extern int pointCount;

// Setter
void setRadabstand(double value);
void setAxenabstand(double value);
void setRadradius(double value);
void setEncoderaufloesung(double value);
void setUebersetzung(double value);
void setKinematik(int value);

void setTranslationalSpeed(double value);
void setTranslationalAcceleration(double value);
void setRotationalSpeed(double value);
void setRotationalAcceleration(double value);
void setPointCount(int value);


// Getter
double getRadabstand();
double getAxenabstand();
double getRadradius();
double getEncoderaufloesung();
double getUebersetzung();
int    getKinematik();

double getTranslationalSpeed();
double getTranslationalAcceleration();
double getRotationalSpeed();
double getRotationalAcceleration();
int    getPointCount();


#endif // ROBOT_PARAMETERS_H
