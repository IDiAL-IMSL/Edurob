#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

#include <vector>
#include <string>   // <-- needed

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
extern int    encoderaufloesung;
extern int    uebersetzung;
extern std::string kinematikOption;

extern double translationalSpeed;
extern double translationalAcceleration;
extern double rotationalSpeed;
extern double rotationalAcceleration;
extern int    pointCount;

// Setter
void setRadabstand(double value);
void setAxenabstand(double value);
void setRadradius(double value);
void setEncoderaufloesung(int value);
void setUebersetzung(int value);
void setKinematikOption(const std::string& value);

void setTranslationalSpeed(double value);
void setTranslationalAcceleration(double value);
void setRotationalSpeed(double value);
void setRotationalAcceleration(double value);
void setPointCount(int value);

// Getter
double              getRadabstand();
double              getAxenabstand();
double              getRadradius();
int                 getEncoderaufloesung();
int                 getUebersetzung();
const std::string&  getKinematikOption();

double getTranslationalSpeed();
double getTranslationalAcceleration();
double getRotationalSpeed();
double getRotationalAcceleration();
int    getPointCount();

#endif // ROBOT_PARAMETERS_H
