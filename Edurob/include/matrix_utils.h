#pragma once

#include <string>
#include <map>
#include <vector>

// Dateiinhalte lesen
std::string readFile(const std::string& filePath);

// Mecanum-Matrix setzen/holen
void setMecanumMatrices(const std::map<std::string, double>& params);
void setMecanumNull();
void setMecanumStandard();
std::vector<std::vector<double>> getMecanumMatrix();
std::vector<std::vector<double>> getInverseMecanumMatrix();
double getScalarMecanumMatrix();
double getScalarInverseMecanumMatrix();


// OmniFour-Matrix setzen/holen
void setOmniFourMatrices(const std::map<std::string, double>& params);
void setOmniFourNull();
void setOmniFourStandard();
std::vector<std::vector<double>> getOmniFourMatrix();
std::vector<std::vector<double>> getInverseOmniFourMatrix();
double getScalarOmniFourMatrix();
double getScalarInverseOmniFourMatrix();


// OmniThree-Matrix setzen/holen
void setOmniThreeMatrices(const std::map<std::string, double>& params);
void setOmniThreeNull();
void setOmniThreeStandard();
std::vector<std::vector<double>> getOmniThreeMatrix();
std::vector<std::vector<double>> getInverseOmniThreeMatrix();
double getScalarOmniThreeMatrix();
double getScalarInverseOmniThreeMatrix();

// Differential-Matrix setzen/holen
void setDifferentialMatrices(const std::map<std::string, double>& params);
void setDifferentialNull();
void setDifferentialStandard();
std::vector<std::vector<double>> getDifferentialMatrix();
std::vector<std::vector<double>> getInverseDifferentialMatrix();
double getScalarDifferentialMatrix();
double getScalarInverseDifferentialMatrix();