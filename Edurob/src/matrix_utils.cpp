#include "matrix_utils.h"

#include <map>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>


static std::vector<std::vector<double>> mecanumMatrix(3, std::vector<double>(3, 0));
static std::vector<std::vector<double>> inverseMecanumMatrix(3, std::vector<double>(3, 0));
static double scalarMecanumMatrix = 0.0;
static double scalarInverseMecanumMatrix = 0.0;

void setMecanumMatrices(const std::map<std::string, double>& params) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            std::string key1 = std::to_string(1) + std::to_string(r + 1) + std::to_string(c + 1);
            std::string key2 = std::to_string(2) + std::to_string(r + 1) + std::to_string(c + 1);
            if (params.count(key1)) {
                mecanumMatrix[r][c] = params.at(key1);
            }
            if (params.count(key2)) {
                inverseMecanumMatrix[r][c] = params.at(key2);
            }
        }
    }
    if (params.count("1")) scalarMecanumMatrix = params.at("1");
    if (params.count("2")) scalarInverseMecanumMatrix = params.at("2");
}

std::vector<std::vector<double>> getMecanumMatrix() {
    return mecanumMatrix;
}

std::vector<std::vector<double>> getInverseMecanumMatrix() {
    return inverseMecanumMatrix;
}

double getScalarMecanumMatrix() {
    return scalarMecanumMatrix;
}

double getScalarInverseMecanumMatrix() {
    return scalarInverseMecanumMatrix;
}



static std::vector<std::vector<double>> omniFourMatrix(3, std::vector<double>(3, 0));
static std::vector<std::vector<double>> inverseOmniFourMatrix(3, std::vector<double>(3, 0));
static double scalarOmniFourMatrix = 0.0;
static double scalarInverseOmniFourMatrix = 0.0;

void setOmniFourMatrices(const std::map<std::string, double>& params) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            std::string key1 = std::to_string(1) + std::to_string(r + 1) + std::to_string(c + 1);
            std::string key2 = std::to_string(2) + std::to_string(r + 1) + std::to_string(c + 1);
            if (params.count(key1)) {
                omniFourMatrix[r][c] = params.at(key1);
            }
            if (params.count(key2)) {
                inverseOmniFourMatrix[r][c] = params.at(key2);
            }
        }
    }
    if (params.count("1")) scalarOmniFourMatrix = params.at("1");
    if (params.count("2")) scalarInverseOmniFourMatrix = params.at("2");
}

std::vector<std::vector<double>> getOmniFourMatrix() {
    return omniFourMatrix;
}

std::vector<std::vector<double>> getInverseOmniFourMatrix() {
    return inverseOmniFourMatrix;
}

double getScalarOmniFourMatrix() {
    return scalarOmniFourMatrix;
}

double getScalarInverseOmniFourMatrix() {
    return scalarInverseOmniFourMatrix;
}


static std::vector<std::vector<double>> omniThreeMatrix(3, std::vector<double>(3, 0));
static std::vector<std::vector<double>> inverseOmniThreeMatrix(3, std::vector<double>(3, 0));
static double scalarOmniThreeMatrix = 0.0;
static double scalarInverseOmniThreeMatrix = 0.0;

void setOmniThreeMatrices(const std::map<std::string, double>& params) {
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            std::string key1 = std::to_string(1) + std::to_string(r + 1) + std::to_string(c + 1);
            std::string key2 = std::to_string(2) + std::to_string(r + 1) + std::to_string(c + 1);
            if (params.count(key1)) {
                omniThreeMatrix[r][c] = params.at(key1);
            }
            if (params.count(key2)) {
                inverseOmniThreeMatrix[r][c] = params.at(key2);
            }
        }
    }
    if (params.count("1")) scalarOmniThreeMatrix = params.at("1");
    if (params.count("2")) scalarInverseOmniThreeMatrix = params.at("2");
}

std::vector<std::vector<double>> getOmniThreeMatrix() {
    return omniThreeMatrix;
}

std::vector<std::vector<double>> getInverseOmniThreeMatrix() {
    return inverseOmniThreeMatrix;
}

double getScalarOmniThreeMatrix() {
    return scalarOmniThreeMatrix;
}

double getScalarInverseOmniThreeMatrix() {
    return scalarInverseOmniThreeMatrix;
}

static std::vector<std::vector<double>> differentialMatrix(4, std::vector<double>(3, 0));
static std::vector<std::vector<double>> inverseDifferentialMatrix(3, std::vector<double>(4, 0));
static double scalarDifferentialMatrix = 0.0;
static double scalarInverseDifferentialMatrix = 0.0;

void setDifferentialMatrices(const std::map<std::string, double>& params) {
    // Setze 4x3 differentialMatrix
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 3; ++c) {
            std::string key = "1" + std::to_string(r + 1) + std::to_string(c + 1);
            if (params.count(key)) {
                differentialMatrix[r][c] = params.at(key);
            }
        }
    }

    // Setze 3x4 inverseDifferentialMatrix
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
            std::string key = "2" + std::to_string(r + 1) + std::to_string(c + 1);
            if (params.count(key)) {
                inverseDifferentialMatrix[r][c] = params.at(key);
            }
        }
    }

    // Skalare Einträge
    if (params.count("1")) scalarDifferentialMatrix = params.at("1");
    if (params.count("2")) scalarInverseDifferentialMatrix = params.at("2");
}


std::vector<std::vector<double>> getDifferentialMatrix() {
    return differentialMatrix;
}

std::vector<std::vector<double>> getInverseDifferentialMatrix() {
    return inverseDifferentialMatrix;
}

double getScalarDifferentialMatrix() {
    return scalarDifferentialMatrix;
}

double getScalarInverseDifferentialMatrix() {
    return scalarInverseDifferentialMatrix;
}

