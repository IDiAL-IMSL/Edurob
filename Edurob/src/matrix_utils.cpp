#include "matrix_utils.h"

#include <map>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>

static std::vector<std::vector<double>> mecanumMatrix(4, std::vector<double>(3, 0));
static std::vector<std::vector<double>> inverseMecanumMatrix(3, std::vector<double>(4, 0));
static double scalarMecanumMatrix = 0.0;
static double scalarInverseMecanumMatrix = 0.0;

void setMecanumMatrices(const std::map<std::string, double>& params) {
    // Setze 4x3 mecanumMatrix
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 3; ++c) {
            std::string key = "1" + std::to_string(r + 1) + std::to_string(c + 1);
            if (params.count(key)) {
                mecanumMatrix[r][c] = params.at(key);
            }
        }
    }

    // Setze 3x4 inverseMecanumMatrix
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
            std::string key = "2" + std::to_string(r + 1) + std::to_string(c + 1);
            if (params.count(key)) {
                inverseMecanumMatrix[r][c] = params.at(key);
            }
        }
    }

    // Skalare Einträge
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



static std::vector<std::vector<double>> omniFourMatrix(4, std::vector<double>(3, 0));
static std::vector<std::vector<double>> inverseOmniFourMatrix(3, std::vector<double>(4, 0));
static double scalarOmniFourMatrix = 0.0;
static double scalarInverseOmniFourMatrix = 0.0;

void setOmniFourMatrices(const std::map<std::string, double>& params) {
    // Setze 4x3 omniFourMatrix 
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 3; ++c) {
            std::string key = "1" + std::to_string(r + 1) + std::to_string(c + 1);
            if (params.count(key)) {
                omniFourMatrix[r][c] = params.at(key);
            }
        }
    }

    // Setze 3x4 inverseOmniFourMatrix
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
            std::string key = "2" + std::to_string(r + 1) + std::to_string(c + 1);
            if (params.count(key)) {
                inverseOmniFourMatrix[r][c] = params.at(key);
            }
        }
    }

    // Skalare Einträge
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
// Forward decls of your existing setters
void setMecanumMatrices(const std::map<std::string, double>& params);
void setOmniFourMatrices(const std::map<std::string, double>& params);
void setOmniThreeMatrices(const std::map<std::string, double>& params);
void setDifferentialMatrices(const std::map<std::string, double>& params);

// ---------- Helpers to clear & set defaults ----------

/** MECANUM (4x3, 3x4) **/
void setMecanumNull() {
    std::map<std::string, double> p;

    // J (prefix "1rc")
    double J[4][3] = {
        { 0, 0, 0},
        { 0, 0, 0},
        { 0, 0, 0},
        { 0, 0, 0}
    };
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 3; ++c)
        p["1" + std::to_string(r+1) + std::to_string(c+1)] = J[r][c];

    // Jinv (prefix "2rc")
    double K[3][4] = {
        { 0, 0, 0, 0},
        { 0, 0, 0, 0},
        { 0, 0, 0, 0} // wz row (geometry applied at runtime)
    };
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 4; ++c)
        p["2" + std::to_string(r+1) + std::to_string(c+1)] = K[r][c];

    // Scalars
    p["1"] = 0;
    p["2"] = 0;
    setMecanumMatrices(p);
}

void setMecanumStandard() {
    // Template signs (no geometry); scalars are classic: 1 and 1/4
    // J (4x3): [ vx, vy, wz-sign ]
    //   [  1,  1,  1 ]
    //   [  1, -1, -1 ]
    //   [  1,  1, -1 ]
    //   [  1, -1,  1 ]
    // Jinv (3x4): rows [vx; vy; wz-sign]
    //   [  1,  1,  1,  1 ]
    //   [  1, -1,  1, -1 ]
    //   [  1, -1, -1,  1 ]  (later scaled by 1/(l1+l2))
    std::map<std::string, double> p;

    // J (prefix "1rc")
    double J[4][3] = {
        { 1,  1,  1},
        { 1, -1, -1},
        { 1,  1, -1},
        { 1, -1,  1}
    };
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 3; ++c)
        p["1" + std::to_string(r+1) + std::to_string(c+1)] = J[r][c];

    // Jinv (prefix "2rc")
    double K[3][4] = {
        { 1,  1,  1,  1},
        { 1, -1,  1, -1},
        { 1, -1, -1,  1} // wz row (geometry applied at runtime)
    };
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 4; ++c)
        p["2" + std::to_string(r+1) + std::to_string(c+1)] = K[r][c];

    // Scalars
    p["1"] = 1.0;        // scalarMecanumMatrix
    p["2"] = 1.0/4.0;    // scalarInverseMecanumMatrix

    setMecanumMatrices(p);
}

/** OMNI-4 (4x3, 3x4) **/
void setOmniFourNull() {
    std::map<std::string, double> p;

    // J (prefix "1rc")
    double J[4][3] = {
        { 0, 0, 0},
        { 0, 0, 0},
        { 0, 0, 0},
        { 0, 0, 0}
    };
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 3; ++c)
        p["1" + std::to_string(r+1) + std::to_string(c+1)] = J[r][c];

    // Jinv (prefix "2rc")
    double K[3][4] = {
        { 0, 0, 0, 0},
        { 0, 0, 0, 0},
        { 0, 0, 0, 0} // wz row (geometry applied at runtime)
    };
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 4; ++c)
        p["2" + std::to_string(r+1) + std::to_string(c+1)] = K[r][c];

    // Scalars
    p["1"] = 0;
    p["2"] = 0;
    setOmniFourMatrices(p);
}

void setOmniFourStandard() {
    // Same sign template as mecanum; different scalars: √2/2 and √2/4
    std::map<std::string, double> p;

    double J[4][3] = {
        { 1,  1,  1},
        { 1, -1, -1},
        { 1,  1, -1},
        { 1, -1,  1}
    };
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 3; ++c)
        p["1" + std::to_string(r+1) + std::to_string(c+1)] = J[r][c];

    double K[3][4] = {
        { 1,  1,  1,  1},
        { 1, -1,  1, -1},
        { 1, -1, -1,  1} // wz row
    };
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 4; ++c)
        p["2" + std::to_string(r+1) + std::to_string(c+1)] = K[r][c];

    p["1"] = std::sqrt(2.0)/2.0;     // scalarOmniFourMatrix
    p["2"] = std::sqrt(2.0)/4.0;     // scalarInverseOmniFourMatrix

    setOmniFourMatrices(p);
}

/** OMNI-3 (3x3, 3x3) **/
void setOmniThreeNull() {
    std::map<std::string, double> p;

    // J (prefix "1rc")
    double J[4][3] = {
        { 0, 0, 0},
        { 0, 0, 0},
        { 0, 0, 0},
        { 0, 0, 0}
    };
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 3; ++c)
        p["1" + std::to_string(r+1) + std::to_string(c+1)] = J[r][c];

    // Jinv (prefix "2rc")
    double K[3][4] = {
        { 0, 0, 0, 0},
        { 0, 0, 0, 0},
        { 0, 0, 0, 0} // wz row (geometry applied at runtime)
    };
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 4; ++c)
        p["2" + std::to_string(r+1) + std::to_string(c+1)] = K[r][c];

    // Scalars
    p["1"] = 0;
    p["2"] = 0;
    setOmniThreeMatrices(p);
}

void setOmniThreeStandard() {
    // Template from your original (no l1 baked in; wz column/row capture sign only)
    std::map<std::string, double> p;

    // J (3x3)
    double J[3][3] = {
        { -std::sqrt(3.0)/2.0, -0.5,  -1.0 },
        {  std::sqrt(3.0)/2.0, -0.5,  -1.0 },
        {  0.0,                 1.0,  -1.0 }
    };
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        p["1" + std::to_string(r+1) + std::to_string(c+1)] = J[r][c];

    // Jinv (3x3)
    double K[3][3] = {
        { -std::sqrt(3.0),  std::sqrt(3.0), 0.0 },
        { -1.0,            -1.0,            2.0 },
        { -1.0,            -1.0,           -1.0 } // wz row (scaled by 1/l1 at runtime)
    };
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        p["2" + std::to_string(r+1) + std::to_string(c+1)] = K[r][c];

    p["1"] = 1.0;        // scalarOmniThreeMatrix
    p["2"] = 1.0/3.0;    // scalarInverseOmniThreeMatrix

    setOmniThreeMatrices(p);
}

/** DIFFERENTIAL (4x3, 3x4) **/
void setDifferentialNull() {
    std::map<std::string, double> p;

    // J (prefix "1rc")
    double J[4][3] = {
        { 0, 0, 0},
        { 0, 0, 0},
        { 0, 0, 0},
        { 0, 0, 0}
    };
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 3; ++c)
        p["1" + std::to_string(r+1) + std::to_string(c+1)] = J[r][c];

    // Jinv (prefix "2rc")
    double K[3][4] = {
        { 0, 0, 0, 0},
        { 0, 0, 0, 0},
        { 0, 0, 0, 0} // wz row (geometry applied at runtime)
    };
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 4; ++c)
        p["2" + std::to_string(r+1) + std::to_string(c+1)] = K[r][c];

    // Scalars
    p["1"] = 0.0;
    p["2"] = 0.0;
    setDifferentialMatrices(p);
}

void setDifferentialStandard() {
    // Template signs; scalars 1 and 1/4
    std::map<std::string, double> p;

    double J[4][3] = {
        { 1, 0,  1},
        { 1, 0, -1},
        { 1, 0, -1},
        { 1, 0,  1}
    };
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 3; ++c)
        p["1" + std::to_string(r+1) + std::to_string(c+1)] = J[r][c];

    double K[3][4] = {
        { 1,  1,  1,  1},
        { 0,  0,  0,  0},
        { 1, -1, -1,  1} // wz row (scaled by 1/l2 at runtime)
    };
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 4; ++c)
        p["2" + std::to_string(r+1) + std::to_string(c+1)] = K[r][c];

    p["1"] = 1.0;        // scalarDifferentialMatrix
    p["2"] = 1.0/4.0;    // scalarInverseDifferentialMatrix

    setDifferentialMatrices(p);
}


