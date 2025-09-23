// request_utils.h
// Expression evaluator - public interface

#ifndef REQUEST_UTILS_H
#define REQUEST_UTILS_H

#include <string>

// Evaluates a mathematical expression given as a string.
// Supports +, -, *, /, ^, parentheses, constants (pi),
// and functions like sqrt, sin, cos, tan, pow.
// Returns 1000 on error or invalid input.
double evalExpression(const std::string& expr);

#endif // REQUEST_UTILS_H