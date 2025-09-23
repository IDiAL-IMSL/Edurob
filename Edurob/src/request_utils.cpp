#include <cmath>
#include <cctype>
#include <string>
#include <stdexcept>
#include <algorithm>


// Forward declarations for recursive descent parser
static double parse_expr(const std::string& s, size_t& i);
static double parse_term(const std::string& s, size_t& i);
static double parse_power(const std::string& s, size_t& i);
static double parse_primary(const std::string& s, size_t& i);
static double parse_number(const std::string& s, size_t& i);
static std::string parse_ident(const std::string& s, size_t& i);



static inline void skip_ws(const std::string& s, size_t& i) {
  while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
}

static bool match(const std::string& s, size_t& i, char c) {
  skip_ws(s, i);
  if (i < s.size() && s[i] == c) { ++i; return true; }
  return false;
}

static double parse_term(const std::string& s, size_t& i) {
  double v = parse_power(s, i);
  while (true) {
    if (match(s, i, '*')) { v *= parse_power(s, i); continue; }
    if (match(s, i, '/')) { v /= parse_power(s, i); continue; }
    break;
  }
  return v;
}

static double parse_expr(const std::string& s, size_t& i) {
  double v = parse_term(s, i);
  while (true) {
    if (match(s, i, '+')) { v += parse_term(s, i); continue; }
    if (match(s, i, '-')) { v -= parse_term(s, i); continue; }
    break;
  }
  return v;
}

static double parse_number(const std::string& s, size_t& i) {
  skip_ws(s, i);
  size_t start = i;
  bool has_dot = false;
  while (i < s.size()) {
    char c = s[i];
    if (std::isdigit(static_cast<unsigned char>(c))) { ++i; continue; }
    if (c == '.' && !has_dot) { has_dot = true; ++i; continue; }
    break;
  }
  if (start == i) throw std::runtime_error("number expected");
  return std::strtod(s.c_str() + start, nullptr);
}


static std::string parse_ident(const std::string& s, size_t& i) {
  skip_ws(s, i);
  size_t start = i;
  if (i < s.size() && (std::isalpha(static_cast<unsigned char>(s[i])) || s[i] == '_')) {
    ++i;
    while (i < s.size() && (std::isalnum(static_cast<unsigned char>(s[i])) || s[i] == '_')) ++i;
    return s.substr(start, i - start);
  }
  return "";
}

static double parse_primary(const std::string& s, size_t& i) {
  skip_ws(s, i);
  if (match(s, i, '(')) {
    double v = parse_expr(s, i);
    if (!match(s, i, ')')) throw std::runtime_error("')' expected");
    return v;
  }

  // function or constant
  {
    size_t save = i;
    std::string id = parse_ident(s, i);
    if (!id.empty()) {
      std::string idl = id;
      for (char& c : idl) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      // constant?
      if (idl == "pi") return M_PI;
      // function?
      if (match(s, i, '(')) {
        if (idl == "sqrt" || idl == "sin" || idl == "cos" || idl == "tan") {
          double a = parse_expr(s, i);
          if (!match(s, i, ')')) throw std::runtime_error("')' expected after arg");
          if (idl == "sqrt") return std::sqrt(a);
          if (idl == "sin")  return std::sin(a);
          if (idl == "cos")  return std::cos(a);
          if (idl == "tan") {
            double val = std::tan(a);
            if (val > 1000.0 || val < -1000.0) {
                return 1000;
            }
            return val;
          }
        } else if (idl == "pow") {
          double a = parse_expr(s, i);
          if (!match(s, i, ',')) throw std::runtime_error("',' expected in pow");
          double b = parse_expr(s, i);
          if (!match(s, i, ')')) throw std::runtime_error("')' expected after pow");
          return std::pow(a, b);
        } else {
          throw std::runtime_error("unknown function: " + id);
        }
      }
    
      i = save;
    }
  }

  // number sign
  if (match(s, i, '+')) return parse_primary(s, i);
  if (match(s, i, '-')) return -parse_primary(s, i);

  return parse_number(s, i);
}

static double parse_power(const std::string& s, size_t& i) {
  double v = parse_primary(s, i);
  while (true) {
    skip_ws(s, i);
    if (i < s.size() && s[i] == '^') {
      ++i;
      double r = parse_primary(s, i);
      v = std::pow(v, r);
    } else break;
  }
  return v;
}




//public API
double evalExpression(const std::string& expr) {
  size_t i = 0;
  double v = 0.0;
  try {
    v = parse_expr(expr, i);
    
    skip_ws(expr, i);
    while (i < expr.size() && expr[i] == ';') { ++i; skip_ws(expr, i); }
    if (i != expr.size()) throw std::runtime_error("unexpected trailing input");
  } catch (...) {
    return 1000;
  }
  if (std::isnan(v) || std::isinf(v)) return 1000;
  return v;
}
