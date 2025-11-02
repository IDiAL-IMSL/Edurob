#include "server_esp.h"
#include <cstring>
#include <string>
#include <map>
#include <vector>
#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"

#include "request_utils.h"
#include "matrix_utils.h"
#include "resources.h"
#include "robot_parameters.h"

#define LOG_TAG "Server"
#include "logger.h"
static Logger LOG(LOG_TAG);

// ---- tiny helpers ----------------------------------------------------------
static inline void send_text(httpd_req_t* req, const char* type, const char* body) {
  httpd_resp_set_type(req, type);
  httpd_resp_sendstr(req, body);
}
// Decode one application/x-www-form-urlencoded component.
// - '+' -> ' '
// - %HH -> byte with hex HH (UTF-8 safe: bytes are preserved)
static std::string decode_form_component(std::string in) {
  std::string out;
  out.reserve(in.size());
  for (size_t i = 0; i < in.size(); ++i) {
    unsigned char c = static_cast<unsigned char>(in[i]);
    if (c == '+') {
      out.push_back(' ');
    } else if (c == '%' && i + 2 < in.size()) {
      auto hex = [](char ch) -> int {
        if (ch >= '0' && ch <= '9') return ch - '0';
        if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
        if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
        return -1;
      };
      int hi = hex(in[i+1]);
      int lo = hex(in[i+2]);
      if (hi >= 0 && lo >= 0) {
        out.push_back(static_cast<char>((hi << 4) | lo));
        i += 2;
      } else {
        // Invalid % sequence → keep it literally
        out.push_back('%');
      }
    } else {
      out.push_back(static_cast<char>(c));
    }
  }
  return out;
}

static std::map<std::string, std::string> parse_query(httpd_req_t* req) {
  std::map<std::string, std::string> kv;

  size_t len = httpd_req_get_url_query_len(req);
  if (len == 0) return kv;

  std::vector<char> buf(len + 1);
  if (httpd_req_get_url_query_str(req, buf.data(), buf.size()) != ESP_OK) {
    LOG.i("Failed to get query string");
    return kv;
  }

  // Collect unique raw keys from the raw query
  std::string q(buf.data());
  std::map<std::string, bool> seen;
  for (size_t pos = 0; pos < q.size(); ) {
    size_t amp = q.find('&', pos);
    std::string pair = q.substr(pos, (amp == std::string::npos) ? std::string::npos : amp - pos);
    size_t eq = pair.find('=');
    if (eq != std::string::npos) {
      std::string raw_key = pair.substr(0, eq);
      if (!raw_key.empty()) seen[raw_key] = true;
    } else if (!pair.empty()) {
      seen[pair] = true; // key present without '='
    }
    if (amp == std::string::npos) break;
    pos = amp + 1;
  }

  // For each key, fetch value, then decode both key and value
  for (const auto& it : seen) {
    const std::string& raw_k = it.first;

    std::vector<char> val(len + 1);
    if (httpd_query_key_value(buf.data(), raw_k.c_str(), val.data(), val.size()) == ESP_OK) {
      std::string dec_k = decode_form_component(raw_k);
      std::string dec_v = decode_form_component(val.data());
      kv[dec_k] = dec_v;
      LOG.i("%s = %s", dec_k.c_str(), dec_v.c_str());
    } else {
      std::string dec_k = decode_form_component(raw_k);
      // Key without value
      kv.emplace(dec_k, "");
      LOG.i("%s present without value", dec_k.c_str());
    }
  }

  return kv;
}


static inline double dget(const std::map<std::string,std::string>& kv, const char* key, double def=0.0) {
  auto it = kv.find(key);
  if (it == kv.end()) return def;
  return evalExpression(it->second);
}
static inline int iget(const std::map<std::string,std::string>& kv, const char* key, int def=0) {
  auto it = kv.find(key);
  if (it == kv.end()) return def;
  return (int)evalExpression(it->second);
}

// ---- JSON builders (mirror your sse_handler.cpp) ----------------------------


static std::string json_matrix_3x3(double F[3][3], double I[3][3], double s1, double s2){
  char buf[768];
  // compact JSON to match your front-end (matrix, inverse, scalar1, scalar2)
  int n = snprintf(buf, sizeof(buf),
    "{\"matrix\":[[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f]],"
    "\"inverse\":[[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f]],"
    "\"scalar1\":%.6f,\"scalar2\":%.6f}",
    F[0][0],F[0][1],F[0][2], F[1][0],F[1][1],F[1][2], F[2][0],F[2][1],F[2][2],
    I[0][0],I[0][1],I[0][2], I[1][0],I[1][1],I[1][2], I[2][0],I[2][1],I[2][2],
    s1, s2);
  return std::string(buf, (size_t)n);
}

static std::string json_matrix_diff(double F[4][3], double I[3][4], double s1, double s2){
  char buf[1024];
  int n = snprintf(buf, sizeof(buf),
    "{\"matrix\":[[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f]],"
    "\"inverse\":[[%.6f,%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f,%.6f]],"
    "\"scalar1\":%.6f,\"scalar2\":%.6f}",
    F[0][0],F[0][1],F[0][2], F[1][0],F[1][1],F[1][2], F[2][0],F[2][1],F[2][2], F[3][0],F[3][1],F[3][2],
    I[0][0],I[0][1],I[0][2],I[0][3], I[1][0],I[1][1],I[1][2],I[1][3], I[2][0],I[2][1],I[2][2],I[2][3],
    s1, s2);
  return std::string(buf, (size_t)n);
}

static std::string json_configuration(){
  int k = getKinematik();
  const char* ks = (k==0?"Mecanum":k==1?"Omni4":k==2?"Omni3":k==3?"Diff":"Mecanum");
  char tmp[384];
  snprintf(tmp, sizeof(tmp),
    "{\"radabstand\":%.6f,\"axenabstand\":%.6f,\"radradius\":%.6f,"
    "\"encoderaufloesung\":%.0f,\"uebersetzung\":%.0f,\"kinematik\":\"%s\"}",
    getRadabstand(), getAxenabstand(), getRadradius(),
    getEncoderaufloesung(), getUebersetzung(), ks);
  return std::string(tmp);
}
// server_esp.cpp

static std::string json_pathplanning() {
  // Gather values from robot_parameters
  double vT  = getTranslationalSpeed();
  double aT  = getTranslationalAcceleration();
  double vR  = getRotationalSpeed();
  double aR  = getRotationalAcceleration();

  const auto& pts = getPunkte();
  int n = static_cast<int>(pts.size());  // prefer actual size over requested count


  std::string out;
  out.reserve(256 + n * 48); // rough prealloc to avoid reallocs

  char hdr[256];
  int m = snprintf(hdr, sizeof(hdr),
    "{\"translationalSpeed\":%.6f,"
    "\"translationalAcceleration\":%.6f,"
    "\"rotationalSpeed\":%.6f,"
    "\"rotationalAcceleration\":%.6f,"
    "\"pointCount\":%d,"
    "\"points\":[",
    vT, aT, vR, aR, n);
  out.append(hdr, (size_t)m);

  for (int i = 0; i < n; ++i) {
    char pb[128];
    // Pose is assumed as {x,y,theta}
    m = snprintf(pb, sizeof(pb),
      "%s{\"x\":%.6f,\"y\":%.6f,\"theta\":%.6f}",
      (i ? "," : ""), pts[i].x, pts[i].y, pts[i].theta);
    out.append(pb, (size_t)m);
  }

  out += "]}";
  return out;
}



static std::string json_mecanum(){
  auto F = getMecanumMatrix();
  auto I = getInverseMecanumMatrix();
  double s1 = getScalarMecanumMatrix();
  double s2 = getScalarInverseMecanumMatrix();

  char buf[1024];
  int n = snprintf(buf, sizeof(buf),
    "{\"matrix\":[[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f]],"
    "\"inverse\":[[%.6f,%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f,%.6f]],"
    "\"scalar1\":%.6f,\"scalar2\":%.6f}",
    F[0][0],F[0][1],F[0][2], F[1][0],F[1][1],F[1][2], F[2][0],F[2][1],F[2][2], F[3][0],F[3][1],F[3][2],
    I[0][0],I[0][1],I[0][2],I[0][3], I[1][0],I[1][1],I[1][2],I[1][3], I[2][0],I[2][1],I[2][2],I[2][3],
    s1, s2);
  return std::string(buf, (size_t)n);
}

static std::string json_omnifour(){
  auto F = getOmniFourMatrix();
  auto I = getInverseOmniFourMatrix();
  double s1 = getScalarOmniFourMatrix();
  double s2 = getScalarInverseOmniFourMatrix();

  char buf[1024];
  int n = snprintf(buf, sizeof(buf),
    "{\"matrix\":[[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f]],"
    "\"inverse\":[[%.6f,%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f,%.6f]],"
    "\"scalar1\":%.6f,\"scalar2\":%.6f}",
    F[0][0],F[0][1],F[0][2], F[1][0],F[1][1],F[1][2], F[2][0],F[2][1],F[2][2], F[3][0],F[3][1],F[3][2],
    I[0][0],I[0][1],I[0][2],I[0][3], I[1][0],I[1][1],I[1][2],I[1][3], I[2][0],I[2][1],I[2][2],I[2][3],
    s1, s2);
  return std::string(buf, (size_t)n);
}

static std::string json_omnithree(){
  auto F = getOmniThreeMatrix();
  auto I = getInverseOmniThreeMatrix();
  double s1 = getScalarOmniThreeMatrix();
  double s2 = getScalarInverseOmniThreeMatrix();

  char buf[768];
  int n = snprintf(buf, sizeof(buf),
    "{\"matrix\":[[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f]],"
    "\"inverse\":[[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f]],"
    "\"scalar1\":%.6f,\"scalar2\":%.6f}",
    F[0][0],F[0][1],F[0][2], F[1][0],F[1][1],F[1][2], F[2][0],F[2][1],F[2][2],
    I[0][0],I[0][1],I[0][2], I[1][0],I[1][1],I[1][2], I[2][0],I[2][1],I[2][2],
    s1, s2);
  return std::string(buf, (size_t)n);
}

static std::string json_differential(){
  auto F = getDifferentialMatrix();       
  auto I = getInverseDifferentialMatrix();  
  double s1 = getScalarDifferentialMatrix();
  double s2 = getScalarInverseDifferentialMatrix();

  char buf[1024];
  int n = snprintf(buf, sizeof(buf),
    "{\"matrix\":[[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f]],"
    "\"inverse\":[[%.6f,%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f,%.6f],[%.6f,%.6f,%.6f,%.6f]],"
    "\"scalar1\":%.6f,\"scalar2\":%.6f}",
    F[0][0],F[0][1],F[0][2], F[1][0],F[1][1],F[1][2], F[2][0],F[2][1],F[2][2], F[3][0],F[3][1],F[3][2],
    I[0][0],I[0][1],I[0][2],I[0][3], I[1][0],I[1][1],I[1][2],I[1][3], I[2][0],I[2][1],I[2][2],I[2][3],
    s1, s2);
  return std::string(buf, (size_t)n);
}




// ---- URI HANDLERS (pages) ---------------------------------------------------

static esp_err_t handle_root(httpd_req_t* req){ send_text(req, "text/html", hauptmenu); return ESP_OK; }
static esp_err_t handle_kinlinks(httpd_req_t* req){ send_text(req, "text/html", kinematiklinks); return ESP_OK; }
static esp_err_t handle_configuration(httpd_req_t* req){
  auto kv = parse_query(req);
  if (kv.size()) {
    if (kv.count("radabstand"))        setRadabstand(dget(kv, "radabstand"));
    if (kv.count("axenabstand"))       setAxenabstand(dget(kv, "axenabstand"));
    if (kv.count("radradius"))         setRadradius(dget(kv, "radradius"));
    if (kv.count("encoderaufloesung")) setEncoderaufloesung(dget(kv, "encoderaufloesung"));
    if (kv.count("uebersetzung"))      setUebersetzung(dget(kv, "uebersetzung"));
    if (kv.count("kinematicOption"))   setKinematik(iget(kv, "kinematicOption"));
  }
  send_text(req, "text/html", configuration); return ESP_OK;
}

static void apply_3x3_mats(const std::map<std::string,std::string>& kv,
                           void (*setter)(const std::map<std::string,double>&)) {
  std::map<std::string,double> p;
  for (int r=1;r<=3;++r) for (int c=1;c<=3;++c) {
    std::string k1 = "1"+std::to_string(r)+std::to_string(c);
    std::string k2 = "2"+std::to_string(r)+std::to_string(c);
    if (kv.count(k1)) p[k1] = dget(kv, k1.c_str());
    if (kv.count(k2)) p[k2] = dget(kv, k2.c_str());
  }
  if (kv.count("1")) p["1"] = dget(kv, "1");
  if (kv.count("2")) p["2"] = dget(kv, "2");
  if (!p.empty()) setter(p);
}

static void apply_4x3_mats(const std::map<std::string,std::string>& kv,
                           void (*setter)(const std::map<std::string,double>&)) {

   std::map<std::string,double> p;
    // forward 4x3
    for (int r=1;r<=4;++r) for (int c=1;c<=3;++c) {
      std::string k = "1"+std::to_string(r)+std::to_string(c);
      if (kv.count(k)) p[k] = dget(kv, k.c_str());
    }
    // inverse 3x4
    for (int r=1;r<=3;++r) for (int c=1;c<=4;++c) {
      std::string k = "2"+std::to_string(r)+std::to_string(c);
      if (kv.count(k)) p[k] = dget(kv, k.c_str());
    }
    if (kv.count("1")) p["1"] = dget(kv, "1");
    if (kv.count("2")) p["2"] = dget(kv, "2");
    if (!p.empty()) setter(p);
}


void setMecanumMatrices(const std::map<std::string,double>& p);
void setOmniFourMatrices(const std::map<std::string,double>& p);
void setOmniThreeMatrices(const std::map<std::string,double>& p);
void setDifferentialMatrices(const std::map<std::string,double>& p);

static esp_err_t handle_mecanum(httpd_req_t* req){
  auto kv = parse_query(req);
  if (!kv.empty()) apply_4x3_mats(kv, setMecanumMatrices);
  send_text(req, "text/html", mecanum); return ESP_OK;
}
static esp_err_t handle_omnifour(httpd_req_t* req){
  auto kv = parse_query(req);
  if (!kv.empty()) apply_4x3_mats(kv, setOmniFourMatrices);
  send_text(req, "text/html", omni4); return ESP_OK;
}
static esp_err_t handle_omnithree(httpd_req_t* req){
  auto kv = parse_query(req);
  if (!kv.empty()) apply_3x3_mats(kv, setOmniThreeMatrices);
  send_text(req, "text/html", omni3); return ESP_OK;
}
static esp_err_t handle_differential(httpd_req_t* req){
  auto kv = parse_query(req);
  if (!kv.empty()) apply_4x3_mats(kv, setDifferentialMatrices);
  send_text(req, "text/html", differential); return ESP_OK;
}

static esp_err_t handle_pathinterpolator(httpd_req_t* req){
  send_text(req, "text/html", pathinterpolator); return ESP_OK;
}
static esp_err_t handle_pathplanning(httpd_req_t* req){
  auto kv = parse_query(req);

  if (!kv.empty()){
    clearPunkte();
    if (kv.count("translationalSpeed"))        setTranslationalSpeed(dget(kv,"translationalSpeed"));
    if (kv.count("translationalAcceleration")) setTranslationalAcceleration(dget(kv,"translationalAcceleration"));
    if (kv.count("rotationalSpeed"))           setRotationalSpeed(dget(kv,"rotationalSpeed"));
    if (kv.count("rotationalAcceleration"))    setRotationalAcceleration(dget(kv,"rotationalAcceleration"));
    if (kv.count("pointCount"))                setPointCount(iget(kv,"pointCount"));
    // waypoints: x1,y1,theta1 ... xN,yN,thetaN
    for (int i=1;i<=getPointCount();++i){
      std::string kx="x"+std::to_string(i), ky="y"+std::to_string(i), kt="theta"+std::to_string(i);
      if (kv.count(kx) && kv.count(ky) && kv.count(kt)) {
        addPunkt(dget(kv,kx.c_str()), dget(kv,ky.c_str()), dget(kv,kt.c_str()));
      }
    }
    send_text(req, "text/html", pathinterpolator); return ESP_OK;
  }else{
    send_text(req, "text/html", pathinterpolator); return ESP_OK;
  }
  
}

static esp_err_t handle_reboot(httpd_req_t* req){
  // optional: read "selected" and act
  send_text(req, "text/plain", "OK");
  // esp_restart();  // if desired
  return ESP_OK;
}


static esp_err_t handle_styles(httpd_req_t* req){
  httpd_resp_set_type(req, "text/css");
  return httpd_resp_sendstr(req, styles);
}

static esp_err_t handle_reset(httpd_req_t* req){
  setMecanumNull();
  setOmniThreeNull();
  setOmniFourNull();
  setDifferentialNull();
  return handle_root(req);
}
static esp_err_t handle_standard(httpd_req_t* req){
  setMecanumStandard();
  setOmniThreeStandard();
  setOmniFourStandard();
  setDifferentialStandard();
  return handle_root(req);
}

// ---- SSE handler ------------------------------------------------------------
// Keeps the socket open and periodically pushes "data: <json>\n\n"
static bool client_connected(httpd_req_t* req) {
  int sock = httpd_req_to_sockfd(req);
  if (sock < 0) return false;
  fd_set rfds; timeval tv{0,0};
  FD_ZERO(&rfds); FD_SET(sock, &rfds);
  // a readable socket doesn’t necessarily mean closed, but we'll rely on send() return to detect closure instead
  return true;
}


static esp_err_t handle_sse(httpd_req_t* req){
  auto kv = parse_query(req);
  std::string type = kv.count("type") ? kv["type"] : "index";

  // Required for SSE
  httpd_resp_set_type(req, "text/event-stream; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  httpd_resp_set_hdr(req, "Connection", "close");  // we’re sending once, then closing

  auto build = [&]()->std::string{
    if (type=="index")         return json_configuration();
    if (type=="configuration") return json_configuration();
    if (type=="mecanum")       return json_mecanum();
    if (type=="omnifour")      return json_omnifour();
    if (type=="omnithree")     return json_omnithree();
    if (type=="differential")  return json_differential();
    if (type=="pathplanning")  return json_pathplanning();
    return std::string("{\"ok\":true}");
  };

  std::string j = build();
  if (httpd_resp_sendstr_chunk(req, "event: message\n") != ESP_OK) return ESP_FAIL;
  if (httpd_resp_sendstr_chunk(req, "data: ")            != ESP_OK) return ESP_FAIL;
  if (httpd_resp_send_chunk(req, j.data(), j.size())     != ESP_OK) return ESP_FAIL;
  if (httpd_resp_sendstr_chunk(req, "\n\n")              != ESP_OK) return ESP_FAIL;

  // IMPORTANT: end the chunked response
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}




// ---- server bootstrap -------------------------------------------------------

static httpd_handle_t s_server = nullptr;

static void register_routes(httpd_handle_t s) {
  httpd_uri_t root = { .uri="/", .method=HTTP_GET, .handler=handle_root, .user_ctx=NULL };
  httpd_register_uri_handler(s, &root);

  httpd_uri_t kin = { .uri="/kinematiklinks", .method=HTTP_GET, .handler=handle_kinlinks, .user_ctx=NULL };
  httpd_register_uri_handler(s, &kin);

  httpd_uri_t cfg = { .uri="/configuration", .method=HTTP_GET, .handler=handle_configuration, .user_ctx=NULL };
  httpd_register_uri_handler(s, &cfg);

  httpd_uri_t meca = { .uri="/mecanum", .method=HTTP_GET, .handler=handle_mecanum, .user_ctx=NULL };
  httpd_register_uri_handler(s, &meca);

  httpd_uri_t o4 = { .uri="/omnifour", .method=HTTP_GET, .handler=handle_omnifour, .user_ctx=NULL };
  httpd_register_uri_handler(s, &o4);

  httpd_uri_t o3 = { .uri="/omnithree", .method=HTTP_GET, .handler=handle_omnithree, .user_ctx=NULL };
  httpd_register_uri_handler(s, &o3);

  httpd_uri_t diff = { .uri="/differential", .method=HTTP_GET, .handler=handle_differential, .user_ctx=NULL };
  httpd_register_uri_handler(s, &diff);

  httpd_uri_t pathi = { .uri="/pathinterpolator", .method=HTTP_GET, .handler=handle_pathinterpolator, .user_ctx=NULL };
  httpd_register_uri_handler(s, &pathi);

  httpd_uri_t pathp = { .uri="/pathplanning", .method=HTTP_GET, .handler=handle_pathplanning, .user_ctx=NULL };
  httpd_register_uri_handler(s, &pathp);

  httpd_uri_t reb = { .uri="/reboot", .method=HTTP_GET, .handler=handle_reboot, .user_ctx=NULL };
  httpd_register_uri_handler(s, &reb);

  httpd_uri_t sse = { .uri="/sse", .method=HTTP_GET, .handler=handle_sse, .user_ctx=NULL };
  httpd_register_uri_handler(s, &sse);

  httpd_uri_t css = { .uri="/styles.css", .method=HTTP_GET, .handler=handle_styles, .user_ctx=NULL };
  httpd_register_uri_handler(s, &css);

  //trust?
  httpd_uri_t reset = { .uri="/reset", .method=HTTP_GET, .handler=handle_reset, .user_ctx=NULL };
  httpd_register_uri_handler(s, &reset);
  httpd_uri_t standart = { .uri="/standart", .method=HTTP_GET, .handler=handle_standard, .user_ctx=NULL };
  httpd_register_uri_handler(s, &standart);
}

extern "C" httpd_handle_t start_webserver(void) {
  LOG.i("Webserver Starting");
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.max_uri_handlers = 16;
  cfg.lru_purge_enable = true;
  cfg.recv_wait_timeout = 10;
  cfg.send_wait_timeout = 10;
  cfg.stack_size = 8192; 
  httpd_handle_t server = nullptr;
  if (httpd_start(&server, &cfg) == ESP_OK) {
    register_routes(server);
    LOG.i("HTTP server started on port %d", cfg.server_port);
  }
  LOG.i("Webserver Started");
  return server;
}

extern "C" void stop_webserver(httpd_handle_t server) {
  if (server) {
    httpd_stop(server);
    LOG.i("HTTP server stopped");
  }
}