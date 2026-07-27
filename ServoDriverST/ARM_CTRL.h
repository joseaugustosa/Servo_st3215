// Controlo do braço a1..a4 + garra via API HTTP.
// POST /api/arm  body: s=id,pos,spd,acc;id,pos,spd,acc;...

#ifndef ARM_CTRL_H
#define ARM_CTRL_H

#include <Arduino.h>

// Geometria default (mm) — alinhada com control/config.yaml
float ARM_L1 = 50.0f;
float ARM_L2 = 120.0f;
float ARM_L3 = 100.0f;
float ARM_L4 = 60.0f;

u8  ARM_IDS[5] = {1, 2, 3, 4, 5};  // a1 a2 a3 a4 gripper
s16 ARM_POS[5] = {2047, 2047, 2047, 2047, 2047};
u16 ARM_SPD[5] = {1500, 1500, 1500, 1500, 1500};
u8  ARM_ACC[5] = {50, 50, 50, 50, 50};

void armApplyPositions() {
  // Sync write ST3215 (tipo 9). Se o tipo ainda não foi detetado, tenta na mesma.
  st.SyncWritePosEx(ARM_IDS, 5, ARM_POS, ARM_SPD, ARM_ACC);
}

// Parse: "1,2047,1500,50;2,2100,1500,50;..."
bool armParseAndApply(const String& body) {
  int count = 0;
  int start = 0;
  String data = body;
  data.trim();
  // Aceitar prefixo "s=" (form) ou JSON mínimo com "s":
  int eq = data.indexOf('=');
  if (eq >= 0 && eq < 8) {
    data = data.substring(eq + 1);
  }
  // Remover aspas/chaves simples se vier algo tipo {"s":"..."} 
  data.replace("\"", "");
  data.replace("{", "");
  data.replace("}", "");
  data.replace("s:", "");
  data.replace("servos:", "");

  while (start < (int)data.length() && count < 5) {
    int semi = data.indexOf(';', start);
    String part = (semi < 0) ? data.substring(start) : data.substring(start, semi);
    part.trim();
    if (part.length() > 0) {
      int c1 = part.indexOf(',');
      int c2 = part.indexOf(',', c1 + 1);
      int c3 = part.indexOf(',', c2 + 1);
      if (c1 > 0 && c2 > c1 && c3 > c2) {
        int id  = part.substring(0, c1).toInt();
        int pos = part.substring(c1 + 1, c2).toInt();
        int spd = part.substring(c2 + 1, c3).toInt();
        int acc = part.substring(c3 + 1).toInt();
        if (pos < 0) pos = 0;
        if (pos > 4095) pos = 4095;
        if (spd < 0) spd = 0;
        if (spd > 4000) spd = 4000;
        if (acc < 0) acc = 0;
        if (acc > 254) acc = 254;
        ARM_IDS[count] = (byte)id;
        ARM_POS[count] = pos;
        ARM_SPD[count] = (u16)spd;
        ARM_ACC[count] = (u8)acc;
        count++;
      }
    }
    if (semi < 0) break;
    start = semi + 1;
  }

  if (count <= 0) return false;
  // Se vieram menos de 5, sync só os recebidos
  st.SyncWritePosEx(ARM_IDS, (u8)count, ARM_POS, ARM_SPD, ARM_ACC);
  return true;
}

void handleArmApi() {
  if (server.method() == HTTP_OPTIONS) {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    server.send(204);
    return;
  }

  server.sendHeader("Access-Control-Allow-Origin", "*");

  if (server.method() == HTTP_GET) {
    String out = "{\"L1\":" + String(ARM_L1, 1);
    out += ",\"L2\":" + String(ARM_L2, 1);
    out += ",\"L3\":" + String(ARM_L3, 1);
    out += ",\"L4\":" + String(ARM_L4, 1);
    out += ",\"ids\":[1,2,3,4,5]}";
    server.send(200, "application/json", out);
    return;
  }

  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Use POST");
    return;
  }

  String body = server.arg("plain");
  if (body.length() == 0) {
    // form field s=...
    if (server.hasArg("s")) body = server.arg("s");
  }
  if (body.length() == 0) {
    server.send(400, "text/plain", "empty body");
    return;
  }

  if (armParseAndApply(body)) {
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"parse\"}");
  }
}

#endif
