#include <Wire.h>
#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(10); // Pin CS del MCP2515

void setup() {
  Serial.begin(115200);

  // Inicializar MCP2515
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Serial.println("MCP2515 Inicializado - Emisor de voltaje y corriente");
}

void loop() {
  // Leer voltaje y corriente simulados (sustituir con sensores reales si es necesario)
  float voltage = analogRead(A0) * (5.0 / 1023.0) * 11.0; // Divisor resistivo 11:1
  float current = analogRead(A1) * (5.0 / 1023.0) * 10.0; // Sensor con escala de 10A

  // Enviar datos por CAN
  sendVoltageCurrentOverCAN(voltage, current);

  delay(50); // Igual que el código principal
}

void sendVoltageCurrentOverCAN(float voltage, float current) {
  struct can_frame canMsg;
  canMsg.can_dlc = 8;

  // Enviar Voltaje (ID: 0x200)
  canMsg.can_id = 0x200;
  memcpy(&canMsg.data[0], &voltage, sizeof(float));
  mcp2515.sendMessage(&canMsg);
  Serial.print("Enviado Voltaje=");
  Serial.println(voltage);
  delay(10);

  // Enviar Corriente (ID: 0x201)
  canMsg.can_id = 0x201;
  memcpy(&canMsg.data[0], &current, sizeof(float));
  mcp2515.sendMessage(&canMsg);
  Serial.print("Enviado Corriente=");
  Serial.println(current);
  delay(10);
}