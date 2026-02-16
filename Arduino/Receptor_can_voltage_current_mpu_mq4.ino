#include <SPI.h>
#include <mcp2515.h>

// MCP2515
MCP2515 mcp2515(10); // Pin CS del MCP2515

// Variables para almacenar los datos recibidos
float Roll, Pitch, Yaw;
float RateRoll, RatePitch, RateYaw;
int lectura_sensor;
float voltaje_sensor;
float corriente_sensor;

void setup() {
  Serial.begin(115200);

  // Inicialización del MCP2515
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ); // Configurar la velocidad del bus CAN (1 Mbps) y cristal de 8 MHz
  mcp2515.setNormalMode(); // Modo normal de operación
  Serial.println("MCP2515 Initialized");
}

void loop() {
  struct can_frame canMsg;

  // Verificar si hay algún mensaje CAN disponible
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) { // Si se recibe un mensaje CAN
    switch (canMsg.can_id) {
      // Recibir datos de Roll y Pitch (ID 0x100)
      case 0x100:
        memcpy(&Roll, &canMsg.data[0], sizeof(float));
        memcpy(&Pitch, &canMsg.data[4], sizeof(float));
        Serial.print("Roll: ");
        Serial.print(Roll);
        Serial.print(" Pitch: ");
        Serial.println(Pitch);
        break;

      // Recibir dato de Yaw (ID 0x101)
      case 0x101:
        memcpy(&Yaw, &canMsg.data[0], sizeof(float));
        Serial.print("Yaw: ");
        Serial.println(Yaw);
        break;

      // Recibir datos de RateRoll y RatePitch (ID 0x102)
      case 0x102:
        memcpy(&RateRoll, &canMsg.data[0], sizeof(float));
        memcpy(&RatePitch, &canMsg.data[4], sizeof(float));
        Serial.print("RateRoll: ");
        Serial.print(RateRoll);
        Serial.print(" RatePitch: ");
        Serial.println(RatePitch);
        break;

      // Recibir dato de RateYaw (ID 0x103)
      case 0x103:
        memcpy(&RateYaw, &canMsg.data[0], sizeof(float));
        Serial.print("RateYaw: ");
        Serial.println(RateYaw);
        break;

      // Recibir valor del sensor MQ-4 (ID 0x104)
      case 0x104:
        char textBuffer[9]; // Tamaño de la cadena de texto
        memcpy(textBuffer, canMsg.data, 8); // Copiar los datos recibidos a la cadena
        textBuffer[8] = '\0'; // Asegurarse de que la cadena esté terminada
        lectura_sensor = atoi(textBuffer); // Convertir la cadena a entero
        Serial.print("Lectura sensor MQ-4: ");
        Serial.println(lectura_sensor);
        break;

      // Recibir valor del sensor de voltaje (ID 0x105)
      case 0x105:
        memcpy(&voltaje_sensor, &canMsg.data[0], sizeof(float)); // Copiar el valor recibido en voltaje_sensor
        Serial.print("Lectura sensor voltaje: ");
        Serial.println(voltaje_sensor);
        break;

      case 0x106:
        memcpy(&corriente_sensor, &canMsg.data[0], sizeof(float)); // Copiar el valor recibido en corriente_sensor
        Serial.print("Lectura sensor corriente: ");
        Serial.println(corriente_sensor);
        break;

      default:
        Serial.println("Mensaje CAN no reconocido");
        break;
    }
  }
}