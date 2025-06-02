#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// === CONFIGURACIÓN UART Y LED ===
#define RXD2 16  // UART2 RX desde la PIC
#define TXD2 17  // UART2 TX hacia la PIC
#define LED_MOTOR 2

HardwareSerial mySerial(2);  // UART2: comunicación con la PIC

bool ledState = false;
bool setVelocidad = false;

// === ESTRUCTURAS DE DATOS ===
typedef struct struct_message {
  String cmd;
} struct_message;

typedef struct struct_telemetry {
  float rps;
  float corriente;
  float temperatura;
  float duty;
} struct_telemetry;

struct_message incomingData;
struct_telemetry datosParaMaster;

// === MAC DEL MAESTRO (F4:65:0B:46:84:F0) ===
uint8_t masterAddress[] = {0xF4, 0x65, 0x0B, 0x46, 0x84, 0xF0};

// === VARIABLES DE ESTADO ===
String lastCmd = "X";
String uartInputBuffer = "";

// === PROTOTIPOS DE FUNCIONES ===
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingDataRaw, int len);
void recibirCadenaDesdePIC();
void procesarCadenaPIC(const String &data);
void enviarDatosAMaestro(float rps, float corriente, float temperatura, float duty);

// === SETUP ===
void setup() {
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pinMode(LED_MOTOR, OUTPUT);
  digitalWrite(LED_MOTOR, LOW);

  // Inicializar ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error al iniciar ESP-NOW");
    return;
  }

  // Registrar peer (maestro)
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, masterAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (!esp_now_is_peer_exist(masterAddress)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Error al registrar el peer maestro.");
    }
  }

  Serial.println("ESP32 esclava lista para recibir comandos del maestro y reenviarlos a la PIC.");
}

// === LOOP ===
void loop() {
  recibirCadenaDesdePIC();  // Prioridad baja
}

// === CALLBACK: recepción ESP-NOW ===
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
  String cmd = incomingData.cmd;

  Serial.print("Comando recibido por ESP-NOW: ");
  Serial.println(cmd);

  if (!cmd.equals(lastCmd)) {
    lastCmd = cmd;

    Serial.print("→ Enviando a la PIC por UART2: ");
    Serial.println(cmd);

    ledState = !ledState;
    digitalWrite(LED_MOTOR, ledState);
    

    mySerial.println(cmd);
    mySerial.flush();
  }
}

// === LECTURA UART DESDE PIC ===
void recibirCadenaDesdePIC() {
  if (mySerial.available()) {
    char c = mySerial.read();
    if (c == '\n') {
      procesarCadenaPIC(uartInputBuffer);
      uartInputBuffer = "";
    } else {
      uartInputBuffer += c;
    }
  }
}

// === PROCESAMIENTO DE CADENA Y ENVÍO AL MAESTRO ===
void procesarCadenaPIC(const String &data) {
  int valores[4] = {0};
  int index = 0, start = 0;

  for (int i = 0; i <= data.length(); i++) {
    if (data[i] == ',' || i == data.length()) {
      valores[index++] = strtol(data.substring(start, i).c_str(), NULL, 16);
      start = i + 1;
    }
  }

  if (index == 4) {
    float rps = valores[0];
    float adc_corriente = valores[1];
    float adc_temp = valores[2];
    float duty = valores[3];

    float iout = 11.0 - 0.005372405373 * adc_corriente;
    float temperatura = 426.2447783 - 0.1350923175 * adc_temp;

    Serial.println("=== Datos recibidos de la PIC ===");
    Serial.print("RPS: "); Serial.println(rps);
    Serial.print("IOUT [A]: "); Serial.println(iout, 3);
    Serial.print("Temperatura [°C]: "); Serial.println(temperatura, 2);
    Serial.print("Duty [%]: "); Serial.println(duty);
    Serial.println("=================================");

    if(!setVelocidad){
      mySerial.println("V0AA");
      setVelocidad = true;
    } 
    // Enviar por ESP-NOW al maestro
    enviarDatosAMaestro(rps, iout, temperatura, duty);
  }
}

// === ENVÍO AL MAESTRO POR ESP-NOW ===
void enviarDatosAMaestro(float rps, float corriente, float temperatura, float duty) {
  datosParaMaster.rps = rps;
  datosParaMaster.corriente = corriente;
  datosParaMaster.temperatura = temperatura;
  datosParaMaster.duty = duty;

  esp_err_t result = esp_now_send(masterAddress, (uint8_t *) &datosParaMaster, sizeof(datosParaMaster));

  if (result == ESP_OK) {
    Serial.println("✔️  Datos enviados al maestro por ESP-NOW.");
  } else {
    Serial.print("❌ Error al enviar datos al maestro: ");
    Serial.println(result);
  }
}