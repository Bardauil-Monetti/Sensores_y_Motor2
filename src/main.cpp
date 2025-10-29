#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_BMP280.h> // BMP280
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h> // DS18B20
#include <MPU6050_light.h>
#include <math.h>
#include <ESP32Servo.h>

const char* red = "CESJT";
const char* contra = "itisjtsmg";
const int Fcarre[2] = {25, 26};
const int ESC[2] = {34, 10};
int ESCval[2] = {0, 0};
Servo esc[2];
int escValue[2];

#define SEALEVELPRESSURE_HPA (1024) // Presión estándar

// BMP280
Adafruit_BMP280 bmp; // I2C

// MPU6050
MPU6050 mpu(Wire);
const float gravedad = 9.80665;
float inclinacionX = 0.0;
float inclinacionY = 0.0;
float inclinacionZ = 0.0;
float aceleracionX = 0.0;
float aceleracionY = 0.0;
float aceleracionZ = 0.0;
float aceleracionTotal = 0.0;

// DS18B20
const int pinDatosDQ = 14;
float tempAgua = 0.0;
OneWire oneWireObjeto(pinDatosDQ);
DallasTemperature sensorDS18B20(&oneWireObjeto);

// Configuro sv asincrono
AsyncWebServer server(80);


const int pinesBomba[2] = {26, 27};
int t_Previo;

// No cambia la página
const char pagina_template[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Nautilus VI</title>
  <style>
    body {
      margin: 0;
      font-family: Arial, sans-serif;
      display: flex;
      height: 100vh;
      background: #0d0d0d;
      color: #e0e0e0;
    }

    .panel {
      flex: 1;
      padding: 20px;
      box-sizing: border-box;
      border: 2px solid #111;
    }

    #controles {
      background: #141414;
      max-width: 20%;
      text-align: center;
    }

    #camara {
      background: #1b1b1b;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      flex: 1.2;
    }

    #info {
      background: #141414;
      max-width: 20%;
      overflow-y: auto;
    }

    h2 {
      text-align: center;
      color: #00aaff;
      text-shadow: 0px 0px 8px #005577;
    }

    img {
      max-width: 90%;
      max-height: 80%;
      border: 3px solid #ff3333;
      border-radius: 12px;
      box-shadow: 0 0 20px #ff000050;
    }

    .control-btn {
      background: linear-gradient(145deg, #222, #111);
      border: 2px solid #ff3333;
      color: #e0e0e0;
      font-size: 22px;
      font-weight: bold;
      border-radius: 8px;
      cursor: pointer;
      transition: 0.2s;
      box-shadow: 0 0 10px #000;
      width: 80px;
      height: 80px;
    }

    .control-btn:hover {
      background: #222;
      color: #00aaff;
      border-color: #00aaff;
      box-shadow: 0 0 15px #00aaff88;
    }

    .joystick {
      display: grid;
      grid-template-columns: 80px 80px 80px;
      grid-template-rows: 80px 80px 80px;
      gap: 10px;
      justify-content: center;
      margin-top: 20px;
    }

    .up { grid-column: 2; grid-row: 1; }
    .left { grid-column: 1; grid-row: 2; }
    .center { grid-column: 2; grid-row: 2; } /* Botón Parar */
    .right { grid-column: 3; grid-row: 2; }
    .subir { grid-column: 1; grid-row: 3; }
    .bajar { grid-column: 3; grid-row: 3; }

    .sensor {
      margin: 10px 0;
      text-align: left;
      font-size: 14px;
    }

    .sensor span {
      color: #ffff66;
      font-weight: bold;
    }
  </style>

  <script>
    const ESP32_IP = window.location.hostname;
    function sendCommand(cmd) {
      fetch(`http://${ESP32_IP}/${cmd}`)
        .then(response => response.text())
        .then(text => console.log("Respuesta:", text))
        .catch(error => console.error("Error:", error));

    }

    // Función para actualizar sensores cada 1s
    function actualizarSensores() {
      fetch(`http://${ESP32_IP}/sensores`)
        .then(res => res.json())
        .then(data => {
          document.getElementById("tempBMP").innerText = data.tempBMP + " °C";
          document.getElementById("presionBMP").innerText = data.presionBMP + " hPa";
          document.getElementById("altitudBMP").innerText = data.altitudBMP + " m";
          document.getElementById("tempAgua").innerText = data.tempAgua + " °C";
          document.getElementById("incX").innerText = data.incX;
          document.getElementById("incY").innerText = data.incY;
          document.getElementById("incZ").innerText = data.incZ;
          document.getElementById("accTotal").innerText = data.accTotal;
        });
    }

    setInterval(actualizarSensores, 1000);
  </script>
</head>
<body>
  <!-- Panel de controles -->
  <div id="controles" class="panel">
    <h2>Controles</h2>
    <div class="joystick">
      <button class="control-btn up" onclick="sendCommand('adelante')">⬆️</button>
      <button class="control-btn left" onclick="sendCommand('izquierda')">⬅️</button>
      <button class="control-btn center" onclick="sendCommand('parar')">⏹️</button>
      <button class="control-btn right" onclick="sendCommand('derecha')">➡️</button>
      <button class="control-btn subir" onclick="sendCommand('subir')">⏫</button>
      <button class="control-btn bajar" onclick="sendCommand('bajar')">⏬</button>
    </div>
  </div>

  <!-- Panel de cámara -->
  <div id="camara" class="panel">
    <h2>Cámara</h2>
    <img src="http://ipdelacam" alt="Transmisión ESP32-CAM">
  </div>

  <!-- Panel de información y sensores -->
  <div id="info" class="panel">
    <h2>Info</h2>
    <p><b>Proyecto:</b> Nautilus VI</p>
    <p><b>Equipo:</b> Gonzalo Bardauil, Benicio Mario Ortiz, Nicolás Monetti y Brenda Tamara Gamboa Arellano Barreira</p>
    <p><b>Profesor:</b> Franco Zapata</p>
    <p><b>Colegio:</b> Instituto Técnico San Judas Tadeo</p>

    <h2>Lecturas Sensores</h2>
    <div class="sensor"><b>Temperatura interna (BMP280):</b> <span id="tempBMP">-- °C</span></div>
    <div class="sensor"><b>Presión (BMP280):</b> <span id="presionBMP">-- hPa</span></div>
    <div class="sensor"><b>Altitud (BMP280):</b> <span id="altitudBMP">-- m</span></div>
    <div class="sensor"><b>Temperatura agua (DS18B20):</b> <span id="tempAgua">-- °C</span></div>
    <div class="sensor"><b>Inclinación X (MPU6050):</b> <span id="incX">--</span></div>
    <div class="sensor"><b>Inclinación Y (MPU6050):</b> <span id="incY">--</span></div>
    <div class="sensor"><b>Inclinación Z (MPU6050):</b> <span id="incZ">--</span></div>
    <div class="sensor"><b>Aceleración total (MPU6050):</b> <span id="accTotal">--</span></div>

  </div>
</body>
</html>

)rawliteral";


// Funciones dedicadas para cada ruta (/...)

// Función para manejar la petición a la página principal (/)
void handleRootRequest(AsyncWebServerRequest *request) {
    String pagina = pagina_template; //hago una variable para no modificar a la original
    ESCval[0] = map(2000, 0, 4095, 1000, 2000);
    if(digitalRead(Fcarre[0]) || digitalRead(Fcarre[1])){
      request->redirect("/parar");
    }
    request->send(200, "text/html", pagina);
}

// Función para manejar la petición de toggle (/toggle)
void subir(AsyncWebServerRequest *request) {
    Serial.println("Subiendo");
    digitalWrite(pinesBomba[0], HIGH);
    digitalWrite(pinesBomba[1], LOW);
    request->send(200, "text/plain", "OK");
  }
  
void bajar(AsyncWebServerRequest *request) {
    Serial.println("Bajando");
    digitalWrite(pinesBomba[1], HIGH);
    digitalWrite(pinesBomba[0], LOW);
    request->send(200, "text/plain", "OK");
  }
  
void parar(AsyncWebServerRequest *request) {
    Serial.println("Alto");
    digitalWrite(pinesBomba[1], LOW);
    digitalWrite(pinesBomba[0], LOW);
    for(int i=0; i<2; i++){
      escValue[i] = map(0, 0, 4095, 1000, 2000);
      esc[i].writeMicroseconds(escValue[i]);
    }
    request->send(200, "text/plain", "OK");
}

void avanzar(AsyncWebServerRequest *request) {
    Serial.println("Adelante");
    for(int i=0; i<2; i++){
      escValue[i] = map(2000, 0, 4095, 1000, 2000);
      esc[i].writeMicroseconds(escValue[i]);
    }
    request->send(200, "text/plain", "OK");
}

void derecha(AsyncWebServerRequest *request) {
    Serial.println("derecha");
    escValue[0] = map(0, 0, 4095, 1000, 2000);
    escValue[1] = map(2000, 0, 4095, 1000, 2000);
    esc[0].writeMicroseconds(escValue[0]);
    esc[1].writeMicroseconds(escValue[1]);
    request->send(200, "text/plain", "OK");
}

void izquierda(AsyncWebServerRequest *request) {
    Serial.println("izquierda");
    escValue[1] = map(0, 0, 4095, 1000, 2000);
    escValue[0] = map(2000, 0, 4095, 1000, 2000);
    esc[1].writeMicroseconds(escValue[1]);
    esc[0].writeMicroseconds(escValue[0]);
    request->send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Bienvendo abordo, capitan");
  for (int i = 0; i < 2; i++) {
  pinMode(pinesBomba[i], OUTPUT);
  pinMode(Fcarre[i], INPUT);
  digitalWrite(pinesBomba[i], LOW);
  esc[i].attach(ESC[i], 1000, 2000);
}

  sensorDS18B20.begin();

  Wire.begin(21, 22); // Pines I2C comunes para sensores MPU6050 y BMP280
  
  //Inicializacion BMP280
  Serial.println("Iniciando BMP280...");
  if (!bmp.begin(0x76)) {
    Serial.println("No se encontro el BMP280. Verifica el cableado.");
    while (1);
  }

  //Inicializacion MPU6050
  Serial.println("Inicializando MPU6050...");
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print("Error de conexión: ");
    Serial.println(status);
    while (1);
  }
  Serial.println("Calibrando...");
  mpu.calcOffsets();
  Serial.println("Listo!");

  Serial.println("Conectando al WiFi...");
  WiFi.begin(red, contra);
  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++intentos > 40) {
      Serial.println("\n No se pudo conectar al WiFi.");
      break;
    }
  }

  // Conexion wifi como antes
 int timeout = 20; 
  while (WiFi.status() != WL_CONNECTED && timeout > 0) {
    delay(500);
    Serial.print(".");
    timeout--;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFallo la conexion. Reiniciando...");
    delay(1000);
    ESP.restart();
  }
  Serial.println("\nWiFi conectado!");
  Serial.print("Dirección IP: http://");
  Serial.println(WiFi.localIP());

  // defino las rutas para las request 
  // Ahora ponemos el nombre de nuestra función dedicada
  server.on("/", HTTP_GET, handleRootRequest);
  server.on("/subir", HTTP_GET, subir);
  server.on("/bajar", HTTP_GET, bajar);
  server.on("/parar", HTTP_GET, parar);
  server.on("/adelante", HTTP_GET, avanzar);
  server.on("/derecha", HTTP_GET, derecha);
  server.on("/izquierda", HTTP_GET, izquierda);
  server.on("/sensores", HTTP_GET, [](AsyncWebServerRequest *request){
    // Lectura rápida para asegurar valores recientes
    mpu.update();
    float tempBMP = bmp.readTemperature();
    float presBMP = bmp.readPressure()/100.0;
    float altBMP = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    sensorDS18B20.requestTemperatures();
    float tAgua = sensorDS18B20.getTempCByIndex(0);

    String json = "{";
    json += "\"tempBMP\":" + String(tempBMP) + ",";
    json += "\"presionBMP\":" + String(presBMP) + ",";
    json += "\"altitudBMP\":" + String(altBMP) + ",";
    json += "\"tempAgua\":" + String(tAgua) + ",";
    json += "\"incX\":" + String(mpu.getAccX()) + ",";
    json += "\"incY\":" + String(mpu.getAccY()) + ",";
    json += "\"incZ\":" + String(mpu.getAccZ()) + ",";
    json += "\"accTotal\":" + String(sqrt(
        pow(mpu.getAccX()*gravedad,2) + pow(mpu.getAccY()*gravedad,2) + pow(mpu.getAccZ()*gravedad,2)
    ));
    json += "}";
    request->send(200, "application/json", json);
});

  // Iniciar servidor
  server.begin();
}

void loop() {
  // El loop sigue libre para las tareas de hardware
  if (millis() - t_Previo > 1000) { // cada 1 segundo
    mpu.update();
    inclinacionX = mpu.getAccX();
    inclinacionY = mpu.getAccY();
    inclinacionZ = mpu.getAccZ();

    aceleracionX = inclinacionX * gravedad;
    aceleracionY = inclinacionY * gravedad;
    aceleracionZ = inclinacionZ * gravedad;

    aceleracionTotal = sqrt(aceleracionX*aceleracionX + aceleracionY*aceleracionY + aceleracionZ*aceleracionZ);
    sensorDS18B20.requestTemperatures();
    tempAgua = sensorDS18B20.getTempCByIndex(0);

        
    t_Previo = millis();
    }
}
