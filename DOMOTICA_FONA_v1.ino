
/* 
Domotica v1
Fecha: 2015-04-12
Autor: Alejandro Leon
Email: alejandroleon215@gmail.com

Descripcion: 

Obtiene valores de temperatura y humedad desde el sensor DHT11 y los envia a Ubidots para registro y graficacion.
La tira de LEDS se enciende segun el modo en el que se ejecute la aplicacion. Los modos se cambian mediante un mando de infrarojos.

Modo 0: No se enciende la tira de LEDS
Modo 1: Muestra colores en base a la temperatura
Modo 2: Cambia colores en base a sonidos captados por el sensor de sonido
Modo 3: Cambia los colores en base al sensor de distancia

Sensores: 

1: Lector de infrarojos
2: Sensor de temperatura y humedad DHT11
3: Sensor de sonido
4: Sensor de distancia

*/

// Includes

#include <IRremote.h>
#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>
#include "DHT.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

extern String token;

// Definicion de pines digitales

#define IR_PIN 2     // Sensor de infrarojos
#define DHTPIN 6     // Sensor DHT11 de temperatura y humedad
#define BLUEPIN 9    // PWM
#define REDPIN 10    // PWM
#define GREENPIN 11  // PWM

/* FONA CONFIG */
#define FONA_PS 3    // Fona
#define FONA_RX 13    // Fona
#define FONA_TX 7    // Fona
#define FONA_RST 8   // Fona
#define FONA_KEY 12  // Fona


// Definicion de pines analogicos

#define MIC_PIN A1   // Microfono

// Variables y constantes

#define READS 3 // Numero de lecturas del sensor de sonido para generar un promedio

int LEDSMode = 0; // 0=No luz, 1=Temp, 2=Sonido, 3=Distancia
int currentRead = 0;
int values[READS];

// Temporizador para LEDS
unsigned long timer_new_leds;
unsigned long timer_old_leds;
int tiempo_maximo_leds = 3; // Tiempo maximo en segundos para apagar leds si no hay suficiente sonido

// Temporizador para leer temp y humedad y enviar a Ubidots
unsigned long timer_new_ubidots;
unsigned long timer_old_ubidots;
unsigned long tiempo_maximo_ubidots = 300; // Tiempo en segundos para leer sensores y enviar a ubidots

// Fona

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
Adafruit_FONA fona = Adafruit_FONA(&fonaSS, FONA_RST);

// DHT

#define DHTTYPE DHT11   // DHT 11 

// Initialize DHT sensor for normal 16mhz Arduino
DHT dht(DHTPIN, DHTTYPE);

// Pantalla LCD

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

// Sensor IR

IRrecv irrecv(IR_PIN);
decode_results IR_resultados;

// ------------ SETUP -------------

void setup() {

  Serial.begin(9600);
  
  pinMode(REDPIN, OUTPUT);
  pinMode(GREENPIN, OUTPUT);
  pinMode(BLUEPIN, OUTPUT);
  
  // LLenamos las variables de temporizador
  timer_new_leds = millis();
  timer_old_leds = millis();
  timer_new_ubidots = millis();
  timer_old_ubidots = millis();
  
  // Arrancamos el sensor IR
  irrecv.enableIRIn();
  
  // Mostramos por defecto el modo 0
  Serial.print(F("LEDS mode: "));
  Serial.println(LEDSMode);
  
  // LCD setup
  lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight 
  
  // ------- Quick 3 blinks of backlight  -------------
  for(int i = 0; i< 3; i++)
  {
    lcd.backlight();
    delay(250);
    lcd.noBacklight();
    delay(250);
  }
  lcd.backlight(); // finish with backlight on 
  
  lcd.clear();
  lcd.setCursor(0,0); //Start at character 1 on line 0
  lcd.print(F("DOMOTICA v1.0"));
  lcd.setCursor(0,1);
  lcd.print(F("INICIANDO"));
  
  // Fona setup
  
  pinMode(FONA_KEY,OUTPUT); //Pin para encender y apagar Fona
  pinMode(FONA_PS,INPUT); //Pin para leer si fona esta encendido (HIGH) o apagado (LOW)
  
  digitalWrite(FONA_KEY,HIGH);
  
  TurnOnFona(); // Encendemos fona
  
  Serial.println(F("FONA basic test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));   // See if the FONA is responding
  if (! fona.begin(4800)) {                                    // make it slow so its easy to read!
    Serial.println(F("Couldn't find FONA"));
  } else {
  Serial.println(F("FONA is OK"));
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("FONA OK"));
  lcd.setCursor(0,1);
  lcd.print(F("LEYENDO DATOS"));
  delay(2000);
  }
  
  // Leemos los primeros datos para mostrar en el LCD
  
  // Leer temperatura
  float temp = dht.readTemperature();
  
  // Leer humedad
  float hum = dht.readHumidity();
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(temp) || isnan(hum)) {
          Serial.println(F("Failed to read from DHT sensor!"));
          return;
        }
  
    PrintSeconds();
    Serial.print(F("Temp: "));
    Serial.println(temp);
  
    PrintSeconds();
    Serial.print(F("Humedad: "));
    Serial.println(hum);
  
    // Encendemos fona
    TurnOnFona();
    delay(3000);
  
    // Esperar por conexion
    GetConnected();
  
    // Leemos el % de bateria LI-PO del Fona
    uint16_t vbat;
    if (! fona.getBattPercent(&vbat)) {
      PrintSeconds();
      Serial.println(F("Failed to read Batt"));
    } else {
      PrintSeconds();
      Serial.print(F("Li-Po battery VPct = ")); Serial.print(vbat); Serial.println(F("%"));
    }
  
    // Print to LCD screen
  
    lcd.clear();
    lcd.setCursor(0,0); //Start at character 1 on line 0
    lcd.print(F("T:")); lcd.print(temp,0); lcd.print((char)223); lcd.print(F("C "));
    lcd.print(F("H:")); lcd.print(hum,0); lcd.print(F("%"));
    lcd.setCursor(0,1);
    lcd.print(F("B:")); lcd.print(vbat); lcd.print(F("%")); lcd.print(F(" "));
    printMode(LEDSMode);
    
    TurnOffFona();
}

// ------------ LOOP -------------

void loop() {
  
  // Leer infrarojos y establecer el modo si se recibe el valor 77E1C062
  leerIR();
  
  // En todos los modos se leen los sensores y se envian los datos a Ubidots
  leerSensores();
  
  switch(LEDSMode) {
    case 1:
      //LEDS muestran temperatura
      encenderLedsTemp();
    case 2:
      //LEDS con sonido
      encenderLedsSonido();
      break;
  }
}


// ------------ FUNCIONES -------------

void setLedColor(int n) {
  
  int randNumberR;
  int randNumberB;
  int randNumberG;
  randNumberR = random(255);
  randNumberB = random(255);
  randNumberG = random(255);
  
  analogWrite(REDPIN, randNumberR);
  analogWrite(BLUEPIN, randNumberB);
  analogWrite(GREENPIN, randNumberG);
 }
 
void TurnOnFona()
{
  
  uint8_t answer=0;
  
  PrintSeconds();
  Serial.print(F("Turning on Fona"));
  // Si fona esta apagado, lo encendemos
  if (digitalRead(FONA_PS) == 0) {
    Serial.print(F(" - Power status before key: "));
    Serial.print(digitalRead(FONA_PS));
    digitalWrite(FONA_KEY,LOW);
    delay(2000);
    digitalWrite(FONA_KEY,HIGH);
    delay(2000);
    Serial.print(F(" - Power status after key: "));
    Serial.println(digitalRead(FONA_PS));
  }
  else {
    Serial.println(F(" - Fona ya estaba encendido"));
  }
  if (! fona.begin(4800)) {                                    // make it slow so its easy to read!
    Serial.println(F(" - Couldn't find FONA"));
  }
  
  // waits for an answer from the module
  while(answer == 0) {     // Send AT every two seconds and wait for the answer
    answer = sendATcommand("AT", "OK", 2000);    
   }
}

void TurnOffFona()
{
  PrintSeconds();
  Serial.print(F("Turning off Fona"));
  // Si fona esta encendido, lo apagamos
  if (digitalRead(FONA_PS == 1)) {
    Serial.print(F(" - Power status before key: "));
    Serial.print(digitalRead(FONA_PS));
    digitalWrite(FONA_KEY,LOW);
    delay(2000);
    digitalWrite(FONA_KEY,HIGH);
    delay(2000);
    Serial.print(F(" - Power status after key: "));
    Serial.println(digitalRead(FONA_PS));
  }
  else {
    Serial.println(F(" - Fona ya estaba apagado"));
  }
}

void PrintSeconds()
{
  float tiempo = millis()/60000.00;
  Serial.print(tiempo,2);
  Serial.print(F(" mins: "));
}

int8_t sendATcommand(char* ATcommand, char* expected_answer, unsigned int timeout){

    uint8_t x=0,  answer=0;
    char response[100];
    unsigned long previous;

    memset(response, '\0', 100);    // Initialize the string
    
    delay(100);
    
    while( fonaSS.available() > 0) fonaSS.read();    // Clean the input buffer
    
    fonaSS.println(ATcommand);    // Send the AT command 


    x = 0;
    previous = millis();

    // this loop waits for the answer
    do{
        if(fonaSS.available() != 0){    
            // if there are data in the UART input buffer, reads it and checks for the asnwer
            response[x] = fonaSS.read();
            x++;
            // check if the desired answer  is in the response of the module
            if (strstr(response, expected_answer) != NULL)    
            {
                answer = 1;
            }
        }
         // Waits for the asnwer with time out
    }while((answer == 0) && ((millis() - previous) < timeout)); 
  
  PrintSeconds();
  Serial.print(F(" sendATcommand request: "));
  Serial.println(ATcommand);
  
  PrintSeconds();
  Serial.print(F(" sendATcommand response: "));  
  Serial.println(response);

  return answer;
}



void Send2ubidots(String value, String variableID, String tokenID)
{

  int num;
  String le;
  String var;
  String variableIDStr;
  String tokenIDStr;
  
  var = "{\"value\":"+ value + "}";           //value is the sensor value
  num = var.length();
  le = String(num);                           //this is to calcule the length of var
  variableIDStr = "POST /api/v1.6/variables/" + variableID + "/values HTTP/1.1";
  tokenIDStr = "X-Auth-Token: " + tokenID;
  
  //Serial.println(tokenIDStr); 
  //Serial.print(F("TokenID: "));
  //Serial.println(tokenID);

  fonaSS.println(F("AT+CGATT?"));
  PrintSeconds();
  Serial.println(F("AT+CGATT?"));
  delay(100);
  PrintSeconds();
  ShowSerialData();

  PrintSeconds();
  Serial.println(F("Start the connection to Ubidots"));
  
  fonaSS.println(F("AT+CIPSTART=\"TCP\",\"things.ubidots.com\",\"80\""));
  PrintSeconds();
  Serial.println(F("AT+CIPSTART=\"TCP\",\"things.ubidots.com\",\"80\""));
  delay(4000);
  PrintSeconds();
  ShowSerialData();
  
  PrintSeconds();
  Serial.println(F("Begin to send data to the remote server"));

  PrintSeconds();
  fonaSS.println(F("AT+CIPSEND"));
  Serial.println(F("AT+CIPSEND"));
  delay(4000);
  PrintSeconds();
  ShowSerialData();
  
  PrintSeconds();
  Serial.println(F("Sending request to Ubidots"));
  
  fonaSS.println(variableIDStr);           // Replace "xxx..." with your variable ID
  Serial.println(variableIDStr);
  delay(400);
  
  fonaSS.println(F("Content-Type: application/json"));
  Serial.println(F("Content-Type: application/json"));
  delay(400);
  
  fonaSS.println("Content-Length: "+le);
  Serial.println("Content-Length: "+le);
  delay(400);
  
  fonaSS.println(tokenIDStr);               // here you should replace "xxx..." with your Ubidots Token
  Serial.println(tokenIDStr);
  delay(400);
  
  fonaSS.println(F("Host: things.ubidots.com"));
  Serial.println(F("Host: things.ubidots.com"));
  delay(400);
  
  fonaSS.println();
  delay(400);
  
  fonaSS.println(var);
  Serial.println(var);
  delay(400);
  
  fonaSS.println();
  delay(400);
  
  fonaSS.println((char)26);   // CONTROL+Z
  Serial.println((char)26);
  delay(4000);
  
  Serial.println("");
  PrintSeconds();
  ShowSerialData();

  PrintSeconds();
  Serial.println(F("Close connection to Ubidots: "));              // Close the connection
  sendATcommand("AT+CIPCLOSE","OK",2000);
}


void GetConnected()
{
  uint8_t n=0;
  do
  {
    n = fona.getNetworkStatus();  // Read the Network / Cellular Status
    Serial.print(F("Network status "));
    Serial.print(n);
    Serial.print(F(": "));
      if (n == 0) Serial.println(F("Not registered"));
      if (n == 1) Serial.println(F("Registered (home)"));
      if (n == 2) Serial.println(F("Not registered (searching)"));
      if (n == 3) Serial.println(F("Denied"));
      if (n == 4) Serial.println(F("Unknown"));
      if (n == 5) Serial.println(F("Registered roaming"));
  } while (n != 1);
  delay(3000);
}

void ShowSerialData()
{
  Serial.println(F("Serial response"));
  while(fonaSS.available()!=0)
    Serial.write(fonaSS.read());
}

void leerIR() {
  
  //PrintSeconds();
  //Serial.println(F("Leyendo IR"));
  if (irrecv.decode(&IR_resultados)) {

    irrecv.resume(); // Recibir el siguiente valor
    Serial.print(F("IR valor: "));
    Serial.println(IR_resultados.value);
    
    // Boton "MENU" del apple remote, valores 2011283554 y 4294967295
    // Boton "CH-" del mando de Jorge
    if (IR_resultados.value == 'FFA25D') {
      switch(LEDSMode) {
        case 0:
          LEDSMode = 1;
          break;
        case 1:
          LEDSMode = 2;
          break;
        case 2: 
          LEDSMode = 3;
          break;
        case 3:
          LEDSMode = 0;
          break;
      }
     PrintSeconds();
     Serial.print(F("LEDS mode: "));
     Serial.println(LEDSMode);
     printMode(LEDSMode);
    }
  }
}

void encenderLedsSonido() {
  
   if (currentRead == READS) {
    int promedio = 0;
    for (int i = 0; i < READS;i++) {
      promedio = promedio + values[i];
    }
    
    promedio = promedio / READS;
    
    if (promedio > 360) {
      
      timer_old_leds = millis(); //Reseteamos el temporizador
      
      Serial.print("Promedio: ");
      Serial.println(promedio);
      setLedColor(promedio);

    } else {
    
      timer_new_leds = millis();  // Apagamos los leds si se supera el tiempo maximo sin sonido
      if ((timer_new_leds - timer_old_leds)>(tiempo_maximo_leds*1000)) {
        analogWrite(REDPIN, 0);
        analogWrite(BLUEPIN, 0);
        analogWrite(GREENPIN, 0);
        timer_old_leds = millis();
      }
    }
    currentRead = 0;
  }
  
  values[currentRead] = analogRead(MIC_PIN);                        // Raw reading from mic 
  Serial.print(F("Read "));
  Serial.print(currentRead);
  Serial.print(F(":"));
  Serial.println(values[currentRead]);
  currentRead++;

}

void encenderLedsTemp() {

  // Leer temperatura
  float temp = dht.readTemperature();
  if (isnan(temp)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  
  if (temp < 16) {
    analogWrite(REDPIN, 0);
    analogWrite(BLUEPIN, 255);
    analogWrite(GREENPIN, 0);
  } else if (temp >= 16 | temp < 23) {
    analogWrite(REDPIN, 255);
    analogWrite(BLUEPIN, 0);
    analogWrite(GREENPIN, 255);
  } else if (temp >= 23) {
    analogWrite(REDPIN, 255);
    analogWrite(BLUEPIN, 0);
    analogWrite(GREENPIN, 0);
  }
}

void leerSensores() {

    //Leemos temporizador y hacemos lecturas de sensores si se cumple el tiempo
    timer_new_ubidots = millis();
    /*
    PrintSeconds();
    Serial.print(F("Ubidots max time: "));
    Serial.print(tiempo_maximo_ubidots*1000);
    Serial.print(F(" - Ubidots timer diff: "));
    Serial.println(timer_new_ubidots - timer_old_ubidots);
    */
    if ((timer_new_ubidots - timer_old_ubidots)>(tiempo_maximo_ubidots*1000)) {
      
        lcd.setCursor(6,1);
        lcd.print(F("- ENVIANDO"));
      
        // Leer temperatura
        float temp = dht.readTemperature();
  
        // Leer humedad
        float hum = dht.readHumidity();
  
        // Check if any reads failed and exit early (to try again).
        if (isnan(temp) || isnan(hum)) {
          Serial.println(F("Failed to read from DHT sensor!"));
          return;
        }
  
    PrintSeconds();
    Serial.print(F("Temp: "));
    Serial.println(temp);
  
    PrintSeconds();
    Serial.print(F("Humedad: "));
    Serial.println(hum);
  
    //PrintSeconds();
    //Serial.print(F("Luz: "));
    //Serial.println(luz);
  
    // Encendemos fona
    TurnOnFona();
    delay(3000);
  
    // Esperar por conexion
    GetConnected();
  
    // Leemos el % de bateria LI-PO del Fona
    uint16_t vbat;
    if (! fona.getBattPercent(&vbat)) {
      PrintSeconds();
      Serial.println(F("Failed to read Batt"));
    } else {
      PrintSeconds();
      Serial.print(F("Li-Po battery VPct = ")); Serial.print(vbat); Serial.println(F("%"));
    }
  
    // Ubidots authentification token
    //String token = "wDWuIUEHtpH1BFZJ6WETr4NlAiw5Vv";
  
    // Enviamos temperatura a Ubidots
    PrintSeconds();
    Serial.println(F("************ Enviando temperatura a Ubidots"));
    Send2ubidots(String(temp),String(F("54dcde5876254240b3af12a8")),String(token));
  
    // Enviamos % voltaje de bateria Li-Po
    PrintSeconds();
    Serial.println(F("************ Enviando % voltaje bateria LiPo a Ubidots"));
    Send2ubidots(String(vbat),String(F("54e70de57625422f2f2313da")),String(token));
  
    // Enviamos humedad a Ubidots
    PrintSeconds();
    Serial.println(F("************ Enviando humedad a Ubidots"));
    Send2ubidots(String(hum),String(F("54e72f8d76254260d428020e")),String(token));
  
    // Enviamos luz a Ubidots
    //PrintSeconds();
    //Serial.println(F("************ Enviando luz a Ubidots"));
    //Send2ubidots(String(luz),String(F("54e772ac7625423c39a651d0")),String(token));
  
    // Apagamos fona
    TurnOffFona();
    
    // Reestablecemos temporizador
    timer_old_ubidots = millis();
    
    // Print to LCD screen
  
    lcd.clear();
    lcd.setCursor(0,0); //Start at character 1 on line 0
    lcd.print(F("T:")); lcd.print(temp,0); lcd.print((char)223); lcd.print(F("C "));
    lcd.print(F("H:")); lcd.print(hum,0); lcd.print(F("%"));
    lcd.setCursor(0,1);
    lcd.print(F("B:")); lcd.print(vbat); lcd.print(F("%")); lcd.print(F(" "));
    printMode(LEDSMode);
    }
}

void printMode(int mode) {
  lcd.setCursor(13,0); //Caracter 13, linea 0
  lcd.print(F("M:"));
  lcd.print(mode);
}

