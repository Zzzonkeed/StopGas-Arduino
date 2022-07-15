#include <Arduino.h>
#include "AsyncTelegram.h"

#include <SPI.h>
#include <RH_NRF24.h>

//----------- WIFI MANAGER -------------
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ESP8266WiFi.h>

// Singleton instance of the radio driver
RH_NRF24 nrf24(2, 15);

AsyncTelegram myBot;
ReplyKeyboard myReplyKbd;   // reply keyboard object helper
InlineKeyboard myInlineKbd; // inline keyboard object helper

bool isKeyboardActive;      // store if the reply keyboard is shown

const char* token = "1766909127:AAG57w0lgshW4--NczJqEtGebSNWHMPBuI4";     // REPLACE myToken WITH YOUR TELEGRAM BOT TOKEN
const char* channel = "@Stopgas001";

#define LIGHT_ON_CALLBACK  "lightON"  // callback data sent when "LIGHT ON" button is pressed
#define LIGHT_OFF_CALLBACK "lightOFF" // callback data sent when "LIGHT OFF" button is pressed

#define LED_GREEN 4
#define LED_RED   5
#define LED_BLUE  0

#define NRF24_CHANNEL 1

//-------- MQ2-SENSOR -----------
#define   MQ_PIN                       (A0)
#define   RO_CLEAN_AIR_FACTOR   (9.83)
#define   RL_VALUE              (5)
#define   CALIBARAION_SAMPLE_TIMES     (50)
#define   CALIBRATION_SAMPLE_INTERVAL  (500)
#define   READ_SAMPLE_INTERVAL         (50)
#define   READ_SAMPLE_TIMES            (5)

#define   GAS_LPG                      (0)
#define   GAS_CO                       (1)
#define   GAS_SMOKE                    (2)

float     LPGCurve[3] = {2.3,0.21,-0.47};
float     COCurve[3] = {2.3,0.72,-0.34};
float     SmokeCurve[3] = {2.3,0.53,-0.44};
float     Ro = 10;

//-------------------------------
extern "C"
{
#include "user_interface.h"
}
WiFiManager wifiManager;
bool connection_wifi = false;
bool last_state_connection = false;

char DataToSend[12];
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
char DataRec[12];
bool sendNRF24 = false;

bool acceso = false;
bool succes = false;
uint8_t no_reply = 0;
bool noReply_state = false;
bool alarma_gas = false;

//------------- BUZZER ------------
const byte speakerPin=16;
unsigned long lastPeriodStart;
const int onDuration=1000;
const int periodDuration=6000;

//------------ SENSOR GAS ----------
float sensor_value;
int twoTime = 0;
int onceTime = 0;
int gas_concentration = 0;

//------------ ESP32 ---------------
#if defined(ESP32)
// WiFi event handler
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.print("\nWiFi connected! IP address: ");
      Serial.println(WiFi.localIP());

      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("\nWiFi lost connection");
      WiFi.setAutoReconnect(true);
      myBot.reset();
      break;
    default: break;
  }
}
#endif


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  // initialize the Serial
  Serial.begin(115200);
  noTone(speakerPin);

  wifiManager.autoConnect("STOPGAS-1.0");

  if(!wifiManager.autoConnect("STOPGAS-1.0")){
    Serial.println("fallo de conexión");  
    analogWrite(LED_GREEN, 0);
    analogWrite(LED_RED, 255);
    analogWrite(LED_BLUE, 0);   
  }

#if defined(ESP32)
  Serial.printf("setup() running on core  %d\n", xPortGetCoreID());
  WiFi.onEvent(WiFiEvent);
#endif
  Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
  Serial.print("\n\nStart connection to WiFi...");
  delay(100);

  if (!nrf24.init())
    Serial.println("init failed");
  if (!nrf24.setChannel(NRF24_CHANNEL))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");

  // To ensure certificate validation, WiFiClientSecure needs time updated
  // myBot.setInsecure(false);
  myBot.setClock("CET-1CEST,M3.5.0,M10.5.0/3");
  
  // Set the Telegram bot properies
  myBot.setUpdateTime(1000);
  myBot.setTelegramToken(token);

  // Check if all things are ok
  Serial.print("\nTest Telegram connection... ");
  myBot.begin() ? Serial.println("OK") : Serial.println("NOK");

  // Add reply keyboard
  isKeyboardActive = false;
  myReplyKbd.addButton("Cerrar válvula");
  myReplyKbd.addButton("Estado válvula");
  
  myReplyKbd.addRow();
  myReplyKbd.addButton("Restaurar válvula");
  myReplyKbd.addButton("Resetear Wifi");
  myReplyKbd.addRow();
  myReplyKbd.addButton("Cerrar Menú");
  myReplyKbd.enableResize();

  // Add sample inline keyboard
  myInlineKbd.addButton("ON", LIGHT_ON_CALLBACK, KeyboardButtonQuery);
  myInlineKbd.addButton("OFF", LIGHT_OFF_CALLBACK, KeyboardButtonQuery);

  analogWrite(LED_GREEN, 0);
  analogWrite(LED_RED, 0);
  analogWrite(LED_BLUE, 0);

  delay(1000);

  Serial.print("Calibrating...\n");                
  Ro = MQCalibration(MQ_PIN);                   
  Serial.print("Calibration is done...\n"); 
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");

  tone(speakerPin, 1000) ;
  delay (200);
  noTone(speakerPin);
  delay(100);
  tone(speakerPin, 1000) ;
  delay (200);
  noTone(speakerPin);
  delay(1000);

  String message ;    
  message += "Conectado a Wifi\n";
  message += "Módulo @";
  message += myBot.userName;
  myBot.sendToChannel(channel, message, true);
}

void loop() {
//------------ BUILTIN LED -------------
  static uint32_t ledTime = millis();
  if (millis() - ledTime > 700) {
    ledTime = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

//------------ TELEGRAM BOT ------------
  TBMessage msg;

  // if there is an incoming message...
  if (myBot.getNewMessage(msg)) {
    // check what kind of message I received
    String tgReply;
    MessageType msgType = msg.messageType;
    
    
    switch (msgType) {
      case MessageText :
        // received a text message
        tgReply = msg.text;
        Serial.print("\nText message received: ");
        Serial.println(tgReply);

        // check if is show keyboard command
        if (tgReply.equalsIgnoreCase("Menú") or tgReply.equalsIgnoreCase("Menu")) {
          // the user is asking to show the reply keyboard --> show it
          myBot.sendMessage(msg, "Menú StopGas", myReplyKbd);
          isKeyboardActive = true;
        }
        else if (tgReply.equalsIgnoreCase("/PASS")) {
          myBot.sendMessage(msg, "Control de acceso", myInlineKbd);
        }
        else if(tgReply.equalsIgnoreCase("Cerrar válvula")){
          if(acceso){
            String channel_msg;
            myBot.sendMessage(msg, "Cerrando válvula...", myReplyKbd);
            channel_msg += "Se ha cerrado la válvula manualmente";
            myBot.sendToChannel(channel, channel_msg, true);
            strcpy(DataToSend, "CLOSE-VALVE");
            sendNRF24 = true;
            acceso = false;
          }else{
            myBot.sendMessage(msg, "Acceso no concedido, activa el control de acceso", myReplyKbd);
          }
        }
        else if(tgReply.equalsIgnoreCase("Estado válvula")){
          myBot.sendMessage(msg, "Consultando estado de la válvula...", myReplyKbd);
          strcpy(DataToSend, "STATE-VALVE");
          sendNRF24 = true;
        }
        else if(tgReply.equalsIgnoreCase("Restaurar válvula")){
          if(acceso){
            myBot.sendMessage(msg, "Restaurando estado de la válvula", myReplyKbd);
            myBot.sendMessage(msg, "Para completar la restauracion de la válvula \nSe debe subir el vastago de la válvula de forma manual", myReplyKbd);
            strcpy(DataToSend, "VALVE-RESET");
            sendNRF24 = true;
            acceso = false;
          }else{
            myBot.sendMessage(msg, "Acceso no concedido, activa el control de acceso", myReplyKbd);
          }
        }
        else if(tgReply.equalsIgnoreCase("Resetear Wifi")){
          if(acceso){
            myBot.sendMessage(msg, "Reseteando sistema", myReplyKbd);
            wifiManager.resetSettings();
            delay(2000);
            ESP.restart();
            acceso = false;
          }else{
            myBot.sendMessage(msg, "Acceso no concedido, activa el control de acceso", myReplyKbd);
          }
        }
        // check if the reply keyboard is active
        else if (isKeyboardActive) {
          // is active -> manage the text messages sent by pressing the reply keyboard buttons
          if (tgReply.equalsIgnoreCase("Cerrar Menú")) {
            // sent the "hide keyboard" message --> hide the reply keyboard
            myBot.removeReplyKeyboard(msg, "Se cerró el Menú");
            isKeyboardActive = false;
          } else {
            // print every others messages received
            myBot.sendMessage(msg, "Opción no registrada, utilice las opciones del menú");
          }
        } 

        // the user write anything else and the reply keyboard is not active --> show a hint message
        else {
          myBot.sendMessage(msg, "Para interactuar con StopGas escriba: Menú");        
        }
        break;

      case MessageQuery:
        // received a callback query message
        tgReply = msg.callbackQueryData;
        Serial.print("\nCallback query message received: ");
        Serial.println(tgReply);
        
        if (tgReply.equalsIgnoreCase(LIGHT_ON_CALLBACK)) {
          // pushed "LIGHT ON" button...
          acceso = true;
          Serial.println("Control de acceso activado");
          analogWrite(LED_GREEN, 255);
          analogWrite(LED_RED, 0);
          analogWrite(LED_BLUE, 0);
          // terminate the callback with an alert message
          myBot.endQuery(msg, "Código de acceso activado", true);
        } 
        else if (tgReply.equalsIgnoreCase(LIGHT_OFF_CALLBACK)) {
          // pushed "LIGHT OFF" button...
          acceso = false;
          Serial.println("Control de acceso desactivado");
          analogWrite(LED_GREEN, 0);
          analogWrite(LED_RED, 0);
          analogWrite(LED_BLUE, 0);
          // terminate the callback with a popup message
          myBot.endQuery(msg, "Código de acceso desactivado", true);
        }
        
        break;

      case MessageLocation:
        // received a location message --> send a message with the location coordinates
        char bufL[50];
        snprintf(bufL, sizeof(bufL), "Longitude: %f\nLatitude: %f\n", msg.location.longitude, msg.location.latitude) ;
        myBot.sendMessage(msg, bufL);
        Serial.println(bufL);
        break;

      case MessageContact:
        char bufC[50];
        snprintf(bufC, sizeof(bufC), "Contact information received: %s - %s\n", msg.contact.firstName, msg.contact.phoneNumber ) ;
        // received a contact message --> send a message with the contact information
        myBot.sendMessage(msg, bufC);
        Serial.println(bufC);
        break;
        
      default:
        break;
    }
  }

//--------------------- GAS SENSOR ---------------------------
  static uint32_t ADCTime = millis();
  if(millis() - ADCTime > 1000){
    ADCTime = millis();
    sensor_value = analogRead(A0);
  }
  Serial.println(sensor_value);
  
  if(sensor_value > 450.0){
    alarma_gas = true;
    if(twoTime < 1){
      String message ;    
      message += "FUGA DE GAS DETECTADA!!! \n";
      message += "del modulo @";
      message += myBot.userName;
      message += ":\n";
      message += "Se realizó la siguiente acción:\n";
      message += "[CERRAR VÁLVULA]\n";

      myBot.sendToChannel(channel, message, true);
      twoTime++;
    }
    analogWrite(LED_GREEN, 0);
    analogWrite(LED_RED, 255);
    analogWrite(LED_BLUE, 0);
    //--------- CERRAR VALVULA -----------
    if(onceTime == 0){
      strcpy(DataToSend, "CLOSE-VALVE");
      sendNRF24 = true;
      onceTime = 1;
    }
   }
   
   if(sensor_value < 40){
    twoTime = 0;
    onceTime = 0;
   }

   if(alarma_gas){
    static uint32_t beep_time = millis();
    tone(speakerPin, 1000);
    if(millis() - beep_time > 3000){ //cambiar 3000 por 10000, 10 segundos
      beep_time = millis();
      noTone(speakerPin);
      alarma_gas = false;
    }
   }else{
      noTone(speakerPin);
   }

//--------------------- NRF24 ---------------------------
  while(sendNRF24){
    Serial.println("Sending to nrf24_server");
    // Send a message to nrf24_server
    nrf24.send((uint8_t*)DataToSend, sizeof(DataToSend));
    
    nrf24.waitPacketSent();
    uint8_t len = sizeof(buf);
  
    if (nrf24.waitAvailableTimeout(500)){   
      if (nrf24.recv(buf, &len)){
        Serial.print("got reply: ");
        Serial.println((char*)buf);
        succes = true;
        strcpy((char*)DataToSend, " ");
      }else{
        Serial.println("recv failed");
      }
    }else{
      Serial.println("No reply, is nrf24_server running?");
      no_reply++;

      if(no_reply > 5){
        Serial.println("No hay conexión con el modulo externo");
        sendNRF24 = false;
        Serial.println(no_reply);
        noReply_state = true;
        no_reply = 0;
      }
    }
    delay(400);
    if(noReply_state){
      myBot.sendMessage(msg, "No se logro conexión con el módulo exterior");
      myBot.sendMessage(msg, "Esto se debe a que el módulo exterior se encuentra apagado o falló la conexión");
      noReply_state = false;
      sendNRF24 = false;
    }
    if(succes){
      String response_bot;
      String valveState = "[ABIERTA]";
      //response_bot += "Comunicación exitosa con el módulo exterior \n";
      if(strcmp((char*)buf, "VALVE-CLOSE") == 0){
        response_bot += "Estado válvula: ";
        response_bot += "[CERRADA]";
        valveState = "[CERRADA]";
        myBot.sendMessage(msg, response_bot);
      }else if(strcmp((char*)buf, "VALVE-OPEN") == 0){
        response_bot += "Estado válvula: ";
        response_bot += "[ABIERTA]";
        valveState = "[ABIERTA]";
        myBot.sendMessage(msg, response_bot);
      }
      
      sendNRF24 = false;
      succes = false;
    }
  }

//--------------------- APAGAR LED ---------------------------
  if(!acceso){
    static uint32_t ledTime = millis();
    if (millis() - ledTime > 5000) {
      ledTime = millis();
      analogWrite(LED_GREEN, 0);
      analogWrite(LED_BLUE, 0);
      analogWrite(LED_RED, 0);
    }
  }
}

/**************** MQResistanceCalculation **************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
**********************************************************************************/ 
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
 
/*************************** MQCalibration **************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
**********************************************************************************/ 
float MQCalibration(int mq_pin)
{
  int i;
  float val=0;
 
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 
 
  return val; 
}
/***************************  MQRead *******************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
**********************************************************************************/ 
float MQRead(int mq_pin)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 
/***************************  MQGetGasPercentage ********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
**********************************************************************************/ 
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    
 
  return 0;
}
 
/***************************  MQGetPercentage ********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
**********************************************************************************/ 
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
