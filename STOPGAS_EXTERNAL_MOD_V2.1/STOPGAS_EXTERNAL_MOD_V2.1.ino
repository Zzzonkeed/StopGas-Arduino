#include <SPI.h>
#include <RH_NRF24.h>

RH_NRF24 nrf24(9, 10);

#define LED_GREEN 5
#define LED_RED   3
#define LED_BLUE  6

#define VOLTAGE_PIN A0

#define NRF24_CHANNEL 4

#define VALVE_PIN 7

char DataToSend[12];
bool valveClose = false;

bool alarm = false;
int sensorValue = 0;
float voltage = 0.0;
char voltageSend[6];
int done = 0;

void setup() 
{
  Serial.begin(115200);
  //analogReference(INTERNAL);
  if (!nrf24.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(NRF24_CHANNEL))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");

  pinMode(VALVE_PIN, OUTPUT);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  digitalWrite(VALVE_PIN, HIGH);
}

void loop(){
  static uint32_t ADCTime = millis();
  if(millis() - ADCTime > 1000){
    ADCTime = millis();
    sensorValue = analogRead(VOLTAGE_PIN);
    voltage = (sensorValue * (3.3 / 1023.0))*2.5;
    dtostrf(voltage, 0, 2, voltageSend);
  }

  if (nrf24.available()){
    // Should be a message for us now   
    uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (nrf24.recv(buf, &len)){
      Serial.print("got request: ");
      Serial.println((char*)buf);

      if(strcmp((char*)buf, "CLOSE-VALVE") == 0){
        strcpy(DataToSend, "VALVE-CLOSE");
        valveClose = true;
        done = 0;
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_BLUE, LOW);
        digitalWrite(LED_RED, HIGH);
        alarm = true;
        strcpy((char*)buf, " ");
      }

      if(strcmp((char*)buf, "STATE-VALVE") == 0){
        Serial.println("Estado de la valvula");
        if(valveClose){
          strcpy(DataToSend, "VALVE-CLOSE");
        }else if(!valveClose){
          strcpy(DataToSend, "VALVE-OPEN");
        }
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(LED_BLUE, LOW);
        digitalWrite(LED_RED, LOW);
        strcmp((char*)buf, " ");
      }

      if(strcmp((char*)buf, "VALVE-RESET") == 0){
        Serial.println("Reset valvula");
        if(valveClose){
          valveClose = false;
          strcpy(DataToSend, "VALVE-OPEN");
        }
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_BLUE, HIGH);
        digitalWrite(LED_RED, LOW);
        alarm = false;
        strcpy((char*)buf, " ");
      }    
      // Send a reply
      // uint8_t DataToSend[] = "And hello back to you";
      nrf24.send((uint8_t*)DataToSend, sizeof(DataToSend));
      nrf24.waitPacketSent();
      //Serial.println("Sent a reply");
    }
    else
    {
      Serial.println("recv failed");
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_BLUE, LOW);
      digitalWrite(LED_RED, HIGH);
    }
  }

  if(valveClose){
    if(done == 0){
      digitalWrite(VALVE_PIN, LOW);
      delay(100);
      digitalWrite(VALVE_PIN, HIGH);
      done = 1;
    }
  }

  if(alarm){
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_RED, HIGH);
  }else{
    static uint32_t ledTime = millis();
    if (millis() - ledTime > 10000) {
      ledTime = millis();
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_BLUE, LOW);
      digitalWrite(LED_RED, LOW);
    }
  }
}
