#include <SPI.h>
#include <RH_NRF24.h>

RH_NRF24 nrf24;

#define VRX_PIN A0 // Arduino pin connected to VRX and VRY pin
#define VRY_PIN A1

const int switchPin = 4; 

int xValue = 0; // To store value of the X axis
int yValue = 0; // To store value of the Y axis

boolean status = false;


void setup()
{
  Serial.begin(9600);

  pinMode(switchPin, INPUT);
  digitalWrite(switchPin, HIGH);

  while (!Serial)
    ; // wait for serial port to connect. Needed for Leonardo only
  if (!nrf24.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");
}

void loop()
{
  if(digitalRead(switchPin) == false){
    status = !status;
  }

  xValue = analogRead(VRX_PIN);
  yValue = analogRead(VRY_PIN);
  if(yValue > 800){
    yValue = 800;
  }

  String message = String(xValue) + " " + String(yValue) +" "+ String(status);
  Serial.println(message);

  char data[message.length() + 1];
  message.toCharArray(data, sizeof(data));

  nrf24.send(data, sizeof(data));

  nrf24.waitPacketSent();
  delay(200);
}
