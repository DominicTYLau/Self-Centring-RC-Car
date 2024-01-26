#include <SPI.h>
#include <RH_NRF24.h>
#include <Servo.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x08
#define DEAD_ZONE 80

// I2C
volatile boolean receiveFlag = true;
char temp[BUFFER_LENGTH];
String command;

// Radio communication
RH_NRF24 nrf24;

// Servo and ESC
Servo servo;
Servo ESC;

// Joystick input
int xValue = 0;
int yValue = 0;
boolean isReceiving = false;

// PID parameters
double Kp = 0.1;
double Ki = 0.0002;
double Kd = 0;

// PID variables
double previousError = 0;
double integral = 0;

// Setpoint and current position
int setpoint = 160;
int currentPos = 160;

// Threshold for currentPos
int currentPosThreshold = 160;

void setup()
{
  // Initialize I2C as a slave
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);

  Serial.begin(9600);

  // Initialize NRF24 radio
  if (!nrf24.init())
    Serial.println("NRF24 initialization failed");
  if (!nrf24.setChannel(1))
    Serial.println("Failed to set NRF24 channel");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("Failed to set NRF24 RF parameters");

  // Attach servo and ESC
  servo.attach(9);
  servo.write(90);
  ESC.attach(5);
  ESC.writeMicroseconds(1000);
  Serial.read();
}

void receiveEvent(int howMany)
{
  for (int i = 0; i < howMany; i++)
  {
    temp[i] = Wire.read();
    temp[i + 1] = '\0'; // Add null after each character
  }

  // Shift everything to the left to remove command byte
  for (int i = 0; i < howMany; ++i)
    temp[i] = temp[i + 1];

  receiveFlag = true;
}

void loop()
{
  if (nrf24.available())
  {
    // Message received from joystick
    uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (nrf24.recv(buf, &len))
    {
      String input = String((char *)buf);

      // Parse joystick values and control mode from input
      xValue = constrain(input.substring(0, input.indexOf(" ")).toInt(), 0, 640);
      input.remove(0, input.indexOf(" ") + 1);
      yValue = constrain(input.substring(0, input.indexOf(" ")).toInt(), 0, 640);
      input.remove(0, input.indexOf(" ") + 1);
      isReceiving = !(input.toInt());
    }
    else
    {
      Serial.println("Failed to receive data from joystick");
    }
  }

  if (isReceiving)
  {
    // Joystick control mode
    adjustServoAndESC();
  }
  else
  {
    // PID control mode
    receiveDataAndPerformPID();
  }
}

void adjustServoAndESC()
{
  if (xValue < 320 - DEAD_ZONE)
  {
    servo.write(map(xValue, 0, 320 - DEAD_ZONE, 50, 90));
  }
  else if (xValue > 320 + DEAD_ZONE)
  {
    servo.write(map(xValue, 320 + DEAD_ZONE, 640, 90, 130));
  }
  else
  {
    servo.write(90);
  }

  if (yValue > 320 + DEAD_ZONE)
  {
    ESC.writeMicroseconds(map(yValue, 320 + DEAD_ZONE, 640, 1000, 2000));
  }
  else
  {
    ESC.writeMicroseconds(1000);
  }
}

void receiveDataAndPerformPID()
{
  ESC.writeMicroseconds(1250);
  Wire.onReceive(receiveEvent);
  if (receiveFlag)
  {
    String data(temp);
    receiveFlag = false;

    // Update setpoint and current position
    currentPos = data.toInt();
    setpoint = 160;


    // Calculate error
    double error = setpoint - currentPos;

    // Calculate PID components
    double proportional = Kp * error;
    integral += Ki * error;
    double derivative = Kd * (error - previousError);

    // Calculate PID output
    double output = proportional + integral + derivative;

    // Update servo angle
    int servoAngle = 90 - output;
    Serial.println(output);
    Serial.println(servoAngle);
    servo.write(servoAngle);

    // Update previous error for the next iteration
    previousError = error;

    // Add a delay to control the loop rate
    delay(50);
  }
}
