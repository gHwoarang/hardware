#include <HardwareSerial.h>   //ESP32 için
#include <WiFi.h>
//#include <SoftwareSerial.h>  //Arduino UNO için

#define LEAST_FRAME_SIZE            5
#define CMD_HEAD                    0x3E
#define CMD_OPEN_CONTROL            0xA0  
#define CMD_TORQUE_CONTROL          0xA1  
#define CMD_SPEED_CONTROL           0xA2  
#define CMD_ANGLE_CONTROL1          0xA3  
#define CMD_READ_ENCODER            0x90
#define CMD_READ_STATE2             0x9C

uint8_t ctlCmd = CMD_OPEN_CONTROL;
uint8_t motorId = 1;
int64_t ctlValue = 0;

uint8_t uart1TxBuffer[200];
uint8_t uart1RxBuffer[200];
uint8_t uart1TxDataSize = 0;
uint8_t uart1RxDataSize = 0;

int8_t motorTemperature = 0;
int16_t motorPowerOrTorque = 0;
int16_t motorSpeed = 0;
uint16_t motorEncoder = 0;
uint16_t motorEncoderOriginal = 0;
uint16_t motorEncoderZero = 0;

void control_Send(uint8_t cmd, uint8_t id, int64_t value);
void control_PackCmd(uint8_t *buffer, uint8_t cmd, uint8_t id, uint8_t size, uint8_t *data);
uint8_t control_CheckReceivedData(void);
uint8_t control_CheckReceivedEncoderData(void);
void uartReceiveData(void);

const char* ssid = "GimbalController";
const char* password = "123456789";

WiFiServer server(80);

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  Serial.print("Setting AP \n");
  WiFi.softAP(ssid, password);
  Serial.print("AP IP Address: ");
  Serial.println(WiFi.softAPIP());
  server.begin();
}

void loop() {
  
  delay(300);
  /*control_Send(CMD_READ_STATE2,3,0);
  Serial.println(Serial.available());
  uartReceiveData();
  control_CheckReceivedData();*/
  control_Send(CMD_OPEN_CONTROL,8,100);
  delay(100);
  uartReceiveData();
  control_CheckReceivedData();
}


void control_PackCmd(uint8_t *buffer, uint8_t cmd, uint8_t id, uint8_t size, uint8_t *data)
{
  uint8_t i = 0;

  buffer[0] = CMD_HEAD;
  buffer[1] = cmd;
  buffer[2] = id;
  buffer[3] = size;
  buffer[4] = 0; 
  for (i=0; i<4; i++)
    buffer[4] += buffer[i];
  
  if (size != 0)
  {
    buffer[LEAST_FRAME_SIZE+size] = 0;
    for (i=0; i<size; i++)
    {
      buffer[LEAST_FRAME_SIZE+i] = data[i];
      buffer[LEAST_FRAME_SIZE+size] += buffer[LEAST_FRAME_SIZE+i];
    }
    uart1TxDataSize = i + LEAST_FRAME_SIZE + 1; 
  }
  else
    uart1TxDataSize = LEAST_FRAME_SIZE ; 
}


void control_Send(uint8_t cmd, uint8_t id, int64_t value)
{
  uint8_t dataSize = 0; 
  int16_t openCtlData = 0;  
  int16_t torqueCtlData = 0;  
  int16_t speedCtlData = 0; 
  int16_t angleCtlData = 0; 
  
  if (cmd == CMD_OPEN_CONTROL)
  {
    openCtlData = value;
    dataSize = 2;
    
    control_PackCmd(uart1TxBuffer, cmd, id, dataSize, (uint8_t *)&openCtlData);
    Serial.write(uart1TxBuffer, uart1TxDataSize);
    uart1RxDataSize = LEAST_FRAME_SIZE + 7 + 1;
  }
  else if (cmd == CMD_TORQUE_CONTROL)
  {
    torqueCtlData = value;
    dataSize = 2;
    
    control_PackCmd(uart1TxBuffer, cmd, id, dataSize, (uint8_t *)&torqueCtlData);
    Serial.write(uart1TxBuffer, uart1TxDataSize);
    uart1RxDataSize = LEAST_FRAME_SIZE + 7 + 1;
  }
  else if (cmd == CMD_SPEED_CONTROL)
  {
    speedCtlData = value;
    dataSize = 4;
    
    control_PackCmd(uart1TxBuffer, cmd, id, dataSize, (uint8_t *)&speedCtlData);
    Serial.write(uart1TxBuffer, uart1TxDataSize);;
    uart1RxDataSize = LEAST_FRAME_SIZE + 7 + 1;
  } 
  else if (cmd == CMD_ANGLE_CONTROL1)
  {
    angleCtlData = value;
    dataSize = 8;
    
    control_PackCmd(uart1TxBuffer, cmd, id, dataSize, (uint8_t *)&angleCtlData);
    Serial.write(uart1TxBuffer, uart1TxDataSize);
    uart1RxDataSize = LEAST_FRAME_SIZE + 7 + 1;
  }
}

uint8_t control_CheckReceivedData(void)
{
  uint8_t receiveSuccess = 0;
  uint8_t temp = 0;
  uint8_t i = 0;

  if (uart1RxBuffer[0] == CMD_HEAD)
  {
    temp = uart1RxBuffer[0] + uart1RxBuffer[1] + uart1RxBuffer[2] + uart1RxBuffer[3];
    if (uart1RxBuffer[4] == temp)
    {
      temp = uart1RxBuffer[5] + uart1RxBuffer[6] + uart1RxBuffer[7] + uart1RxBuffer[8] + uart1RxBuffer[9] + uart1RxBuffer[10] + uart1RxBuffer[11];
      if (uart1RxBuffer[12] == temp)
      {
        motorTemperature = (int8_t)uart1RxBuffer[5];
        Serial.print("motorTemperature = ");
        Serial.println(motorTemperature);
        motorPowerOrTorque = (int16_t)(uart1RxBuffer[6] + (uart1RxBuffer[7]<<8));
        Serial.print("motorPowerOrTorque = ");
        Serial.println(motorPowerOrTorque);
        motorSpeed = (int16_t)(uart1RxBuffer[8] + (uart1RxBuffer[9]<<8)); 
        Serial.print("motorSpeed = ");
        Serial.println(motorSpeed);  
        motorEncoder = (int16_t)(uart1RxBuffer[10] + (uart1RxBuffer[11]<<8));
        Serial.print("motorEncoder = ");
        Serial.println(motorEncoder);     
        receiveSuccess = 1;
        WiFiClient client = server.available();
          client.print(motorEncoder);
          Serial.println("Data have been sent to the client");
          delay(100);
      }
    }
  }
  
  for (i=0; i<uart1RxDataSize; i++)
    uart1RxBuffer[i] = 0;
  uart1RxDataSize = 0;
  
  return receiveSuccess;
}

uint8_t control_CheckReceivedEncoderData(void)
{
  uint8_t receiveSuccess = 0;
  uint8_t temp = 0;
  uint8_t i = 0;

  if (uart1RxBuffer[0] == CMD_HEAD)
  {
    temp = uart1RxBuffer[0] + uart1RxBuffer[1] + uart1RxBuffer[2] + uart1RxBuffer[3];
    if (uart1RxBuffer[4] == temp)
    {
      temp = uart1RxBuffer[5] + uart1RxBuffer[6] + uart1RxBuffer[7] + uart1RxBuffer[8] + uart1RxBuffer[9] + uart1RxBuffer[10];
      if (uart1RxBuffer[11] == temp)
      {
        motorEncoder = (int16_t)(uart1RxBuffer[5] + (uart1RxBuffer[6]<<8));
        Serial.print("motorEncoder = ");
        Serial.println(motorEncoder);
        motorEncoderOriginal = (int16_t)(uart1RxBuffer[7] + (uart1RxBuffer[8]<<8));
        Serial.print("MotorEncoderOriginal = ");
        Serial.println(motorEncoderOriginal);
        motorEncoderZero = (int16_t)(uart1RxBuffer[9] + (uart1RxBuffer[10]<<8)); 
        Serial.print("MotorEncoderZero = ");
        Serial.println(motorEncoderZero);       
        receiveSuccess = 1;
      }
    }
  }
  
  for (i=0; i<uart1RxDataSize; i++)
    uart1RxBuffer[i] = 0;
  uart1RxDataSize = 0;
  
  return receiveSuccess;
}

void uartReceiveData(void){
 int i = 0;
 Serial.println("READ");
 while (Serial.available() > 0) {
 uart1RxBuffer[i] = Serial.read();
 i++;
 //Serial.println(uart1RxBuffer[i],HEX);

 }
}
