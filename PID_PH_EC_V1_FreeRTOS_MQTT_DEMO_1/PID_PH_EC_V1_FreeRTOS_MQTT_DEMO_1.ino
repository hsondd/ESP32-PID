#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#include <DHT.h>
#include <Wire.h>
#include <HardwareSerial.h> 
#include "SPI.h"
#include <BH1750FVI.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <PubSubClient.h>

//==========================================================================

const char* ssid = "lab411";
const char* password = "ktttlab411";
const char* mqtt_server = "192.168.1.5";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

//==========================================================================

//define lora
//#define rx 16                                          //LORA TX
//#define tx 17                                          //LORA RX
//HardwareSerial myserial(1);

//define mcp
#define CS_PIN 15     // ESP32 default SPI pins
#define CLOCK_PIN 14  // Should work with any other GPIO pins, since the library does not formally
#define MOSI_PIN 13   // use SPI, but rather performs pin bit banging to emulate SPI communication.
#define MISO_PIN 12   //
#define MCP3204 4     // (Generally "#define MCP320X X", where X is the last model digit/number of inputs)

//define illuminance
uint8_t ADDRESSPIN = 18;
BH1750FVI::eDeviceAddress_t DEVICEADDRESS = BH1750FVI::k_DevAddress_H;
BH1750FVI::eDeviceMode_t DEVICEMODE = BH1750FVI::k_DevModeContHighRes;
BH1750FVI LightSensor(ADDRESSPIN, DEVICEADDRESS, DEVICEMODE);

//define t.,h.
#define DHTPIN 32
#define DHTTYPE DHT22
DHT dht(DHTPIN,DHTTYPE);

//define soil moisture 
float mois;
int output_value;
int measurePin = 39;
int digital_mois = 4;

//define PH
#define samplingInterval 20
#define print_Interval 800
#define ArrayLenth  40      //times of collection
#define Offset -0.5
int pHArray[ArrayLenth];    //Store the average value of the sensor feedback
int pHArrayIndex=0;

//define EC
#define StartConvert 0
#define ReadTemperature 1
#define AnalogSampleInterval 25 //  analog sample interval
#define printInterval 800       //  serial print interval
#define tempSampleInterval 850  //  temperature sample interval
const byte numReadings = 20;    //  the number of sample times
byte DS18B20_Pin = 2;           //  DS18B20 signal, pin on digital 2
unsigned int readings[numReadings];       // the readings from the analog input
byte Index = 0;                           // the index of the current reading
unsigned long AnalogValueTotal = 0;                 // the running total
unsigned int AnalogAverage = 0,averageVoltage=0;    // the average
float temperature,ECcurrent;

//Temperature chip i/o
OneWire ds(DS18B20_Pin);

//Define PID
#define RELAY_1 5
#define RELAY_2 26
#define RELAY_3 25
#define RELAY_4 33

double Setpoint_Target_pH, Input_pH, Output_pH, Kp_pH, Ki_pH, Kd_pH;
double Setpoint_Target_EC, Input_EC, Output_EC, Kp_EC, Ki_EC, Kd_EC;

double Ratio_A_solution = 0.5;
double Temperature_solution;

double Total_time_pump_ph_up =0;
double Total_time_pump_ph_down=0;
double Total_time_pump_A=0;
double Total_time_pump_B=0;
double Total_time_pump_water=0;

PID myPID_pH(&Input_pH, &Output_pH, &Setpoint_Target_pH, Kp_pH, Ki_pH, Kd_pH, DIRECT);
PID myPID_EC(&Input_EC, &Output_EC, &Setpoint_Target_EC, Kp_EC, Ki_EC, Kd_EC, DIRECT);


// define two tasks for Blink & AnalogRead
void Task_PID_PH( void *pvParameters );
void Task_PID_EC( void *pvParameters );


void setup() { // Start the serial so we can see some output
  Serial.begin(9600);
  dht.begin();
  pinMode(39,INPUT);
  LightSensor.begin();  
  for (byte thisReading = 0; thisReading < numReadings; thisReading++)
  readings[thisReading] = 0;
  TempProcess(StartConvert);   //let the DS18B20 start the convert
  
  SPI.begin(CLOCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  pinMode(CS_PIN, OUTPUT);
//  myserial.begin(9600, SERIAL_8N1, rx, tx);
  digitalWrite(23,LOW);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(RELAY_1,OUTPUT);
  pinMode(RELAY_2,OUTPUT);
  pinMode(RELAY_3,OUTPUT);
  pinMode(RELAY_4,OUTPUT);

  digitalWrite(RELAY_1,HIGH);
  digitalWrite(RELAY_2,HIGH);
  digitalWrite(RELAY_3,HIGH);
  digitalWrite(RELAY_4,HIGH);

  xTaskCreatePinnedToCore(
    Task_MQTT
    ,  "MQTT"
    ,  5120  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(
    Task_PID_EC
    ,  "PID_EC"
    ,  10240  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(
    Task_PID_PH
    ,  "PID_PH"   // A name just for humans
    ,  5120  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}
void loop() {
// myserial.println("ok");
// read_ph();
// delay(1000);
// myserial.println("ok");
// read_dht22();
// delay(1000);
// myserial.println("ok");
// read_mois();
// delay(1000);
// myserial.println("ok");
// read_lux();
// delay(1000);
// myserial.println("ok");
// read_EC();
// delay(10000);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void Task_PID_PH(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
  // Setup PID PH
  Setpoint_Target_pH = 4.8;
  myPID_pH.SetOutputLimits(-50,50);
  myPID_pH.SetTunings(20,1,0.1); 
  myPID_pH.SetSampleTime (20000) ;
  myPID_pH.SetControllerDirection(DIRECT) ;
  myPID_pH.SetMode(AUTOMATIC);

  // A Task shall never return or exit.
  for (;;) {
    read_ph();
  }
}

void Task_PID_EC(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
  // Setup PID EC
  Setpoint_Target_EC = 1.200;
  myPID_EC.SetOutputLimits(-100,100);
  myPID_EC.SetTunings(300,3,10); 
  myPID_EC.SetSampleTime (40000) ;
  myPID_EC.SetControllerDirection(DIRECT) ;
  myPID_EC.SetMode(AUTOMATIC);
  
  for (;;) {
    read_EC();
  }
}

void Task_MQTT(void *pvParameters)
{
  (void) pvParameters;

  for(;;){
    mqtt_Updata();
  }
}

void mqtt_Updata()
{
    if (!client.connected()) {
    reconnect();
  }
  client.loop();
//
//  long now = millis();
//  if (now - lastMsg > 2000) {
//    lastMsg = now;
//    // Convert the value to a char array
//    char PHString[8];
//    dtostrf(Input_pH, 1, 2, PHString);
//    Serial.print("Độ pH: ");
//    Serial.println(PHString);
//    client.publish("esp32/PH", PHString);
//    
//    char ECString[8];
//    dtostrf(Input_EC, 1, 3, ECString);
//    Serial.print("Giá trị EC: ");
//    Serial.println(ECString);
//    client.publish("esp32/EC", ECString);
//    
//    char TEMPString[8];
//    dtostrf(Temperature_solution, 1, 2, TEMPString);
//    Serial.print("Giá trị Temperature: ");
//    Serial.println(TEMPString);
//    client.publish("esp32/TEMPERATURE_SOLUTION", TEMPString);
//
//    char Pump_PH_UP_String[8];
//    dtostrf(Total_time_pump_ph_up, 1, 1, Pump_PH_UP_String);
//    Serial.print("Tổng dung tích pH Up: ");
//    Serial.println(Pump_PH_UP_String);
//    client.publish("esp32/TOTAL_PUMP_PH_UP", Pump_PH_UP_String);
//    
//    char Pump_PH_DOWN_String[8];
//    dtostrf(Total_time_pump_ph_down, 1, 1, Pump_PH_DOWN_String);
//    Serial.print("Tổng dung tích pH Down: ");
//    Serial.println(Pump_PH_DOWN_String);
//    client.publish("esp32/TOTAL_PUMP_PH_DOWN", Pump_PH_DOWN_String);
//    
//    char Pump_A_String[8];
//    dtostrf(Total_time_pump_A, 1, 1, Pump_A_String);
//    Serial.print("Tổng dung tích dung dịch A: ");
//    Serial.println(Pump_A_String);
//    client.publish("esp32/TOTAL_PUMP_A", Pump_A_String);
//    
//    char Pump_B_String[8];
//    dtostrf(Total_time_pump_B, 1, 1, Pump_B_String);
//    Serial.print("Tổng dung tích dung dịch B: ");
//    Serial.println(Pump_B_String);
//    client.publish("esp32/TOTAL_PUMP_B", Pump_B_String);
//    
//    char Pump_WATER_String[8];
//    dtostrf(Total_time_pump_water, 1, 1, Pump_WATER_String);
//    Serial.print("Tổng dung tích nước: ");
//    Serial.println(Pump_WATER_String);
//    client.publish("esp32/TOTAL_PUMP_WATER", Pump_WATER_String);
//  }
}
int read_mcp(int channel){
byte data1;
byte data2;
digitalWrite(CS_PIN,HIGH);
digitalWrite(CS_PIN,LOW);
SPI.transfer(0x06);
switch( channel)
{
  case 0:
  {
    data1 = SPI.transfer(0x00);
    break;
  }
  case 1:
  {
    data1 = SPI.transfer(0x40);
    break;
  }
  case 2:
  {
    data1 = SPI.transfer(0x80);
    break;
  }
  case 3:
  {
    data1 = SPI.transfer(0xc0);
    break;
  }
  default: break;
}
data2 = SPI.transfer(0x00);
//digitalWrite(CS_PIN,HIGH);
uint16_t  analogPH = ((data1<<8)|data2) & 0b0000111111111111;
return analogPH;
}

double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}

//PH
void read_ph(){
  static unsigned long runPIDTime = millis();
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;
  if(millis()-samplingTime > samplingInterval)
  {
      samplingTime=millis();
      pHArray[pHArrayIndex++]=read_mcp(0);
      if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
      voltage = avergearray(pHArray, ArrayLenth)*5.0/4095;
      pHValue = 3.5*voltage+Offset;
      
  }
  if(millis() - printTime > print_Interval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    printTime=millis();
    Input_pH = pHValue; 
    Serial.print(" | Voltage: ");
    Serial.print(voltage*1000);
    Serial.print(" mV ");
    Serial.print(" | pH value: ");
    Serial.println(pHValue,3);
    
  }
  if(millis() - runPIDTime >= 20000) {
    runPIDTime = millis();
    myPID_pH.Compute();
//    Serial.print(" => OUTPUT_PH : ");
//    Serial.println(Output_pH);
    char PHString[8];
    dtostrf(Input_pH, 1, 2, PHString);
    Serial.print("Độ pH: ");
    Serial.println(PHString);
    client.publish("esp32/PH", PHString);
    char output_PH_String[8];
    dtostrf(Output_pH, 1, 2, output_PH_String);
    Serial.print("Tín hiệu điều khiển bơm PH: ");
    Serial.println(output_PH_String);
    client.publish("esp32/OUTPUT_PID_PH", output_PH_String);
//    if(Input_pH<(Setpoint_Target_pH - 0.02) || Input_pH>(Setpoint_Target_pH + 0.02)) {
      Pump_pH_Solution(Output_pH);
//    }
    
  }
}
//EC
void read_EC(){
  static unsigned long AnalogSampleTime = millis();
  static unsigned long tempSampleTime = millis();
  static unsigned long printTime = millis();
  static unsigned long runPIDTime = millis();
 if(millis()-AnalogSampleTime>=AnalogSampleInterval)
  {
    AnalogSampleTime=millis();
    // subtract the last reading:
    AnalogValueTotal = AnalogValueTotal - readings[Index];
    // read from the sensor:
    //readings[Index] = mcp.readADC(channel_1);
    readings[Index] = read_mcp(1);
    // add the reading to the total:
    AnalogValueTotal = AnalogValueTotal + readings[Index];
    // advance to the next position in the array:
    Index = Index + 1;
    // if we're at the end of the array...
    if (Index >= numReadings)
    // ...wrap around to the beginning:
    Index = 0;
    // calculate the average:
    AnalogAverage = AnalogValueTotal / numReadings;
  }
  /*
   Every once in a while,MCU read the temperature from the DS18B20 and then let the DS18B20 start the convert.
   Attention:The interval between start the convert and read the temperature should be greater than 750 millisecond,or the temperature is not accurate!
  */
   if(millis()-tempSampleTime>=tempSampleInterval)
  {
    tempSampleTime=millis();
    temperature = TempProcess(ReadTemperature);  // read the current temperature from the  DS18B20
    TempProcess(StartConvert);                   //after the reading,start the convert for next reading
    char TEMPString[8];
    dtostrf(temperature, 1, 2, TEMPString);
    Serial.print("Giá trị Temperature: ");
    Serial.println(TEMPString);
    client.publish("esp32/TEMPERATURE_SOLUTION", TEMPString);
  }
   /*
   Every once in a while,print the information on the serial monitor.
  */
if(millis()-printTime>=printInterval){
    printTime=millis();
    averageVoltage=AnalogAverage*(float)5000/4095;
    float TempCoefficient=1.0+0.0185*(temperature-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.0185*(fTP-25.0));
    float CoefficientVolatge=(float)averageVoltage/TempCoefficient;
    // if(CoefficientVolatge<190)Serial.println("No solution!");     //25^C 1413us/cm<-->about 216mv  if the voltage(compensate)<150,that is <1ms/cm,out of the range
    if(CoefficientVolatge>2500)Serial.println("Out of the range!");  //>20ms/cm,out of the range
    else {
      if(CoefficientVolatge<=805)ECcurrent=5.03*CoefficientVolatge+481.5113;   //1ms/cm<EC<=4ms/cm
      else if(CoefficientVolatge<=1410)ECcurrent=3.15*CoefficientVolatge+1987.92;  //4ms/cm<EC<=7ms/cm
      else ECcurrent=17.426*CoefficientVolatge-18107.218;                           //7ms/cm<EC<14ms/cm
      ECcurrent/=1000;
      Serial.print(" | Analog value: ");
      Serial.print(AnalogAverage);   //analog average,from 0 to 4095
      Serial.print(" | Voltage: ");
      Serial.print(averageVoltage);  //millivolt average,from 0mv to 4995mV
      Serial.print(" mV ");
      Serial.print(" | Temp:");
      Serial.print(temperature);    //current temperature
      Serial.print(" | EC: ");
      Serial.print(ECcurrent,3);  //two decimal
      Serial.println(" ms/cm ");
      Input_EC = ECcurrent;
//      Temperature_solution = temperature;
      //Serial.println(CoefficientVolatge);
    }
  }
if(millis()-runPIDTime >= 40000) {
    runPIDTime= millis();
    myPID_EC.Compute();
//    Serial.print(" => OUTPUT_EC : ");
//    Serial.println(Output_EC);
    char ECString[8];
    dtostrf(Input_EC, 1, 3, ECString);
    Serial.print("Giá trị EC: ");
    Serial.println(ECString);
    client.publish("esp32/EC", ECString);
    char output_EC_String[8];
    dtostrf(Output_EC, 1, 2, output_EC_String);
    Serial.print("Tín hiệu điều khiển bơm EC: ");
    Serial.println(output_EC_String);
    client.publish("esp32/OUTPUT_PID_EC", output_EC_String);
//    if(Input_EC<(Setpoint_Target_EC - 0.005) || Input_EC>(Setpoint_Target_EC + 0.005)){
      Pump_EC_Solution(Output_EC);
//    }
}
}
float TempProcess(bool ch)
{
  //returns the temperature from one DS18B20 in DEG Celsius
  static byte data[12];
  static byte addr[8];
  static float TemperatureSum;
  if(!ch){
          if ( !ds.search(addr)) {
              Serial.println("no more sensors on chain, reset search!");
              ds.reset_search();
              return 0;
          }
          if ( OneWire::crc8( addr, 7) != addr[7]) {
              Serial.println("CRC is not valid!");
              return 0;
          }
          if ( addr[0] != 0x10 && addr[0] != 0x28) {
              Serial.print("Device is not recognized!");
              return 0;
          }
          ds.reset();
          ds.select(addr);
          ds.write(0x44,1); // start conversion, with parasite power on at the end
  }
  else{
          byte present = ds.reset();
          ds.select(addr);
          ds.write(0xBE); // Read Scratchpad
          for (int i = 0; i < 9; i++) { // we need 9 bytes
            data[i] = ds.read();
          }
          ds.reset_search();
          byte MSB = data[1];
          byte LSB = data[0];
          float tempRead = ((MSB << 8) | LSB); //using two's compliment
          TemperatureSum = tempRead / 16;
    }
          return TemperatureSum;
}
void read_lux(){
  uint16_t lux = LightSensor.GetLightIntensity();
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lux");
  delay(1000);
  }
void read_dht22(){
  float temp=dht.readTemperature();
  float hum=dht.readHumidity();
  if (isnan(temp)||isnan(hum)) {    
    Serial.println("Failed to read from DHT sensor!");}
  else{
  Serial.println("Temperature:");
  Serial.print(temp);
  Serial.println("*C");
  Serial.println("Humidity:");
  Serial.print(hum);
  Serial.println("%");}
  delay(1000);   
}
void read_mois(){
  int value = analogRead(measurePin);
  mois = map(value,4095,1200,0,100);
  Serial.println("Do am dat : ");
  Serial.print(mois);
  Serial.println("%");
//  Serial.println(value);
  delay(1000);
}
//========================================================================================PUMP-PH================================================
void Pump_pH_Solution(int number_cycles) {
  if(number_cycles>=0) {
   Total_time_pump_ph_up += number_cycles/10;
    char Pump_PH_UP_String[8];
    dtostrf(Total_time_pump_ph_up, 1, 1, Pump_PH_UP_String);
    Serial.print("Tổng dung tích pH Up: ");
    Serial.println(Pump_PH_UP_String);
    client.publish("esp32/TOTAL_PUMP_PH_UP", Pump_PH_UP_String);
   digitalWrite(RELAY_3,LOW);
   delay(50*number_cycles);
   digitalWrite(RELAY_3,HIGH);
  
  }
 else {
  number_cycles = number_cycles*(-1);
  Total_time_pump_ph_down+=number_cycles/10;
   char Pump_PH_DOWN_String[8];
    dtostrf(Total_time_pump_ph_down, 1, 1, Pump_PH_DOWN_String);
    Serial.print("Tổng dung tích pH Down: ");
    Serial.println(Pump_PH_DOWN_String);
    client.publish("esp32/TOTAL_PUMP_PH_DOWN", Pump_PH_DOWN_String);
  digitalWrite(RELAY_4,LOW);
  delay(50*number_cycles);
  digitalWrite(RELAY_4,HIGH);
 
  }
}
//========================================================================================PUMP-EC================================================
void Pump_EC_Solution(int number_cycles) {
  if(number_cycles>=0) {
    Total_time_pump_A+=number_cycles*Ratio_A_solution/10;
    Total_time_pump_B+=number_cycles*(1-Ratio_A_solution)/10;
    char Pump_A_String[8];
    dtostrf(Total_time_pump_A, 1, 1, Pump_A_String);
    Serial.print("Tổng dung tích dung dịch A: ");
    Serial.println(Pump_A_String);
    client.publish("esp32/TOTAL_PUMP_A", Pump_A_String);
    char Pump_B_String[8];
    dtostrf(Total_time_pump_B, 1, 1, Pump_B_String);
    Serial.print("Tổng dung tích dung dịch B: ");
    Serial.println(Pump_B_String);
    client.publish("esp32/TOTAL_PUMP_B", Pump_B_String);
    digitalWrite(RELAY_2,LOW);
    delay(50*number_cycles);
    digitalWrite(RELAY_2,HIGH);
    
  }
  else {
    number_cycles = number_cycles*(-1);
    Total_time_pump_water += number_cycles*5/10;
    char Pump_WATER_String[8];
    dtostrf(Total_time_pump_water, 1, 1, Pump_WATER_String);
    Serial.print("Tổng dung tích nước: ");
    Serial.println(Pump_WATER_String);
    client.publish("esp32/TOTAL_PUMP_WATER", Pump_WATER_String);
    digitalWrite(RELAY_1,LOW);
    delay(5*50*number_cycles);
    digitalWrite(RELAY_1,HIGH);
    
    
  }
}
