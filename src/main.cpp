// Based on Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <memory>

#include <N2kMessages.h>
#include <NMEA2000_esp32.h>
#include <NMEA2000_CAN.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


#include "sensesp.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include "sensesp_onewire/onewire_temperature.h"
#include "sensesp/ui/config_item.h"


// CAN bus (NMEA 2000) pins on SH-ESP32
#define CAN_RX_PIN GPIO_NUM_34
#define CAN_TX_PIN GPIO_NUM_32

//BME stuff

#define SDA 16
#define SCL 17
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;

using namespace sensesp;
using namespace sensesp::onewire;

tNMEA2000* nmea2000;

double temperature_1 = N2kDoubleNA;
double temperature_2 = N2kDoubleNA;
double BarometricPressure = N2kDoubleNA;
double ctemp = 0;
//dont know what this does
unsigned long delayTime;

/**
 * @brief Send Ambient temp data
 *
 * Send engine temperature data using the Temperature ext PGN.
 */
 
void printValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  // Convert temperature to Fahrenheit
  /*Serial.print("Temperature = ");
  Serial.print(1.8 * bme.readTemperature() + 32);
  Serial.println(" *F");*/

  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}

//this sends pressure to n2k

void sendPressure() {
  BarometricPressure = bme.readPressure();
  tN2kMsg N2kMsg;
  SetN2kPressure(N2kMsg, 2, 1, N2kps_Atmospheric, BarometricPressure);
  nmea2000->SendMsg(N2kMsg);
  Serial.println("Sent the press to n2k!");

}

// The setup function performs one-time application initialization.
void setup() {

  


  SetupLogging(ESP_LOG_DEBUG);
//bme test
  Serial.begin(115200);
  Wire.begin(SDA,SCL);
  delay(100);
  Serial.println(F("BME280 test"));

  bool status;

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin();
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Serial.println("-- Default Test --");
  delayTime = 1000;

  Serial.println();

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("shesp32-sensors-to-n2k")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi_client("ssid", "pwd")
                    //->set_wifi_access_point("My AP SSID", "my_ap_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();
  const uint8_t one_wire_pin = 4;
  
  DallasTemperatureSensors* dts = new DallasTemperatureSensors(one_wire_pin);

  // define three 1-Wire temperature sensors that update every 1000 ms
  // and have specific web UI configuration paths

  

  auto ambient_temperature_1 =
      new OneWireTemperature(dts, 1000, "/ambientTemp1/oneWire");
  
  auto main_engine_exhaust_temperature =
      new OneWireTemperature(dts, 1000, "/mainEngineWetExhaustTemp/oneWire");
  auto ambient_temperature_2 =
      new OneWireTemperature(dts, 1000, "/ambientTemp2/oneWire");

  // read atmo pressure from BME    
   
  // auto atmospheric_pressure = bme.readPressure() / 100.0F;

  nmea2000 = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);

  // Reserve enough buffer for sending all messages. This does not work on small
  // memory devices like Uno or Mega
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  nmea2000->SetProductInformation(
      "20210405",  // Manufacturer's Model serial code (max 32 chars)
      103,         // Manufacturer's product code
      "SH-ESP32 Temp Sensor",  // Manufacturer's Model ID (max 33 chars)
      "0.1.0.0 (2025-05-30)",  // Manufacturer's Software version code (max 40
                               // chars)
      "0.1.0.0 (2025-05-30)"   // Manufacturer's Model version (max 24 chars)
  );
  // Set device information
  nmea2000->SetDeviceInformation(
      1,    // Unique number. Use e.g. Serial number.
      130,  // Device function=Analog to NMEA 2000 Gateway. See codes on
            // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      75,   // Device class=Inter/Intranetwork Device. See codes on
           // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      2046  // Just choosen free from code list on
            // http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 22);
  // Disable all msg forwarding to USB (=Serial)
  nmea2000->EnableForward(false);
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  
  event_loop()->onRepeat(1, []() { nmea2000->ParseMessages(); });

  main_engine_exhaust_temperature->connect_to(
      new LambdaConsumer<float>([](float temperature) {
        tN2kMsg N2kMsg;
        SetN2kTemperature(N2kMsg,
                          1,                            // SID
                          1,                            // TempInstance
                          N2kts_ExhaustGasTemperature,  // TempSource
                          temperature                   // actual temperature
        );
        nmea2000->SendMsg(N2kMsg);
        ctemp = temperature -273.15; 
        Serial.print("Sent tempexh ");
        Serial.println(ctemp);
      }));
  
  ambient_temperature_1->connect_to(
      new LambdaConsumer<float>([](float temperature) {
        tN2kMsg N2kMsg;
        SetN2kTemperature(N2kMsg, 2, 2, N2kts_OutsideTemperature, temperature_1);
        nmea2000->SendMsg(N2kMsg);
        ctemp = temperature -273.15; 
        Serial.print("Sent temp1 ");
        Serial.println(ctemp);        
      }));

  ambient_temperature_2->connect_to(
      new LambdaConsumer<float>([](float temperature) {
        tN2kMsg N2kMsg;
        SetN2kTemperature(N2kMsg, 3, 3, N2kts_MainCabinTemperature, temperature_2);
        nmea2000->SendMsg(N2kMsg); 
        ctemp = temperature -273.15; 
        Serial.print("Sent temp2 ");
        Serial.println(ctemp);       
      }));

  

  
    

 
  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void loop() { 
  //printValues();
  sendPressure();
  delay(delayTime);
  event_loop()->tick(); }