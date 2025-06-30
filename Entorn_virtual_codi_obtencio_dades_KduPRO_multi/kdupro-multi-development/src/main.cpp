#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include "RTClib.h"
#include <Adafruit_AS7341.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include "driver/rtc_io.h"

// Settings
int real_time_clock_setup = 30;                // Time to wait to set up time of RTC (in seconds)
int initial_wait = 30;                          // Time to wait before start the loop (in seconds)
int measures = 60;                              // Number of measurements to do[1, 59]
int period = 1;                                 // Sampling period (in minutes) [1, 60]
int sample_counter = 1;                         // Counter of measurements
float depth = 3.50;                             // Absolute depth of the device [0.1, 30] (in meters)

// Metadata
String name = "Kdupro01_multispectral";             // Name of the module
String buoy = "Buoy01";                             // Name of the buoy
String country = "Spain";                           // Name of the country
String place = "Barcelona";                         // Text with place of deployment
String maker = "ICM-CSIC";                          // Maker name
String curator = "ICM-CSIC";                        // Curator name
String email = "rodero@icm.csic.es";                // Email of the curator
String sensors = "AS7341";                          // List with name of used sensors "Sensor 1, ..., Sensor n"
String description = "calibration";                 // Description (optional)
String units = "counts, counts, counts, counts";    // Units of the measurements "Unit 1, ..., Unit n"

// Project Metadata
String latitude = "41.383189";                      // Latitude
String longitude = "2.197949";                      // Longitude
String altitude = "0";                              // Altitude  
String ref_coord_system = "WGS84";                  // Reference Coordinate System
String location_source = "GNSS";                    // Source of the Geodesic information
String time_source = "internet time pool";          // Source of the Time information
String processing_level = "0";                      // Defined by manufacturer and described in the reference documentation.
String processing_procedure = "https://git.csic.es/kduino/kdupro-multi";           // Reference to protocols and algorithms describing the steps involved in data processing
String processing_version = "build";                // Version of the data processing software
String processing_revision = "0";                   // Incremental version of the processed data
String calibration_procedure = "https://git.csic.es/kduino/kdupro-multi";          // For calibrated data: documentation describing the calibration procedure. Can be the same as Processing procedure reference
String calibration_reference = "0";                 // Identifier of calibration information
String calibration_time = "0";                      // Date/time stamp of applicable (uncalibrated data, if available) or applied (calibrated data) sensor calibration.
String calibration_version = "0";                   // Version of the calibration processing software

// sensor_id, platform_id, deployment_id, sample_id generated in method "generate_metadata_id"
String sensor_id = "";                              // Unique identifiers used to prevent data duplication with data consumers
String platform_id = "";                            // Platform serial number or randomly assigned identifier (UUID) used with all connected sensors. May be left empty if not applicable.
String deployment_id = "";                          // Randomly assigned identifier (UUID) specific to deployment sequence (e.g. cruise, campaign, vertical profile) of a specific sensor. Not shared with other sensors.
String sample_id = "";                              // Randomly assigned identifier (UUID) generated with each distinct data record from any set of sensors belonging to a single observation.
String observer_id = "";                            // Randomly assigned identifier (UUID) repeated with each data record from this and/or other sensors when operated by a specific observer.
String owner_contact = "jpiera@icm.csic.es";        // An email address where the owner of the data can be contacted now and in future
String operator_contact = "rodero@icm.csic.es";     // An email address where the current operator can be contacted
String license = "MIT License";                     // A licence string or coding that is either self-explanatory or detailed in the License_reference field.
String license_reference = "https://opensource.org/licenses/MIT";           // A reference describing the data license in detail.
String embargo_date = "";                           // A date following which the data may be used according to the specified license. Used, for example, to hide the data record in NRT visualization until quality control is completed.

// Time set up in the serial monitor. Format YYYY-MM-DDThh:mm:ssZ  // Character string formatted according to ISO8601
String datetime = "YYYY-MM-DDThh:mm:ssZ";

// Constants
#define BAUDRATE 9600
#define SD_CS 10        // Chip Select pin for SD card
#define SPI_MOSI 11     // SD Card MOSI
#define SPI_MISO 13     // SD Card MISO
#define SPI_SCK 12      // SD Card Clock
#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds 
#define TIME_TO_SLEEP  300         // Time ESP32 will go to sleep (in seconds) 
#define NUM_LEDS 1        // Define the number of LEDs in the strip (usually 1 for built-in LED)
#define interruptPin GPIO_NUM_15    // Pin for magnetic reed

int led = LED_BUILTIN;  // Red led pin 13 of ESP32-S3

int state; // 0 close - 1 open switch

// Communication protocol
#define UPDATE_RTC 'T'

// Vars
RTC_PCF8523 rtc;
DateTime now;
Adafruit_AS7341 as7341;
String filename;
uint16_t ch415, ch445, ch480, ch515, ch555, ch590, ch630, ch680, clear, nir;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);     // Create an instance of the Adafruit_NeoPixel class
RTC_DATA_ATTR int bootCount = 0;            // var to save in memory despite the deep sleep

// Function declaration
void measure_AS7341();          
void serial_data();              
void serial_metadata();          
void serial_header();
void serial_date();
void save_data();
void save_new_line();
void save_date();
void save_metadata();
void save_header();
void actions();
void update_rtc();

void generate_metadata_id();
void generate_filename();
String get_datetime();

void setup() {

  // Start communication with Serial. Need Wire.begin.
  Wire.begin();
  Serial.begin(BAUDRATE);

  // set LED to be an output pin
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

  // pin for magnetic reed
  pinMode(interruptPin, INPUT_PULLUP);

  #if defined(NEOPIXEL_POWER)
    // If this board has a power control pin, we must set it to output and high
    // in order to enable the NeoPixels. We put this in an #if defined so it can
    // be reused for other boards without compilation errors
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
  #endif

  // Initialize the NeoPixel library
  pixels.begin();

  pixels.fill(0xFF0000);
  pixels.show();
  delay(1000);

  if(!Serial ) {
    delay(1000 );
  }

  delay(10000); // Wait 10 seconds
  Serial.println("\nKduino Pro Multispectral");
  Serial.println("Starting...");
  Serial.println("");
  
  // Set up the RTC
  Serial.println("Setting up the RTC...");
  if (!rtc.begin()) {
      digitalWrite(led, HIGH);
      while (1){
          Serial.println("Couldn't find RTC");
          delay(10000);
      }
  }
  Serial.println("Done.");

  //Update time of RTC
  delay(100);
  Serial.print("Write now the actual date and time (YYYYMMDDhhmmss)");
  update_rtc();

  // SD
  Serial.println("Initializing SD card.");
  SPI.begin();
  pinMode(SD_CS, OUTPUT);                      
  digitalWrite(SD_CS, HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);
  if(!SD.begin(SD_CS)) {
      digitalWrite(led, HIGH);
      while (1){
          Serial.println("Card failed, or not present");
          delay(10000);
      }
  }
  Serial.println("Done.");

  // AS7341 sensor
  Serial.println("Initializing AS7341.");
  if(as7341.begin()) {
      as7341.clearInterruptStatus();
      Serial.println("Done.");
      as7341.setATIME(53);                   // Total integration time = (ATIME [µs] + 1) * (ASTEP [µs] + 1) * 2,78 µs. --> (53+1)*(999+1)*2.78= 150.12 ms
      as7341.setASTEP(999);                  //                        = (100 µs + 1s) * (999 µs + 1s) * 2,78 µs = 2,78 us     
      as7341.setGain(AS7341_GAIN_1X);        // Sensor AS7341 gain
  } else {
      digitalWrite(led, HIGH);
      while (1){
          Serial.println("No AS7341 found");
          delay(10000);
      }
   
  }

  // Initial wait
  Serial.println("Waiting ");
  Serial.print(initial_wait);
  Serial.print(" seconds");
  delay(initial_wait*1000);
  Serial.println("");
  Serial.println(filename);

  // Initial functions generates only 1 time
  generate_metadata_id();           // Generate metadata ids
  generate_filename();              // Set name of the file
  save_metadata();                  // Write metadata into file.txt
  save_header();                    // Write header into file.txt
  serial_metadata();                // Send metadata through serial communication
  serial_header();                  // Send header through serial communication
}

void loop () {
  digitalWrite(led, HIGH);
  // Read time
  now = rtc.now();

  int state = digitalRead(interruptPin);
  if (state == HIGH){
    // no magnetic contact
    digitalWrite(led, LOW); // LED off
  }
  else{
    Serial.print("magnetic contact");
    digitalWrite(led, HIGH);

    rtc_gpio_pullup_en(interruptPin);
    rtc_gpio_pulldown_dis(interruptPin);
    esp_sleep_enable_ext0_wakeup(interruptPin, 0);

    delay(3000);

    // esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  }
  
  // Check if it is time to measure
  if (now.minute() % period == 0){
      if (now.second() == 0){
          // Save time
          save_date();
          serial_date();
          
          // Measurement
          for (int i = 0; i < measures; i++) {
              measure_AS7341();
              save_data();
              serial_data();
          }
          save_new_line();
          // Just to check time
          now = rtc.now();
          serial_date();
      }
  }
  actions();
  delay(500);
}
 
////////////////////////////////////////////////////////////
////// GENERATE METADATA ID AND FILE NAME //////////////////
////////////////////////////////////////////////////////////

void generate_metadata_id(){
    /* update the ids from metadata */
 
    String date = get_datetime().substring(0, 10);
    String time_sample = get_datetime().substring(11, 16);
    String rnd_number = String(random(0xffff), HEX);
 
    sensor_id = WiFi.macAddress();
    sensor_id.replace(":", "");
    platform_id = buoy + "_" + country + "_" + place;
    deployment_id = sensor_id + "_" + name + "_" + depth + "_" + rnd_number;
    sample_id = platform_id + "_" + time_sample;
    observer_id = operator_contact + "_" + date + "_" + time_sample;
  }

  void generate_filename(){
    String date = get_datetime().substring(0, 10);
    date.replace("-", "");           // Replaces scripts to nothing
    filename = "/" + date + "_" + platform_id + "_" + depth + ".txt";
    filename.replace(" ", "_");      // Replace spaces to underscore
    Serial.println("Filename: " + filename);
    Serial.println("");
}

////////////////////////////////////////////////////////////
//////////////////// MEASUREMENTS //////////////////////////
////////////////////////////////////////////////////////////

void measure_AS7341(){

    delay(100);  
    as7341.startReading();
    while (!as7341.checkReadingProgress()) {
        delay(10);  
    }

    ch415 = as7341.getChannel(AS7341_CHANNEL_415nm_F1);
    ch445 = as7341.getChannel(AS7341_CHANNEL_445nm_F2);
    ch480 = as7341.getChannel(AS7341_CHANNEL_480nm_F3);
    ch515 = as7341.getChannel(AS7341_CHANNEL_515nm_F4);
    ch555 = as7341.getChannel(AS7341_CHANNEL_555nm_F5);
    ch590 = as7341.getChannel(AS7341_CHANNEL_590nm_F6);
    ch630 = as7341.getChannel(AS7341_CHANNEL_630nm_F7);
    ch680 = as7341.getChannel(AS7341_CHANNEL_680nm_F8);

    clear = as7341.getChannel(AS7341_CHANNEL_CLEAR);
    nir = as7341.getChannel(AS7341_CHANNEL_NIR);
}

////////////////////////////////////////////////////////////
//////////////////// SERIAL COMMUNICATION //////////////////
////////////////////////////////////////////////////////////

void serial_date(){
  /*It sends the data throught the serial  communication*/
  Serial.println(get_datetime());
}

void serial_header(){
    /*It sends the header info of the datathrought the serial communication*/
    Serial.println("DATA");
    Serial.println("------");
    Serial.println("CH415 CH445 CH480 CH515 CH555 CH590 CH630 CH680 CLEAR NIR");
}

void serial_data(){
  /*It sends the data throught the serial  communication*/
  Serial.print(" ");
  Serial.print(ch415, DEC);
  Serial.print(" ");
  Serial.print(ch445, DEC);
  Serial.print(" ");
  Serial.print(ch480, DEC);
  Serial.print(" ");
  Serial.print(ch515, DEC);
  Serial.print(" ");
  Serial.print(ch555, DEC);
  Serial.print(" ");
  Serial.print(ch590, DEC);
  Serial.print(" ");
  Serial.print(ch630, DEC);
  Serial.print(" ");
  Serial.print(ch680, DEC);
  Serial.print(" ");
  Serial.print(clear, DEC);
  Serial.print(" ");
  Serial.print(nir, DEC);
  Serial.println("");
}

void serial_metadata(){
    /*It sends the metadata info throught the serial  communication*/

    Serial.println("METADATA");
    Serial.println("--------");
    Serial.print("initial_wait: "); Serial.println(initial_wait, DEC);
    Serial.print("measures: "); Serial.println(measures, DEC);
    Serial.print("period: "); Serial.println(period, DEC);
    Serial.print("depth: "); Serial.println(depth);
    Serial.print("sample_counter: "); Serial.println(sample_counter);
    Serial.print("name: "); Serial.println(name);
    Serial.print("buoy: "); Serial.println(buoy);
    Serial.print("place: "); Serial.println(place);
    Serial.print("country: "); Serial.println(country);
    Serial.print("maker: "); Serial.println(maker);
    Serial.print("curator: "); Serial.println(curator);
    Serial.print("email: "); Serial.println(email);
    Serial.print("sensors: "); Serial.println(sensors);
    Serial.print("description: "); Serial.println(description);
    Serial.print("units: "); Serial.println(units);
    Serial.print("latitude: "); Serial.println(latitude);
    Serial.print("longitude: "); Serial.println(longitude);
    Serial.print("altitude: "); Serial.println(altitude);
    Serial.print("ref_coord_system: "); Serial.println(ref_coord_system);
    Serial.print("location_source: "); Serial.println(location_source);
    Serial.print("time_source: "); Serial.println(time_source);
    Serial.print("processing_level: "); Serial.println(processing_level);
    Serial.print("processing_procedure: "); Serial.println(processing_procedure);
    Serial.print("processing_version: "); Serial.println(processing_version);
    Serial.print("processing_revision: "); Serial.println(processing_revision);
    Serial.print("calibration_procedure: "); Serial.println(calibration_procedure);
    Serial.print("calibration_reference: "); Serial.println(calibration_reference);
    Serial.print("calibration_time: "); Serial.println(calibration_time);
    Serial.print("calibration_version: "); Serial.println(calibration_version);
    Serial.print("sensor_id: "); Serial.println(sensor_id);
    Serial.print("platform_id: "); Serial.println(platform_id);
    Serial.print("deployment_id: "); Serial.println(deployment_id);
    Serial.print("sample_id: "); Serial.println(sample_id);
    Serial.print("observer_id: "); Serial.println(observer_id);
    Serial.print("owner_contact: "); Serial.println(owner_contact);
    Serial.print("operator_contact: "); Serial.println(operator_contact);
    Serial.print("license: "); Serial.println(license);
    Serial.print("license_reference: "); Serial.println(license_reference);
    Serial.print("embargo_date: "); Serial.println(embargo_date);
    Serial.println("");
    Serial.print("time: "); Serial.println(get_datetime());
    Serial.println("");
}

////////////////////////////////////////////////////////////
//////////////////// SD MANAGEMENT /////////////////////////
////////////////////////////////////////////////////////////

void save_data(){
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File data_file = SD.open(filename, FILE_APPEND);
  if (! data_file) {
      digitalWrite(LED_BUILTIN, HIGH);
      // Wait forever since we cant write data
      while (1){
          Serial.println("error opening " + filename);
          delay(10000);
      }
  }
  // Save data
  data_file.print(" "); data_file.flush();
  data_file.print(ch415, DEC); data_file.flush();
  data_file.print(" "); data_file.flush();
  data_file.print(ch445, DEC); data_file.flush();
  data_file.print(" "); data_file.flush();
  data_file.print(ch480, DEC); data_file.flush();
  data_file.print(" "); data_file.flush();
  data_file.print(ch515, DEC); data_file.flush();
  data_file.print(" "); data_file.flush();
  data_file.print(ch555, DEC); data_file.flush();
  data_file.print(" "); data_file.flush();
  data_file.print(ch590, DEC); data_file.flush();
  data_file.print(" "); data_file.flush();
  data_file.print(ch630, DEC); data_file.flush();
  data_file.print(" "); data_file.flush();
  data_file.print(ch680, DEC); data_file.flush();
  data_file.print(" "); data_file.flush();
  data_file.print(clear, DEC); data_file.flush();
  data_file.print(" "); data_file.flush();
  data_file.print(nir, DEC); data_file.flush();
  // Close dataFile
  data_file.close();
}

void save_new_line(){
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File data_file = SD.open(filename, FILE_APPEND);
  if (! data_file) {
      digitalWrite(LED_BUILTIN, HIGH);
      // Wait forever since we cant write data
      while (1){
          Serial.println("error opening " + filename);
          delay(10000);
      }
  }
  // Save new line
  data_file.println(""); data_file.flush();
  // Close dataFile
  data_file.close();
}

void save_date(){
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File data_file = SD.open(filename, FILE_APPEND);
  if (! data_file) {
      digitalWrite(LED_BUILTIN, HIGH);
      // Wait forever since we cant write data
      while (1){
          Serial.println("error opening " + filename);
          delay(10000);
      }
  }
  // Save data
  data_file.print(get_datetime()); data_file.flush();

  // Close dataFile
  data_file.close();
}

void save_metadata() {
  // Open the file in append mode
  File data_file = SD.open(filename, FILE_APPEND);
  if (!data_file) {
      digitalWrite(led, HIGH);
      while (1) {
          Serial.println("Error opening " + filename);
          delay(10000);
      }
  }

  // Save metadata
  data_file.println("METADATA");
  data_file.print("initial_wait: "); data_file.println(initial_wait, DEC);
  data_file.print("measures: "); data_file.println(measures, DEC);
  data_file.print("period: "); data_file.println(period, DEC);
  data_file.print("depth: "); data_file.println(depth);
  data_file.print("sample_counter: "); data_file.println(sample_counter);
  data_file.print("name: "); data_file.println(name);
  data_file.print("buoy: "); data_file.println(buoy);
  data_file.print("place: "); data_file.println(place);
  data_file.print("country: "); data_file.println(country);
  data_file.print("maker: "); data_file.println(maker);
  data_file.print("curator: "); data_file.println(curator);
  data_file.print("email: "); data_file.println(email);
  data_file.print("sensors: "); data_file.println(sensors);
  data_file.print("description: "); data_file.println(description);
  data_file.print("units: "); data_file.println(units);
  data_file.print("latitude: "); data_file.println(latitude);
  data_file.print("longitude: "); data_file.println(longitude);
  data_file.print("altitude: "); data_file.println(altitude);
  data_file.print("ref_coord_system: "); data_file.println(ref_coord_system);
  data_file.print("location_source: "); data_file.println(location_source);
  data_file.print("time_source: "); data_file.println(time_source);
  data_file.print("processing_level: "); data_file.println(processing_level);
  data_file.print("processing_procedure: "); data_file.println(processing_procedure);
  data_file.print("processing_version: "); data_file.println(processing_version);
  data_file.print("processing_revision: "); data_file.println(processing_revision);
  data_file.print("calibration_procedure: "); data_file.println(calibration_procedure);
  data_file.print("calibration_reference: "); data_file.println(calibration_reference);
  data_file.print("calibration_time: "); data_file.println(calibration_time);
  data_file.print("calibration_version: "); data_file.println(calibration_version);
  data_file.print("sensor_id: "); data_file.println(sensor_id);
  data_file.print("platform_id: "); data_file.println(platform_id);
  data_file.print("deployment_id: "); data_file.println(deployment_id);
  data_file.print("sample_id: "); data_file.println(sample_id);
  data_file.print("observer_id: "); data_file.println(observer_id);
  data_file.print("owner_contact: "); data_file.println(owner_contact);
  data_file.print("operator_contact: "); data_file.println(operator_contact);
  data_file.print("license: "); data_file.println(license);
  data_file.print("license_reference: "); data_file.println(license_reference);
  data_file.print("embargo_date: "); data_file.println(embargo_date);
  data_file.print("time: "); data_file.println(get_datetime());

  // Close the file (this also forces the write operation)
  data_file.close();
}

void save_header(){
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File data_file = SD.open(filename, FILE_APPEND);
  if (! data_file) {
      digitalWrite(LED_BUILTIN, LOW);
      // Wait forever since we cant write data
      while (1){
          Serial.println("error opening " + filename);
          delay(10000);
      }
  }
  // Save header
  data_file.println("DATA"); data_file.flush();
  data_file.println("TIME CH415 CH445 CH480 CH515 CH555 CH590 CH630 CH680 CLEAR NIR");
  data_file.flush();
  // Close dataFile
  data_file.close();
}

///////////////////////////////////////////
///////////// ACTIONS /////////////////////
///////////////////////////////////////////

void actions() {
  int inByte = 0; // incomming serial byte
  if (Serial.available() > 0) {
      // get incoming byte:
      inByte = Serial.read();
      if (inByte == UPDATE_RTC){
          update_rtc();
      }
  }
}

void update_rtc() {

  int watchdog_time = 100; // To do it not blocking
  int inByte = 0;
  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int minute = 0;
  int second = 0;

  for (int i=0; i<watchdog_time; i++) {
      if (Serial.available() > 13) { // YYYYMMDDhhmmss (14 bytes to receive)
          for (int date_position = 0; date_position < 14; date_position++) {
              // get incoming byte
              inByte = Serial.read();
              switch (date_position) {
                  case 0: year += (inByte - '0')*1000; break;
                  case 1: year += (inByte - '0')*100; break;
                  case 2: year += (inByte - '0')*10; break;
                  case 3: year += (inByte - '0'); break;
                  case 4: month += (inByte - '0')*10; break;
                  case 5: month += (inByte - '0'); break;
                  case 6: day += (inByte - '0')*10; break;
                  case 7: day += (inByte - '0'); break;
                  case 8: hour += (inByte - '0')*10; break;
                  case 9: hour += (inByte - '0'); break;
                  case 10: minute += (inByte - '0')*10; break;
                  case 11: minute += (inByte - '0'); break;
                  case 12: second += (inByte - '0')*10; break;
                  case 13: second += (inByte - '0'); break;
              }
          }
          rtc.adjust(DateTime(year, month, day, hour, minute, second));
          // Read time
          Serial.println("");
          Serial.println("Real Time Clock Updated");
          serial_date();
          delay(3);

          return;
      }
      else delay(real_time_clock_setup*10);
  }
}

String get_datetime() {
  /* Updates datetime string with correct format */

  now = rtc.now();
  char buffer [31] = "";
  sprintf(buffer, "%04d-%02d-%02dT%02d:%02d:%02dZ", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  datetime = String(buffer);

  return datetime;
}

