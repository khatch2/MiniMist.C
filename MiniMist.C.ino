#include "Arduino.h"
#include "esp_adc_cal.h"
/*!< WIFI and Asyncronous Server Library */
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
/*!< Camera Library */
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
/*!< Libraries to Disable Brownouts */
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
/*!< DHT11 and JSON Library */
#include <DHT.h>
#include "Arduino_JSON.h"
/*!< SPIFFS for FileSystem*/
#include "SPIFFS.h"
/*!< Buzzer module pitches*/
#include "pitches.h"

#define DHTTYPE DHT11 /*!< Define the type of DHT sensor used*/

/*!< Configure Pins for the ESP32 Camera and set counter to track consucutive capture fails*/
#define CAMERA_MODEL_WROVER_KIT
#if defined(CAMERA_MODEL_WROVER_KIT)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 21
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 19
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 5
#define Y2_GPIO_NUM 4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#else
#error "Camera model not selected"
#endif

//int camera_counter = 0;

/*!< Configure Indicator Leds*/
const int led1 = 12;
const int led2 = 2;
const int led3 = 15;

/*!< Assign Pin numbers allocated for reading Solar Panel, Battery, and DHT11*/
const int solar_volt = 32;
const int batt_volt = 33;
const int sensor_dht = 13;

/*!< Assign Pin numbers for buzzer and configures the frequency, pwm channel and duty cycle()*/
const int buzzer = 14;
int freq = 2000;
int channel = 6;
int resolution_buzz = 10;

/*!< Initialize DHT11 Sensor */
DHT dht(sensor_dht, DHTTYPE); 

/*!< Start Server at port 80. Set ssid, password and hostname for the device when connecting to router*/
AsyncWebServer server(80); 
const char *ssid = "Kocin";
const char *password = "kocin555";
String hostname = "minimist-thesis";


String response;
JSONVar solar_readings;
JSONVar sensor_readings;
#define FILE_PHOTO "/photo.jpg"

/**
 * @brief Connecting ESP32 to Wifi network.
 *
 */
void init_wifi()
{
  // delete old config
//  WiFi.disconnect(true);

  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname.c_str());
  WiFi.begin(ssid, password);
  float startTime = millis();
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED)
  {
    if (millis() - startTime > 10000) // Timeout in 10s
    {
      WiFi.disconnect(true);
      Serial.println("\n  Cound't connect to local Wifi network. Check Wifi credentials and Wifi Network status.");
      esp_restart();
      break;
    }
    else
    {
      delay(250);
      Serial.print(".");
    }
  }
  if (WiFi.status() == WL_CONNECTED){
  Serial.println("WiFi connected.");
  }
  // Serial.println(WiFi.localIP());
  get_network_info();
}

/**
 * @brief Get the Network information of ESP32.
 *
 */
void get_network_info()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("[*] Network information for ");
    Serial.println(WiFi.SSID());
    Serial.print("[+] ESP32 IP Addr: ");
    Serial.println(WiFi.localIP());
    Serial.print("[+] ESP32 Hostname : ");
    Serial.println(WiFi.getHostname());
    Serial.print("[+] Subnet Mask : ");
    Serial.println(WiFi.subnetMask());
    Serial.print("[+] Gateway IP : ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("[+] DNS : ");
    Serial.println(WiFi.dnsIP());
    Serial.println("[+] BSSID : " + WiFi.BSSIDstr());
    Serial.println((String) "[+] RSSI : " + WiFi.RSSI() + " dB");
    delay(20);
  }
}

/**
 * @brief Configure Camera Pins and Initiate Camera.
 *
 */
void init_cam()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  Serial.println("Camera configuration complete!");
}

void deinit_cam()
{
}
/**
 * @brief Initate SPIFFS. Restart ESP32 if failed.
 *
 */
void init_spiffs()
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    ESP.restart();
  }
  else
  {
    Serial.println("SPIFFS Init Success!");
  }
}

void readFiles(){
 File root = SPIFFS.open("/");
 
 File file = root.openNextFile();
 
 while(file){
  Serial.print("FILE: ");
  Serial.println(file.name());
  file = root.openNextFile();
  }  
    Serial.print("FREE HEAP SPACE: ");
  Serial.println(ESP.getFreeHeap());
}

/**
 * @brief Reads Humidity, Temperature and Hall readings from DHT11 sensor and inbuilt Hallsensor.
 * The DHT11 sensor is placed in Pin 13.
 * Pin 36 & 39 are connected to the hall effect sensor
 *
 * @return JSONString:
 *  -humidity Humidity
 *  -tempratureC Temperature in Celsius
 *  -tempratureF Temperature in Fahrenheit
 */
String get_sensor()
{
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h, t, f;
  h = dht.readHumidity();        // Read humidity
  t = dht.readTemperature();     // Read temperature as Celsius (the default)
  f = dht.readTemperature(true); // Read temperature as Fahrenheit (isFahrenheit = true)

  if (isnan(h) || isnan(t) || isnan(f))
  {
    Serial.println(F("Failed to read from DHT sensor!"));
    return "";
  }
  sensor_readings["humidity"] = String(h);
  sensor_readings["temperatureC"] = String(t);
  sensor_readings["temperatureF"] = String(f);
  // sensor_readings["hall"] = String(hallRead()); // Read Hall sensor value
  // sensor_readings["heatindexF"] = dht.computeHeatIndex(f , h); // Compute heat index in Fahrenheit (the default)
  // sensor_readings["heatindexC"] = dht.computeHeatIndex(t , h, false);  // Compute heat index in Celsius (isFahreheit = false)

  String stringsensor = JSON.stringify(sensor_readings);
  Serial.println(stringsensor);
  return stringsensor;
}

/**
 * @brief Get Voltage reading from solar panels and battery.
 * Convert the analog reading to voltage.
 *
 * @return JSONString:
 *  -voltage_solar Voltage received from Solar Panels
 *  -current_solar Current received from Solar Panels
 *  -power_solar   Power received from Solar Panels
 *  -voltage_batt  Voltage remaining in Battery
 */
String get_solar()
{
  /*!< Calibrating the Analog voltage read*/
  float calibration = 1.000;
  float vref = 1100;
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  vref = adc_chars.vref;

  float readv = analogRead(solar_volt);
  delay(100);
  float readb = analogRead(batt_volt);

  float vout = (readv > 12) ? (readv * (3.3 / 4095.0) * (1100 / vref) * calibration) + 0.13 : (readv * (3.3 / 4095.0) * (1100 / vref) * calibration);
  float volt_batt = (readb > 12) ? (readb * (3.3 / 4095.0) * (1100 / vref) * calibration) + 0.13 : (readb * (3.3 / 4095.0) * (1100 / vref) * calibration);
  float vin = (vout * ((21600 + 8170) / 8170));
  float cur = (vin * 1000) / (21600 + 8100);

  float battout = (volt_batt * 2 * 100) / 4.794;
  // Serial.print(vout);
  // Serial.print(" ");
  // Serial.print(volt_batt);
  // Serial.print(" ");
  // Serial.println(battout);

  solar_readings["solar_voltage"] = String(vin);
  solar_readings["solar_current"] = String(cur);
  solar_readings["solar_power"] = String(vin * cur);
  solar_readings["battery_voltage"] = String(battout);

  String stringsolar = JSON.stringify(solar_readings);
  Serial.println(stringsolar);
  return stringsolar;
}

/**
 * @brief Check if photo exist in the FileSystem at location FILE_PHOTO
 * @attention This function is not used
 * @param fs
 * @return true
 * @return false
 */
//bool checkPhoto(fs::FS &fs)
//{
//  File f_pic = fs.open(FILE_PHOTO);
//  unsigned int pic_sz = f_pic.size();
//  return (pic_sz > 100);
//}

/**
 * @brief Campute image using ESP32-Camera and store it in the /photo.jpg location in the file system.
 *
 * @return bool if the image was stored succesfully.
 */
//bool storeimage()
//{
//  camera_fb_t *fb = NULL; // pointer
//  bool ok = 0;            // Boolean indicating if the picture has been taken correctly
//
//  do
//  {
//    // Take a photo with the camera
//    Serial.println("Taking a photo...");
//
//    fb = esp_camera_fb_get();
//    if (!fb)
//    {
//      Serial.println("Camera capture failed");
//      return "";
//    }
//
//    // Photo file name
//    Serial.printf("Picture file name: %s\n", FILE_PHOTO);
//    File file = SPIFFS.open(FILE_PHOTO, FILE_WRITE);
//
//    // Insert the data in the photo file
//    if (!file)
//    {
//      Serial.println("Failed to open file in writing mode");
//    }
//    else
//    {
//      file.write(fb->buf, fb->len); // payload (image), payload length
//      Serial.print("The picture has been saved in ");
//      Serial.print(FILE_PHOTO);
//      Serial.print(" - Size: ");
//      Serial.print(file.size());
//      Serial.println(" bytes");
//    }
//    // Close the file
//    file.close();
//    esp_camera_fb_return(fb);
//
//    // check if file has been correctly saved in SPIFFS
//    ok = checkPhoto(SPIFFS);
//  } while (!ok);
//  return ok;
//}

/**
 * Blink function is used to blink an led with delay.
 * 
 * @param led: The LED intended to blink. Value {led1, led2, led3}
 *        val: ON or OFF. Value {1,0} = {HIGH, LOW} = (ON, OFF);
 */
void blink(int led, int val)
{
  digitalWrite(led, val);
  delay(150);
}

/**
 * Play funciton takes a melody and plays in on the buzzer.
 * 
 * @param buzzer_pin: The pin designinated to the buzzer in ESP32
 *        melodies[]: An Array of Melodies to play 
 *        melody_len: Length of the melodies array
 *        buzz_delay: The after each note
 */
void play(int buzzer_pin,int melodies[],int melody_len ,int buzz_delay){
  ledcAttachPin(buzzer_pin, channel);
    for (int i = 0; i < melody_len; i++){
      ledcWriteTone(channel, melodies[i]);
//      Serial.println(melodies[i]);
      delay(buzz_delay);
    }
   ledcWriteTone(channel, 0);
}

/**
 * notify1 uses the play() function to play a melody on the buzzer
 */
void notify1(){
  int melody[] = {NOTE_A4, NOTE_A4};
  int melody_len = sizeof (melody) / sizeof(int);
//  Serial.println(melody_len);
  int buzz_delay = 100;
  play(buzzer, melody, melody_len ,buzz_delay);
}
/**
 * notify2 uses the play() function to play a melody on the buzzer
 */
void notify2(){
  int melody[] = {NOTE_A4, NOTE_E4};
  int melody_len = sizeof (melody) / sizeof(int);
//  Serial.println(melody_len);
  int buzz_delay = 150;
  play(buzzer, melody, melody_len ,buzz_delay);
}
/**
 * notify3 uses the play() function to play a melody on the buzzer
 */
void notify3(){
  int melody[] = {NOTE_A4, NOTE_G7};
  int melody_len = sizeof (melody) / sizeof(int);
//  Serial.println(melody_len);
  int buzz_delay = 100;
  play(buzzer, melody, melody_len ,buzz_delay);
}

void setup()
{
  // initialize the Serial Monitor
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); /*!< Disable Brownout*/
  
  //  Initializing the Pins
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);

  // Initializing the Buzzer Pin
//  pinMode (buzzer, OUTPUT);
  ledcSetup(channel, freq, resolution_buzz);

  init_spiffs();
  dht.begin();
  init_cam();
  
  // Initiate Wifi Connection to the network
  init_wifi();
  

  // Configure routes on incoming HTTP request and corresponding functions.
  /*!< INDEX Page Route*/
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    notify1();    
    request->send(SPIFFS, "/MiniMist.html", String(), false); });
 
  /*!< Solar Panel data Route*/
  server.on("/getsolar", HTTP_GET, [](AsyncWebServerRequest *request){
    notify1();    
    String response_solar = get_solar();
    request->send(200,"application/json",response_solar); });

  /*!< Sensor data Route*/
  server.on("/getsensor", HTTP_GET, [](AsyncWebServerRequest *request){
    notify1();    
    String response_sensor = get_sensor();
    request->send(200,"application/json",response_sensor);
    response_sensor = String(); 
  });

  /*!< Camera Image data Route*/
  server.on("/getimage", HTTP_GET, [](AsyncWebServerRequest *request){
    camera_fb_t * fb = NULL;
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      request->send(500);
      notify3();
    }
    size_t size = fb->len;
    request->send_P(200,"image/jpeg",(const uint8_t *)fb->buf,size);
    esp_camera_fb_return(fb);
  });

  server.begin();
}

void loop()
{
}
