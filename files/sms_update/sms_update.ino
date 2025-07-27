#include <Wire.h>
#include <Adafruit_BMP085.h>
#include "ThingSpeak.h"
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>

//----------- Enter you Wi-Fi Details---------//
char ssid[] = "Virus#404"; //SSID
char password[] = "password2"; // Password
//-------------------------------------------//

//SMS Update
const char* apiKey = "0fINr30Lk71q";
const char* templateID = "102";
const char* mobileNumber = "916382667630";
const char* var1 = "Muscle Pain Detected!";

WiFiClient  client;

#define SAMPLE_RATE 500
#define INPUT_PIN A0
#define BUFFER_SIZE 128

int circular_buffer[BUFFER_SIZE];
int data_index, sum;

WiFiClient  client;

Adafruit_BMP085 bmp; // I2C

unsigned long Channel_ID = 2844884; // Channel ID
const char * WriteAPIKey = "YFARKCD869VFA86U"; // Your write API Key

float temperature,pressure,altitude; 
int emg_val, emg_env;

void wifi(){
  Serial.print("Connecting to Wi-Fi!\n");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi\n");
}

void sendSMS() {
 //if (WiFi.status() == WL_CONNECTED) {
   //WiFiClientSecure client; // Use WiFiClientSecure for HTTPS connections
   client.setInsecure();    // Skip certificate validation (not secure but works for development)
   HTTPClient http;
   // Build the API URL with the template ID
   String apiUrl = "https://www.circuitdigest.cloud/send_sms?ID=" + String(templateID);
   // Start the HTTPS connection with WiFiClientSecure
   http.begin(client, apiUrl);
   http.addHeader("Authorization", apiKey);
   http.addHeader("Content-Type", "application/json");
   // Create the JSON payload with SMS details
   String payload = "{\"mobiles\":\"" + String(mobileNumber) + "\",\"var1\":\"" + String(var1) + "\",\"var2\":\"" + var2 + "\"}";
   // Send POST request
   int httpResponseCode = http.POST(payload);
   // Check response
   if (httpResponseCode == 200) {
     Serial.println("SMS sent successfully!");
     Serial.println(http.getString());
   } else {
     Serial.print("Failed to send SMS. Error code: ");
     Serial.println(httpResponseCode);
     Serial.println("Response: " + http.getString());
   }
   http.end(); // End connection
}


void thing(){
  ThingSpeak.setField(1, temperature);
  ThingSpeak.setField(2, emg_val);
  ThingSpeak.setField(3, emg_env);
  ThingSpeak.writeFields(Channel_ID, WriteAPIKey);
}

void emg() {
  static unsigned long past = 0;
  unsigned long present = micros();
  unsigned long interval = present - past;
  past = present;

  // Run timer
  static long timer = 0;
  timer -= interval;

  // Sample and get envelop
  if(timer < 0) {
    timer += 1000000 / SAMPLE_RATE;
    int sensor_value = analogRead(INPUT_PIN);
    emg_val = EMGFilter(sensor_value);
    emg_env = getEnvelop(abs(emg_val));
  }
}

int getEnvelop(int abs_emg){
  sum -= circular_buffer[data_index];
  sum += abs_emg;
  circular_buffer[data_index] = abs_emg;
  data_index = (data_index + 1) % BUFFER_SIZE;
  return (sum/BUFFER_SIZE) * 2;
}

float EMGFilter(float input){
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - 0.05159732*z1 - 0.36347401*z2;
    output = 0.01856301*x + 0.03712602*z1 + 0.01856301*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -0.53945795*z1 - 0.39764934*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - 0.47319594*z1 - 0.70744137*z2;
    output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.00211112*z1 - 0.74520226*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

void transmit(){
    temperature = bmp.readTemperature();
    emg();

    Serial.print("x");
    Serial.print((int)temperature);
    Serial.print("y");
    Serial.print((int)emg_val);    
    Serial.print("z");
    Serial.print((int)emg_env);
    Serial.print("o");
    Serial.println();
    
    thing();
}

void setup() {
  Serial.begin(115200);

  while (!Serial) delay(100);

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP180 sensor, check wiring!"));
    while (1);
  }

  wifi();
  ThingSpeak.begin(client);
}

void loop() {
  transmit();
}
