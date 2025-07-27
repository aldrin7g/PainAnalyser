#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "ThingSpeak.h"
#include <ESP8266WiFi.h>

//----------- Enter you Wi-Fi Details---------//
char ssid[] = "Aby's A73"; //SSID
char password[] = "1234567890"; // Password
//-------------------------------------------//

WiFiClient  client;

Adafruit_BMP280 bmp; // I2C

unsigned long Channel_ID = 2844884; // Channel ID
const char * WriteAPIKey = "YFARKCD869VFA86U"; // Your write API Key

float temperature,pressure,altitude; 
int value1, value2;

String receivedData = ""; // Buffer to store received data
int receivedValue = 0; // Store extracted values
bool isReceiving = false;
char identifier = '\0'; // Stores the current identifier ('a' or 'b')

void wifi(){
  Serial.print("Connecting to Wi-Fi!\n");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi\n");
}

void thing(){
  ThingSpeak.writeField(Channel_ID, 1, temperature, WriteAPIKey);
  ThingSpeak.writeField(Channel_ID, 2, pressure, WriteAPIKey);
  ThingSpeak.writeField(Channel_ID, 3, altitude, WriteAPIKey);
  ThingSpeak.writeField(Channel_ID, 4, value1, WriteAPIKey);
  ThingSpeak.writeField(Channel_ID, 5, value2, WriteAPIKey);
}

void receive() {
    while (Serial.available() > 0) { // Check if data is available
        char receivedChar = Serial.read(); // Read one character

        if (receivedChar == 'a' || receivedChar == 'b') {
            // Store previous received value before switching
            if (identifier == 'a') {
                value1 = receivedValue; // Store in temperature
            } else if (identifier == 'b') {
                value2 = receivedValue; // Store in humidity
            }

            // Reset receivedValue for new number and update identifier
            receivedValue = 0;
            identifier = receivedChar;
            isReceiving = true;
        } 
        else if (receivedChar >= '0' && receivedChar <= '9') { 
            receivedValue = receivedValue * 10 + (receivedChar - '0'); // Convert char to integer
        } 
        else if (receivedChar == '\n' || receivedChar == '\r') { // Enter key pressed
            if (isReceiving) { // If we received a valid number
                // Store last received value based on identifier
                if (identifier == 'a') {
                    value1 = receivedValue;
                } else if (identifier == 'b') {
                    value2 = receivedValue;
                }

                // // Print the values to confirm correct extraction
                // Serial.print("\nTemperature: ");
                // Serial.println(value1);
                // Serial.print("Humidity: ");
                // Serial.println(value2);

                thing();

                // Reset variables for next input
                receivedValue = 0;
                isReceiving = false;
                identifier = '\0';
            }
        }
    }
}

void transmit(){
    temperature = bmp.readTemperature();
    pressure = bmp.readPressure()/100.0F;
    altitude = bmp.readAltitude(1013.25);

    Serial.print("x");
    Serial.print((int)temperature);
    Serial.print("y");
    Serial.print((int)pressure);
    Serial.print("z");
    Serial.print((int)altitude);
    Serial.println();

    thing();
    delay(500);
}

void test(){
      while (Serial.available() > 0) {  // Check if data is available
        char receivedChar = Serial.read();  // Read a character
        if (receivedChar == '\n' || receivedChar == '\r') {  // Newline or carriage return
            Serial.print("Received: ");  
            Serial.println(receivedData); // Print the full received data
            receivedData = ""; // Reset buffer for the next message
        } else {
            receivedData += receivedChar; // Append character to the buffer
        }
    }
}

void setup() {
  Serial.begin(115200); // Initialize Serial at 115200 baud rate

  while ( !Serial ) delay(100);   // wait for native usb

  unsigned status;

  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring"));
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,    /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,   /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,     /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  wifi();
  ThingSpeak.begin(client);
}

void loop() {
  transmit();
  receive();
}
