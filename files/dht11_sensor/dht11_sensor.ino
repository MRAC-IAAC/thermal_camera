// Install library DHT sensor library by Adafruit
#include <DHT.h> // This includes the library for the sensor we use

#define DHTPIN 2        // Pin connected to the DHT11
#define DHTTYPE DHT11   // DHT11 sensor type

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600); // Initialize serial communication
  dht.begin();        // Initialize the DHT sensor
}

void loop() {
  int humidity = dht.readHumidity(); // On purpose as int
  int temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Error reading from DHT sensor!");
    return;
  }

  // Send temperature and humidity as separate lines
  Serial.println("The temperature is:");
  Serial.println(temperature); // Print temperature 
  Serial.println("The humidity is:");
  Serial.println(humidity);    // Print humidity

  delay(10000); // Wait for 10 seconds
}
