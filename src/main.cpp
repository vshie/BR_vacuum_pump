#include <Arduino.h>
#include <Wire.h>
#include <MS5837.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include "brlogo.h"  // Include the logo header file

// Acebott ESP32-MAX-V1.0 I2C pins
#define I2C_SDA 21  // H13
#define I2C_SCL 22  // H14

// LED pin
#define LED_PIN 2   // D2 LED on board

// Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Create sensor and display instances
MS5837 sensor;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Store initial pressure reading
float initialPressure = 0.0;

// Logo rotation variables
unsigned long lastRotationTime = 0;
int currentRotation = 0;
const int ROTATION_INTERVAL = 2000; // 2 seconds
const int ROTATION_STEP = 45; // 45 degree increments

// Function to rotate a bitmap by a given angle
void rotateBitmap(const uint8_t* src, uint8_t* dst, int width, int height, int angle) {
  // Clear destination bitmap
  memset(dst, 0, (width * height + 7) / 8);
  
  // Convert angle to radians
  float rad = angle * PI / 180.0;
  
  // Calculate center point
  int centerX = width / 2;
  int centerY = height / 2;
  
  // Rotate each pixel
  for(int y = 0; y < height; y++) {
    for(int x = 0; x < width; x++) {
      // Calculate rotated coordinates
      int dx = x - centerX;
      int dy = y - centerY;
      
      int newX = centerX + (dx * cos(rad) - dy * sin(rad));
      int newY = centerY + (dx * sin(rad) + dy * cos(rad));
      
      // Check if new coordinates are within bounds
      if(newX >= 0 && newX < width && newY >= 0 && newY < height) {
        // Get source pixel
        int srcByte = (y * width + x) / 8;
        int srcBit = 7 - ((y * width + x) % 8);
        bool srcPixel = (src[srcByte] >> srcBit) & 1;
        
        // Set destination pixel
        if(srcPixel) {
          int dstByte = (newY * width + newX) / 8;
          int dstBit = 7 - ((newY * width + newX) % 8);
          dst[dstByte] |= (1 << dstBit);
        }
      }
    }
  }
}

const char* getI2CErrorString(byte error) {
  switch(error) {
    case 0: return "Success";
    case 1: return "Data too long";
    case 2: return "NACK on transmit of address";
    case 3: return "NACK on transmit of data";
    case 4: return "Other error";
    default: return "Unknown error";
  }
}

void checkPinState(int pin, const char* name) {
  pinMode(pin, INPUT);
  int state = digitalRead(pin);
  Serial.printf("Pin %s (%d) state: %s\n", name, pin, state ? "HIGH" : "LOW");
}

void setupI2CPins() {
  // Configure SDA and SCL pins with internal pull-ups
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  
  // Check initial states
  Serial.println("\nChecking I2C pin states before Wire.begin():");
  checkPinState(I2C_SDA, "I2C_SDA");
  checkPinState(I2C_SCL, "I2C_SCL");
}

void blinkLED(int pin, int count) {
  pinMode(pin, OUTPUT);
  for(int i = 0; i < count; i++) {
    digitalWrite(pin, HIGH);
    delay(500);
    digitalWrite(pin, LOW);
    delay(500);
  }
}

void scanI2CDevices() {
  Serial.println("\nScanning I2C bus...");
  byte error, address;
  int deviceCount = 0;
  
  // Check pin states
  checkPinState(I2C_SDA, "I2C_SDA");
  checkPinState(I2C_SCL, "I2C_SCL");
  
  Serial.println("\nFull I2C bus scan:");
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.printf("I2C device found at address 0x%02X\n", address);
      deviceCount++;
    }
    delay(5); // Increased delay between scans for stability
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found!");
  } else {
    Serial.printf("Found %d device(s)\n", deviceCount);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  
  // Blink LED to indicate program start
  Serial.println("\nBlinking LED to verify program execution...");
  blinkLED(LED_PIN, 5); // Blink 5 times
  
  Serial.println("Starting MS5837-02BA Pressure Sensor Test...");
  
  // Setup I2C pins with pull-ups
  setupI2CPins();
  
  // Initialize I2C with an extremely low clock speed
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(10000); // Set to 10kHz (extremely slow for maximum stability)
  Serial.printf("I2C initialized on SDA:%d (H13), SCL:%d (H14) at 10kHz\n", I2C_SDA, I2C_SCL);
  delay(200); // Give I2C more time to stabilize
  
  // Initialize SSD1306 display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();
  
  // Scan for I2C devices
  scanI2CDevices();
  
  // Initialize MS5837 sensor with retries
  Serial.println("\nInitializing MS5837-02BA sensor...");
  bool initSuccess = false;
  for(int i = 0; i < 3; i++) {
    if (sensor.init()) {
      initSuccess = true;
      Serial.println("Sensor initialized successfully!");
      break;
    }
    Serial.printf("Initialization attempt %d failed, retrying...\n", i + 1);
    delay(100);
  }
  
  if (!initSuccess) {
    Serial.println("Failed to initialize MS5837 sensor after 3 attempts!");
    Serial.println("Please check:");
    Serial.println("1. I2C connections (H13=SDA, H14=SCL)");
    Serial.println("2. Sensor power supply");
    Serial.println("3. I2C clock signal integrity");
    // Blink LED rapidly to indicate error
    while(1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  
  // Set model of MS5837 sensor to 02BA (air pressure)
  sensor.setModel(MS5837::MS5837_02BA);
  sensor.setFluidDensity(1.225); // kg/m^3 (air density at sea level)
  
  // Get initial pressure reading
  sensor.read();
  initialPressure = sensor.pressure();
  Serial.printf("Initial pressure: %.2f mbar\n", initialPressure);
  
  Serial.println("MS5837-02BA sensor initialized successfully!");
  
  // Blink LED to indicate successful completion
  blinkLED(LED_PIN, 3);
}

void loop() {
  // Read sensor
  sensor.read();
  
  // Convert pressure from mbar to PSI (1 mbar = 0.0145038 PSI)
  float differentialPressurePSI = (initialPressure - sensor.pressure()) * 0.0145038;
  
  // Update logo rotation if pressure is non-zero
  if (abs(differentialPressurePSI) > 0.05) {
    unsigned long currentTime = millis();
    if (currentTime - lastRotationTime >= ROTATION_INTERVAL) {
      currentRotation = (currentRotation + ROTATION_STEP) % 360;
      lastRotationTime = currentTime;
    }
  } else {
    currentRotation = 0;
  }
  
  // Print readings to Serial
  Serial.print("Pressure: ");
  Serial.print(sensor.pressure() * 0.0145038);
  Serial.println(" PSI");
  
  Serial.print("Differential: ");
  Serial.print(differentialPressurePSI);
  Serial.println(" PSI");
  
  Serial.print("Temperature: ");
  Serial.print(sensor.temperature());
  Serial.println(" deg C");
  
  Serial.println("-------------------");
  
  // Update display
  display.clearDisplay();
  
  // Set up for pressure value
  display.setFont(&FreeSansBold9pt7b);
  
  // Calculate number of digits in the pressure value
  float absDiff = abs(differentialPressurePSI);
  int numDigits = 1;  // Start at 1 for the decimal point
  if (absDiff >= 10.0) numDigits++;
  if (absDiff >= 100.0) numDigits++;
  
  // Adjust text size based on number of digits
  if (numDigits > 1) {
    display.setTextSize(2);  // Smaller size for 2+ digits
  } else {
    display.setTextSize(3);  // Larger size for 1 digit
  }
  
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 35);
  
  // Show negative sign and value only when pressure is significantly below ambient
  if (sensor.pressure() < initialPressure) {
    if (absDiff > 0.1) {
      // Use a more compact negative sign by drawing a line
      display.drawLine(0, 19, 8, 19, SSD1306_WHITE);  // Shifted up by 4 pixels from previous position (23-4=19)
      display.setCursor(10, 35);  // Move cursor after the line
      display.print(absDiff, 1);
    } else {
      display.print("0.0");
    }
  } else {
    display.print("0.0");
  }
  
  // Set up for PSI text
  display.setFont(&FreeSansBold9pt7b);
  display.setTextSize(1.5);
  display.setCursor(0, 63);  // Moved PSI text to bottom of screen
  display.print("PSI");
  
  // Create rotated version of logo
  uint8_t rotatedLogo[128]; // 32x32 = 1024 bits = 128 bytes
  rotateBitmap(br_logo, rotatedLogo, 32, 32, currentRotation);
  
  // Draw rotated logo
  display.drawBitmap(96, 32, rotatedLogo, 32, 32, SSD1306_WHITE);
  
  display.display();
  
  // Blink LED to show we're running
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}