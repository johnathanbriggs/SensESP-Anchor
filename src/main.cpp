// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <EEPROM.h>

#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp_app_builder.h"

using namespace sensesp;

#define CLK 26               // DATA signal
#define DT 25                // CLOCK signal
#define RESET_BUTTON_PIN 33  // Pin for the reset button
float chainLength = 50.0;    // Total length of chain in meters
int encoderTicksPerMeter =
    106;  // Number of encoder ticks per meter of chain deployed
int counter = 0;
int aState;
int aLastState;
int address = 0;
bool shouldWriteToFlash = false;
bool runOnce = false;
float chainDeployed;
unsigned long lastEncoderMoveTime = 0;

SKOutput<float>* chainLength_output;
SKOutput<float>* rotations_output;
reactesp::ReactESP app;

// functions

void resetChain() {  // Reset the chain position to 0
  counter = 0;
  EEPROM.begin(512);  // initialize EEPROM library
  EEPROM.put(address, counter);
  EEPROM.commit();  // commit the changes to flash memory
  EEPROM.end();     // release the EEPROM
  chainLength_output->set_input(chainDeployed);
}
void writeToFlash() {
  // Save the current position to flash memory
  EEPROM.begin(512);  // initialize EEPROM library
  EEPROM.put(address, counter);
  EEPROM.commit();  // commit the changes to flash memory
  EEPROM.end();     // release the EEPROM
  Serial.print("##############FLASH WRITE ~~~~~~~~~~~~");
}

void checkChain() {
  if (! runOnce) { // send the initial value once only
    chainLength_output->set_input(chainDeployed);
    runOnce=true;
  }
  aState = digitalRead(DT);  // Reads the "current" state of the outputA
  // If the previous and the current state of the outputA are different, that
  // means a Pulse has occured
  if (aState != aLastState) {
    // If the outputB state is different to the outputA state, that means the
    // encoder is rotating clockwise
    if (digitalRead(CLK) != aState) {
      counter--;
    } else {
      counter++;
    }
    Serial.print("Encoder position: ");
    Serial.println(counter);
    // Calculate the amount of chain deployed based on the number of encoder
    // ticks
    chainDeployed = (float)counter / encoderTicksPerMeter;
    Serial.print("Chain deployed: ");
    Serial.print(chainDeployed);
    chainLength_output->set_input(chainDeployed);
    Serial.print("m out of ");
    Serial.print(chainLength);
    Serial.println("m");
    shouldWriteToFlash = true;
    lastEncoderMoveTime = millis();
  }
  aLastState = aState;  // Updates the previous state of the outputA with the
                        // current state

  if (shouldWriteToFlash && (millis() - lastEncoderMoveTime >=
                             5000)) {  // only write to the flash memoory 5sedc
                                       // after the last rotation
    writeToFlash();
    shouldWriteToFlash = false;
  }
}
// end functions

// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("sensesp-anchor")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();

  // Read last saved position from flash memory
  EEPROM.begin(512);  // initialize EEPROM library
  EEPROM.get(address, counter);
  EEPROM.end();  // release the EEPROM
  delay(500);
  pinMode(DT, INPUT_PULLUP);
  pinMode(CLK, INPUT_PULLUP);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RESET_BUTTON_PIN), resetChain, FALLING);
  Serial.begin(115200);
  // Reads the initial state of the outputA
  aLastState = digitalRead(DT);
  chainDeployed = (float)counter / encoderTicksPerMeter;
  // Serial.print("Chain deployed: ");
  // Serial.print(chainDeployed);
  // chainLength_output->set_input(chainDeployed);

  chainLength_output = new SKOutput<float>(
      "navigation.anchor.rodeDeployed", "/navigation/anchor/rodeDeployed",
      new SKMetadata("M", "Anchor Chain Deployed"));
  sensesp_app->start();
}

void loop() {
  checkChain();
  app.tick();
}
