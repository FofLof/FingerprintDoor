/**
 * 1. Setup button and servo
 * 2. Fill out each state
 *     a. StandBy --> Not reading finger print and door is closed
 *                    switches to reading print if button pressed
 *     b. Reading_Print --> Read fingerprint and if is valid then switch
 *                          to open door. If no valid print found in 10 seconds
 *                          go back to standby
 *     c. Open_door --> Open door for 5 seconds, after 5 seconds go back to stand by
*/

/**
 * How to change the state
 * 
 * doorState = DESIREDSTATE
 * example: doorState = READING_PRINT
*/
#include <Adafruit_Fingerprint.h>
#include <Servo.h>


#if (defined(__AVR__) || defined(ESP8266)) && !defined(__AVR_ATmega2560__)
// For UNO and others without hardware serial, we must use software serial...
// pin #2 is IN from sensor (GREEN wire)
// pin #3 is OUT from arduino  (WHITE wire)
// Set up the serial port to use softwareserial..
SoftwareSerial mySerial(2, 3);

#else
// On Leonardo/M0/etc, others with hardware serial, use hardware serial!
// #0 is green wire, #1 is white
#define mySerial Serial1

#endif


Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

bool isValidFingerprintID();

const int BUTTON_PORT = 4;
const int SERVO_PORT = 5;

const int HANDLE_DOWN_POSITION = 0;
const int HANDLE_UP_POSITION = 180;

double time_initial = 0;

enum DoorState{READING_PRINT, STANDBY, OPEN_DOOR};
DoorState doorState = STANDBY;
Servo myservo; 
void setup()
{
  Serial.begin(9600);
  while (!Serial);  // For Yun/Leo/Micro/Zero/...
  delay(100);
  Serial.println("\n\nAdafruit finger detect test");
  pinMode(BUTTON_PORT, INPUT);
  myservo.attach(SERVO_PORT);

  // set the data rate for the sensor serial port
  finger.begin(57600);
  delay(5);
  if (finger.verifyPassword()) {
    Serial.println("Found fingerprint sensor!");
  } else {
    Serial.println("Did not find fingerprint sensor :(");
    while (1) { delay(1); }
  }

  Serial.println(F("Reading sensor parameters"));
  finger.getParameters();
  Serial.print(F("Status: 0x")); Serial.println(finger.status_reg, HEX);
  Serial.print(F("Sys ID: 0x")); Serial.println(finger.system_id, HEX);
  Serial.print(F("Capacity: ")); Serial.println(finger.capacity);
  Serial.print(F("Security level: ")); Serial.println(finger.security_level);
  Serial.print(F("Device address: ")); Serial.println(finger.device_addr, HEX);
  Serial.print(F("Packet len: ")); Serial.println(finger.packet_len);
  Serial.print(F("Baud rate: ")); Serial.println(finger.baud_rate);

  finger.getTemplateCount();

  if (finger.templateCount == 0) {
    Serial.print("Sensor doesn't contain any fingerprint data. Please run the 'enroll' example.");
  }
  else {
    Serial.println("Waiting for valid finger...");
      Serial.print("Sensor contains "); Serial.print(finger.templateCount); Serial.println(" templates");
  }
}

void loop()                     // run over and over again
{
  double time_final = millis() / 1000;
  double delta_time = time_final - time_initial;
  switch (doorState) {
    case STANDBY:
      if (digitalRead(BUTTON_PORT) == HIGH) {
        time_initial = millis() / 1000;
        doorState = READING_PRINT;
      } else {
        myservo.write(HANDLE_UP_POSITION);
      }


      break;
    case READING_PRINT:
      if (isValidFingerprintID() == true) {
        time_initial = millis() / 1000;
        doorState = OPEN_DOOR;
      }
      
      if (delta_time > 10) {
        doorState = STANDBY;
      }

      
      break;

    case OPEN_DOOR:
      myservo.write(HANDLE_DOWN_POSITION);
      if (delta_time > 5) {
        doorState = STANDBY;
      }

      break;
    default:
      Serial.println("Error State");
  }
  delay(50);            //don't ned to run this at full speed.
}

/**
 * Checks if a valid fingerprint has been given
*/
bool isValidFingerprintID() {
  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println("No finger detected");
      return false;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return false;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      return false;
    default:
      Serial.println("Unknown error");
      return false;
  }

  // OK success!

  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return false;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return false;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return false;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return false;
    default:
      Serial.println("Unknown error");
      return false;
  }

  // OK converted!
  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    Serial.println("Found a print match!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_NOTFOUND) {
    Serial.println("Did not find a match");
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);

  return true;
}

// returns -1 if failed, otherwise returns ID #
int getFingerprintIDez() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)  return -1;

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);
  return finger.fingerID;
}
