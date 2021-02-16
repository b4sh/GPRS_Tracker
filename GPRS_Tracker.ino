/*

  APRS-IS GPRS Tracker
  Krzysztof Kwiatkowski SQ1KW (chris@sq1kw.info)

  Hardware: Arduino Pro Mini, SIM800L GSM Module, Neo6M GPS Module

  Based on Yuri Maltsev UB3FBR gprs tracker https://github.com/UB3FBR/sim800l_gprs_gps_aprs
  SmartBeaconing routine form https://github.com/billygr/arduino-aprs-tracker

*/

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

/////////////////////////////////////////////////////////////// SETTINGS

char callsign[] = "N0CALL-5";
char aprspass[] = "XXXXX";
char comment[] = "Your comment here";

// SmartBeaconing(tm) Setting http://www.hamhud.net/hh2/smartbeacon.html

#define LOW_SPEED 3           // [km/h]
#define HIGH_SPEED 90

#define SLOW_RATE 600         // [sec]
#define FAST_BEACON_RATE  60

#define TURN_MIN  15
#define TURN_SLOPE  24
#define MIN_TURN_TIME 20

/////////////////////////////////////////////////////////////// 

SoftwareSerial myGSM(8, 9);   // SIM800L pin TX to Arduino pin 8, SIM800L pin RX to Arduino pin 9
SoftwareSerial myGPS(6, 7);   // GPS Neo6M pin TX to Arduino pin 6

TinyGPSPlus gps;

/*
   NMEA sequences, more information on http://aprs.gids.nl/nmea/
   From the sequence "cut out" the elements necessary for sending data to the APRS server.
   For APRS we need raw LAT/LON in NMEA format.
*/

TinyGPSCustom la(gps, "GPGGA", 2);    // $GPGGA sentence, element 2
TinyGPSCustom n_s(gps, "GPGGA", 3);   // $GPGGA sentence, element 3
TinyGPSCustom lon(gps, "GPGGA", 4);   // $GPGGA sentence, element 4
TinyGPSCustom e_w(gps, "GPGGA", 5);   // $GPGGA sentence, element 5

/* Variables for console output and the formation of a packet sent to APRS-IS */

String lla;
String llon;
String ns;
String ew;
String sats;
String spds;
String head;
String alt;

int intsats;
int intspds;
int inthead;
int intalt;

int send_error = 0;
unsigned long last_print = 0UL;

String vbatt;

unsigned long lastTX = 0;

/////////////////////////////////////////////////////////////// SETUP

void setup()
{
  Serial.begin(9600);
  delay(500);

  myGSM.begin(9600);
  delay(500);

  myGPS.begin(9600);  // set the correct baud rate for your GPS module
  delay(500);

  pinMode(LED_BUILTIN, OUTPUT);
  randomSeed(analogRead(1));

  GPSfixTest();       // wait for correct GPS data
  myGSMsetup();       // set the parameters of the SIM800L module and connecting to gprs
}

/////////////////////////////////////////////////////////////// LOOP

void loop()
{
  while (myGPS.available() > 0)
    gps.encode(myGPS.read());

  if (gps.location.isUpdated() && gps.location.age() < 1000 && gps.location.isValid())
  {
    lla = la.value();
    lla = lla.substring(0, 7);    //string.substring (from, to)
    ns = n_s.value();

    llon = lon.value();
    llon = llon.substring(0, 8);  //string.substring (from, to)
    ew = e_w.value();

    sats = gps.satellites.value(); intsats = sats.toInt();
    spds = gps.speed.knots(); intspds = spds.toInt();
    head = gps.course.deg(); inthead = head.toInt();
    alt = gps.altitude.feet(); intalt = alt.toInt();

    if (millis() - last_print > 1000) {

      Serial.println();
      Serial.print(F("LAT = ")); Serial.print(lla); Serial.println(ns);
      Serial.print(F("LON = ")); Serial.print(llon); Serial.println(ew);
      Serial.print(F("Speed (knots) = ")); Serial.println(intspds);
      Serial.print(F("Heading (deg) = ")); Serial.println(inthead);
      Serial.print(F("Alt (feet) = ")); Serial.println(intalt);
      Serial.println();
      Serial.print(F("SATs = ")); Serial.println(sats);
      Serial.print(F("Loc age (ms) = ")); Serial.println(gps.location.age());
      Serial.println();

      last_print = millis();
    }

    /*
        If GPS data is ok, send packet to APRS
    */
    if ((intsats > 3) && (lla.length() != NULL && llon.length() != NULL) && inthead <= 360) {

      Serial.println(F(">> SATs > 3"));
      getVBAT();
      Serial.println(F(">> Open port and Send data"));

      Serial.println(F(">> LED On"));
      digitalWrite(LED_BUILTIN, HIGH);

      myGSMopenport();
      sendTCPpacket();

      lastTX = millis();

      Serial.println(F(">> Close port and Delay"));
      myGSMcloseport();

      Serial.println(F(">> LED Off"));
      digitalWrite(LED_BUILTIN, LOW);

      Serial.println (F("========================="));

      smartDelay();
    }
  }
}

///////////////////////////////////////////////////////////////

void printSerialData()
{
  while (myGSM.available()) {
    String inData = myGSM.readStringUntil('\n');
    Serial.println(inData);
  }
}

///////////////////////////////////////////////////////////////

void sendTCPpacket()
{
  char login[60];
  char sentence[150];

  char clla[10];
  char cllon[10];
  char cns[2];
  char cew[2];
  char cvbatt[5];

  lla.toCharArray(clla, 10);
  llon.toCharArray(cllon, 10);
  ns.toCharArray(cns, 2);
  ew.toCharArray(cew, 2);
  vbatt.toCharArray(cvbatt, 5);

  myGSM.listen();

  sprintf(login, "user %s pass %s vers Arduino-SIM800 0.9\n", callsign, aprspass);
  myGSM.println(login);
  delay(3000);

  sprintf(sentence, "%s>APRS,TCPIP*:=%s%s/%s%s[%03d/%03d/GPRS Tracker // Sat: %d // Bat: %sV /A=%06d\n", callsign, clla, cns, cllon, cew, inthead, intspds, intsats, cvbatt, intalt);
  myGSM.println(sentence);
  delay(500);
  sprintf(sentence, "%s>APRS,TCPIP*:>%s\n", callsign, comment);
  myGSM.println(sentence);
  delay(500);

  myGSM.write(0x1A);
  delay(1000);

  myGPS.listen();
}

///////////////////////////////////////////////////////////////

/*
    AT commands for GPRS connection
*/

void myGSMsetup()
{
  Serial.println (F(">> GSM setup"));
  myGSM.listen();
  delay (10000);

  do {
    Serial.println(F(">> Check module status"));
    myGSM.println(F("AT+CPAS"));
    delay (500);
  } while (!myGSM.find("0")); // Status = 0, modem ready to work

  do {
    Serial.println(F(">> Check registration in the network"));
    myGSM.println(F("AT+CREG?"));
    delay (500);
  } while (!(myGSM.find("+CREG: 0,1")) or (myGSM.find("+CREG: 0,5"))); // (0,1) registered - home network, (0,5) registered - roaming

  Serial.println(F(">> GPRS initialization"));
  myGSM.println(F("AT+CIPSHUT")); // Close all IP connections
  printSerialData();
  delay(1000);

  do {
    Serial.println(F(">> GPRS Attaching"));
    myGSM.println(F("AT+CGATT=1"));
    delay (500);
  } while (!myGSM.find("OK"));

  myGSM.println(F("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""));
  delay(1000);
  printSerialData();

  myGSM.println(F("AT+CSTT=\"internet\""));   // Set Access Point Name (APN)
  printSerialData();
  delay(1000);

  do {
    Serial.println(F(">> Start wireless"));
    myGSM.println(F("AT+CIICR"));
    delay (1000);
  } while (!myGSM.find("OK"));

  do {
    Serial.println(F(">> Check IP address"));
    myGSM.println(F("AT+CIFSR"));
    delay (2500);
  } while (!myGSM.find("."));

  send_error = 0;

  Serial.println(F(">> Setup completed"));
  Serial.println(F("========================="));

  myGPS.listen();
}

///////////////////////////////////////////////////////////////

void myGSMopenport()
{
  /*

    TCP connection to euro.aprs2.net (62.69.192.233) on port 14580

  */
  myGSM.listen();

  myGSM.println(F("AT+CIPSTART=\"TCP\",\"62.69.192.233\",\"14580\""));
  printSerialData();
  delay(3000);

  myGSM.println(F("AT+CIPSEND"));
  printSerialData();
  delay(2000);

  if (myGSM.find("ERROR"))
  {
    send_error = 1;
    Serial.println(F(">> Aprs server connect error"));
  }

  myGPS.listen();
}

///////////////////////////////////////////////////////////////

void myGSMcloseport() {
  myGSM.listen();

  myGSM.println(F("AT+CIPCLOSE"));
  delay(1000);

  if (myGSM.find("OK")) {
    send_error = 0;
    Serial.println(F(">> Aprs server connection close"));
  }

  myGSM.println(F("AT+CIPSHUT"));

  myGPS.listen();
}

///////////////////////////////////////////////////////////////

void GPSfixTest() {
  bool gpsfixed = false;

  myGPS.listen();

  Serial.print(F(">> Waiting for GPS fix: "));
  digitalWrite(LED_BUILTIN, HIGH);
  do
  {
    while (myGPS.available() > 0)
      gps.encode(myGPS.read());

    if (gps.location.isValid() && gps.location.age() < 1000 && gps.altitude.age() < 5000)
    {
      Serial.println(F("Fixed!"));
      gpsfixed = true;
      digitalWrite(LED_BUILTIN, LOW);
    }
  } while (gpsfixed != true);
}

///////////////////////////////////////////////////////////////

void smartDelay()
{
  int currentcourse;
  int previouscourse = inthead;
  int fkmph;
  int head;

  unsigned long tx_interval = 0;

  myGPS.listen();
  Serial.print(F("SmartDelay start..."));

  /// SmartDelay Loop
  while (1) {
    bool newData = false;
    int turn_threshold = 0, courseDelta = 0;

    for (unsigned long start = millis(); millis() - start < 1000;)
    {
      while (myGPS.available() > 0)
        gps.encode(myGPS.read());

      if (gps.speed.isUpdated() && gps.course.isUpdated())
      {
        fkmph = gps.speed.kmph();
        head = gps.course.deg();
        newData = true;
        digitalWrite(LED_BUILTIN, HIGH);
      } else {
        digitalWrite(LED_BUILTIN, LOW);
      };
    }

    if (newData)
    {
      currentcourse = head;

      // Calculate difference of course to smartbeacon
      courseDelta = (int) ( previouscourse - currentcourse );
      courseDelta = abs(courseDelta);
      if (courseDelta > 180) {
        courseDelta = courseDelta - 360;
      }
      courseDelta = abs (courseDelta);
      //Serial.print(F("Course delta deg = ")); Serial.println(courseDelta);

      // Based on HamHUB Smart Beaconing(tm) algorithm
      if ( fkmph < LOW_SPEED ) {
        tx_interval = SLOW_RATE * 1000L;
      }
      else if ( fkmph > HIGH_SPEED) {
        tx_interval = FAST_BEACON_RATE  * 1000L;
      }
      else {
        // Interval inbetween low and high speed
        tx_interval = ((FAST_BEACON_RATE * HIGH_SPEED) / fkmph ) * 1000L ;
      }

      turn_threshold = TURN_MIN + TURN_SLOPE / fkmph;

      if (courseDelta > turn_threshold ) {
        if ( millis() - lastTX > MIN_TURN_TIME * 1000L) {
          Serial.print(F("...SmartDelay end. Min turn time < ")); Serial.print((millis() - lastTX) / 1000);
          return;
        }
      }

      previouscourse = currentcourse;

      if ( millis() - lastTX > tx_interval) {
        Serial.print(F("...SmartDelay end. TX interval (sec) = ")); Serial.print(tx_interval / 1000);
        return;
      }
    }
  }
}

///////////////////////////////////////////////////////////////

void getVBAT() {
  unsigned long waitTime = millis();
  String batt;
  char incomingByte;
  
  myGSM.listen();

  // Get the battery voltage
  Serial.print(">> Get the battery voltage: ");
  myGSM.println(F("AT+CBC"));

  while (millis() - waitTime < 4000) {
    if (!myGSM.available()) {
      // Nothing in the buffer, wait a bit
      delay(5);
      continue;
    }
    incomingByte = myGSM.read();
    if (incomingByte == 0) {
      // Ignore NULL character
      continue;
    }
    //Serial.print(incomingByte);
    batt += incomingByte;

    bool found = batt.lastIndexOf("CBC:") != -1;
    if (incomingByte == '\n' && !found) {
      // Not the line we're looking for
      batt = "";
    }
    if (incomingByte == '\n' && found) {
      // Found the response
      batt = batt.substring(batt.lastIndexOf(",") + 1, batt.lastIndexOf(",") + 5);
      break;
    }
  }
  vbatt = String(batt.toFloat() / 1000);
  Serial.println(vbatt);
}
