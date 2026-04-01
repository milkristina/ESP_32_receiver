#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

struct DecodeResult {
  byte data;
  bool corrected;
  bool valid;
  int syndrome;
};

// OLED
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET       4
#define SDA_PIN         21
#define SCL_PIN         22
#define SCREEN_ADDRESS 0x3C

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Wi-Fi / UDP

const char* ssid     = "ESP32_MORSE_TX";
const char* password = "12345678";

WiFiUDP udp;
const uint16_t localPort = 4210;


// Morse lentelė

const char* letters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
String morseTable[] = {
  ".-","-...","-.-.","-..",".","..-.","--.","....","..",".---",
  "-.-",".-..","--","-.","---",".--.","--.-",".-.","...","-",
  "..-","...-",".--","-..-","-.--","--..",
  "-----",".----","..---","...--","....-",".....",
  "-....","--...","---..","----."
};


// Būsena
String currentMorse = "";
String fullMorse    = "";
String decodedText  = "";
String statusMsg    = "BOOT";

IPAddress lastSenderIP;
uint16_t  lastSenderPort = 4210;

// laiko sekimas raidžių/žodžių atskyrimui
unsigned long lastPacketTime = 0;
const unsigned long LETTER_GAP_MS = 700;
const unsigned long WORD_GAP_MS   = 1800;


// Statistikos

unsigned long totalPackets         = 0;
unsigned long syncPackets          = 0;
unsigned long dataPackets          = 0;
unsigned long correctedPackets     = 0;
unsigned long uncorrectablePackets = 0;
unsigned long arqRequests          = 0;

unsigned long totalBitsReceived    = 0;
unsigned long usefulBitsRecovered  = 0;
unsigned long correctedBitCount    = 0;

int lastRSSI = 0;


// Pagalbinės funkcijos

void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print("ESP32 MORSE RX");
  display.drawLine(0, 9, 127, 9, SH110X_WHITE);

  display.setCursor(0, 13);
  display.print("TXT: ");
  String txt = decodedText;
  if (txt.length() > 12) txt = txt.substring(txt.length() - 12);
  display.print(txt);

  display.setCursor(0, 25);
  display.print("MRS: ");
  String m = fullMorse + currentMorse;
  if (m.length() > 14) m = m.substring(m.length() - 14);
  display.print(m);

  display.setCursor(0, 37);
  display.print("RSSI: ");
  display.print(lastRSSI);
  display.print(" dBm");

  float ber = (totalBitsReceived > 0) ? (float)correctedBitCount / (float)totalBitsReceived : 0.0f;
  float eff = (totalBitsReceived > 0) ? 100.0f * ((float)usefulBitsRecovered / (float)totalBitsReceived) : 0.0f;

  display.setCursor(0, 49);
  display.print("BER:");
  display.print(ber, 3);

  display.setCursor(70, 49);
  display.print("EFF:");
  display.print(eff, 1);
  display.print("%");

  display.setCursor(0, 57);
  String st = statusMsg;
  if (st.length() > 21) st = st.substring(0, 21);
  display.print(st);

  display.display();
}

void connectToAP() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Jungiamasi prie WiFi");
  statusMsg = "Connecting WiFi";
  updateDisplay();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Prisijungta. IP: ");
  Serial.println(WiFi.localIP());

  lastRSSI = WiFi.RSSI();
  statusMsg = "WiFi connected";
  updateDisplay();
}

char morseToChar(String code) {
  for (int i = 0; i < 36; i++) {
    if (morseTable[i] == code) {
      return letters[i];
    }
  }
  return '?';
}

void flushCurrentLetter() {
  if (currentMorse.length() == 0) return;

  char c = morseToChar(currentMorse);
  decodedText += c;

  fullMorse += currentMorse;
  fullMorse += " ";

  Serial.print("Raide: ");
  Serial.println(c);

  currentMorse = "";
}

void sendARQRequest() {
  if (lastSenderIP == IPAddress(0,0,0,0)) return;

  udp.beginPacket(lastSenderIP, lastSenderPort);
  udp.print("NACK");
  udp.endPacket();

  arqRequests++;
  Serial.println(">> Issiustas ARQ/NACK");
}

byte stringToByte(const String &s) {
  int value = s.toInt();
  if (value < 0) value = 0;
  if (value > 255) value = 255;
  return (byte)value;
}


// Hamming (7,4) dekodavimas
// code bitų tvarka siųstuve:
// bit0=p1 bit1=p2 bit2=d1 bit3=p3 bit4=d2 bit5=d3 bit6=d4

DecodeResult hammingDecode(byte code) {
  DecodeResult res;
  res.data = 0;
  res.corrected = false;
  res.valid = true;
  res.syndrome = 0;

  byte b1 = bitRead(code, 0);
  byte b2 = bitRead(code, 1);
  byte b3 = bitRead(code, 2);
  byte b4 = bitRead(code, 3);
  byte b5 = bitRead(code, 4);
  byte b6 = bitRead(code, 5);
  byte b7 = bitRead(code, 6);

  byte s1 = b1 ^ b3 ^ b5 ^ b7;
  byte s2 = b2 ^ b3 ^ b6 ^ b7;
  byte s3 = b4 ^ b5 ^ b6 ^ b7;

  int syndrome = s1 + (s2 << 1) + (s3 << 2);
  res.syndrome = syndrome;

  if (syndrome != 0) {
    int bitIndex = syndrome - 1;
    if (bitIndex >= 0 && bitIndex <= 6) {
      code ^= (1 << bitIndex);
      res.corrected = true;
    } else {
      res.valid = false;
      return res;
    }
  }

  byte d1 = bitRead(code, 2);
  byte d2 = bitRead(code, 4);
  byte d3 = bitRead(code, 5);
  byte d4 = bitRead(code, 6);

  byte data = 0;
  bitWrite(data, 0, d1);
  bitWrite(data, 1, d2);
  bitWrite(data, 2, d3);
  bitWrite(data, 3, d4);

  res.data = data;
  return res;
}

// 4 bitų simbolio interpretavimas pagal tavo siųstuvą
// '.' ASCII 46 -> 0xE -> 14
// '-' ASCII 45 -> 0xD -> 13
// ' ' ASCII 32 -> 0x0 -> 0

char nibbleToMorseSymbol(byte nibble) {
  if (nibble == 14) return '.';
  if (nibble == 13) return '-';
  if (nibble == 0)  return ' ';
  return '?';
}

void handleDecodedNibble(byte nibble) {
  char sym = nibbleToMorseSymbol(nibble);

  if (sym == '.') {
    currentMorse += ".";
    statusMsg = "Dot received";
  } 
  else if (sym == '-') {
    currentMorse += "-";
    statusMsg = "Dash received";
  } 
  else if (sym == ' ') {
    flushCurrentLetter();
    statusMsg = "Letter end";
  } 
  else {
    statusMsg = "Unknown nibble";
    Serial.print("Nezinomas nibble: ");
    Serial.println(nibble);
  }
}

void handlePacketTimeouts() {
  if (lastPacketTime == 0) return;

  unsigned long dt = millis() - lastPacketTime;

  if (currentMorse.length() > 0 && dt > LETTER_GAP_MS) {
    flushCurrentLetter();
  }

  static bool wordGapInserted = false;
  if (dt > WORD_GAP_MS && !wordGapInserted) {
    if (decodedText.length() > 0 && decodedText[decodedText.length() - 1] != ' ') {
      decodedText += " ";
      fullMorse += "/ ";
      wordGapInserted = true;
      statusMsg = "Word gap";
    }
  }

  if (dt <= LETTER_GAP_MS) {
    wordGapInserted = false;
  }
}

// Setup

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, LOW);
  delay(20);
  digitalWrite(OLED_RESET, HIGH);
  delay(100);

  display.begin(SCREEN_ADDRESS, true);
  display.clearDisplay();
  display.display();

  statusMsg = "Init";
  updateDisplay();

  connectToAP();

  udp.begin(localPort);
  Serial.print("UDP klausosi porto ");
  Serial.println(localPort);

  statusMsg = "Listening UDP";
  updateDisplay();
}


// Loop

void loop() {
  handlePacketTimeouts();

  int packetSize = udp.parsePacket();
  if (packetSize <= 0) {
    lastRSSI = WiFi.RSSI();
    updateDisplay();
    delay(50);
    return;
  }

  totalPackets++;
  totalBitsReceived += 7;
  usefulBitsRecovered += 4;

  lastSenderIP = udp.remoteIP();
  lastSenderPort = udp.remotePort();
  lastPacketTime = millis();
  lastRSSI = WiFi.RSSI();

  char buffer[64];
  int len = udp.read(buffer, sizeof(buffer) - 1);
  if (len < 0) return;
  buffer[len] = '\0';

  String packet = String(buffer);
  packet.trim();

  Serial.print("Gautas paketas: ");
  Serial.println(packet);

  if (packet == "SYNC") {
    syncPackets++;

    flushCurrentLetter();

    if (decodedText.length() > 0) {
      Serial.print("Pilnas tekstas iki SYNC: ");
      Serial.println(decodedText);
    }

    currentMorse = "";
    fullMorse    = "";
    decodedText  = "";
    statusMsg    = "SYNC received";

    updateDisplay();
    return;
  }

  dataPackets++;

  bool numericPacket = true;
  for (unsigned int i = 0; i < packet.length(); i++) {
    if (!isDigit(packet[i])) {
      numericPacket = false;
      break;
    }
  }

  if (!numericPacket) {
    uncorrectablePackets++;
    statusMsg = "Bad packet format";
    sendARQRequest();
    updateDisplay();
    return;
  }

  byte rawCode = stringToByte(packet);
  DecodeResult dr = hammingDecode(rawCode);

  if (!dr.valid) {
    uncorrectablePackets++;
    statusMsg = "Decode failed";
    sendARQRequest();
    updateDisplay();
    return;
  }

  if (dr.corrected) {
    correctedPackets++;
    correctedBitCount++;
    Serial.println("Vieno bito klaida pataisyta.");
  }

  handleDecodedNibble(dr.data);

  Serial.print("Decoded nibble: ");
  Serial.println(dr.data);

  Serial.print("Current Morse: ");
  Serial.println(currentMorse);

  Serial.print("Full Morse   : ");
  Serial.println(fullMorse);

  Serial.print("Decoded text : ");
  Serial.println(decodedText);

  updateDisplay();
}
