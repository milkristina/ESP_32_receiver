#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------------- BLE UUID (SUDERINK SU SIUSTUVU!) ----------------
static NimBLEUUID SERVICE_UUID("12345678-1234-1234-1234-1234567890ab");
static NimBLEUUID CHAR_UUID   ("abcdefab-1234-5678-9abc-def012345678");

static const char* TARGET_NAME = "MORSE_TX"; 

uint32_t framesOk = 0, framesBad = 0;
uint32_t totalCodewords = 0;
uint32_t correctedCodewords = 0;
uint32_t totalBitsRx = 0;
uint32_t usefulBits = 0;

int lastRssi = 0;
uint8_t lastSeq = 0;


String oledMorse = "";
String oledText  = "";
String oledStatus = "BOOT";
String oledError  = "";

unsigned long lastGoodFrameMs = 0;                
const unsigned long NO_DATA_TIMEOUT_MS = 5000;    

static NimBLEAdvertisedDevice* advDevice = nullptr;
static NimBLEClient* pClient = nullptr;
static NimBLERemoteCharacteristic* pRemoteChar = nullptr;

// ---------------- Morse mapping ----------------
struct MorseMap { const char* code; char ch; };
const MorseMap MORSE_TABLE[] = {
  {".-", 'A'},   {"-...", 'B'}, {"-.-.", 'C'}, {"-..", 'D'},  {".", 'E'},
  {"..-.", 'F'}, {"--.", 'G'},  {"....", 'H'}, {"..", 'I'},   {".---", 'J'},
  {"-.-", 'K'},  {".-..", 'L'}, {"--", 'M'},   {"-.", 'N'},   {"---", 'O'},
  {".--.", 'P'}, {"--.-", 'Q'}, {".-.", 'R'},  {"...", 'S'},  {"-", 'T'},
  {"..-", 'U'},  {"...-", 'V'}, {".--", 'W'},  {"-..-", 'X'}, {"-.--", 'Y'},
  {"--..", 'Z'},
  {"-----",'0'}, {".----",'1'}, {"..---",'2'}, {"...--",'3'}, {"....-",'4'},
  {".....",'5'}, {"-....",'6'}, {"--...",'7'}, {"---..",'8'}, {"----.",'9'},
  {nullptr, 0}
};

char morseToChar(const String& morse) {
  for (int i = 0; MORSE_TABLE[i].code != nullptr; i++) {
    if (morse == MORSE_TABLE[i].code) return MORSE_TABLE[i].ch;
  }
  return '?';
}

// Morzės eilutė formatu: "... --- ... / .-"
String morseStringToText(const String& morseLine) {
  String out = "", token = "";
  for (size_t i = 0; i < morseLine.length(); i++) {
    char c = morseLine[i];
    if (c == ' ') {
      if (token.length()) { out += morseToChar(token); token = ""; }
    } else if (c == '/') {
      if (token.length()) { out += morseToChar(token); token = ""; }
      out += ' ';
    } else {
      token += c; // '.' arba '-'
    }
  }
  if (token.length()) out += morseToChar(token);
  return out;
}

// ---------------- CRC8 ----------------
uint8_t crc8_simple(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x07;
      else crc <<= 1;
    }
  }
  return crc;
}

// ---------------- Hamming(7,4) decode (1-bit correction) ----------------
// poz: 1 2 3 4 5 6 7
// bit: p1 p2 d1 p4 d2 d3 d4
// cw7 laikom kaip 7 bitus: bit0=poz1 ... bit6=poz7
bool hamming74_decode(uint8_t cw7, uint8_t &nibble, bool &corrected) {
  int b1 = (cw7 >> 0) & 1;
  int b2 = (cw7 >> 1) & 1;
  int b3 = (cw7 >> 2) & 1;
  int b4 = (cw7 >> 3) & 1;
  int b5 = (cw7 >> 4) & 1;
  int b6 = (cw7 >> 5) & 1;
  int b7 = (cw7 >> 6) & 1;

  int s1 = b1 ^ b3 ^ b5 ^ b7; // tikrina 1,3,5,7
  int s2 = b2 ^ b3 ^ b6 ^ b7; // tikrina 2,3,6,7
  int s4 = b4 ^ b5 ^ b6 ^ b7; // tikrina 4,5,6,7
  int syndrome = (s4 << 2) | (s2 << 1) | s1; // 0..7

  corrected = false;

  if (syndrome != 0) {
    // pataisom 1 bitą
    cw7 ^= (1 << (syndrome - 1));
    corrected = true;

    // percheck
    b1 = (cw7 >> 0) & 1; b2 = (cw7 >> 1) & 1; b3 = (cw7 >> 2) & 1;
    b4 = (cw7 >> 3) & 1; b5 = (cw7 >> 4) & 1; b6 = (cw7 >> 5) & 1; b7 = (cw7 >> 6) & 1;

    s1 = b1 ^ b3 ^ b5 ^ b7;
    s2 = b2 ^ b3 ^ b6 ^ b7;
    s4 = b4 ^ b5 ^ b6 ^ b7;
    int synd2 = (s4 << 2) | (s2 << 1) | s1;
    if (synd2 != 0) return false; // tikėtina 2-bit klaida
  }

  // d1 d2 d3 d4 = b3 b5 b6 b7
  nibble = (b3 << 3) | (b5 << 2) | (b6 << 1) | b7;
  return true;
}

// Bit-stream reader: skaito po n bitų iš baitų masyvo (LSB-first)
struct BitReader {
  const uint8_t* data; size_t len; size_t bitPos;
  BitReader(const uint8_t* d, size_t l): data(d), len(l), bitPos(0) {}
  bool readBits(uint8_t &out, int n) {
    if (bitPos + n > len * 8) return false;
    out = 0;
    for (int i = 0; i < n; i++) {
      size_t byteIdx = (bitPos + i) / 8;
      int bitIdx = (bitPos + i) % 8;
      int bit = (data[byteIdx] >> bitIdx) & 1;
      out |= (bit << i);
    }
    bitPos += n;
    return true;
  }
};

// Dekoduoja Hamming srautą -> ASCII simbolių eilutė (Morzė teksto pavidalu)
bool decodeHammingStreamToBytes(const uint8_t* enc, size_t encLen, String &outAscii) {
  outAscii = "";
  BitReader br(enc, encLen);

  while (true) {
    uint8_t cw1, cw2;
    if (!br.readBits(cw1, 7)) break;
    if (!br.readBits(cw2, 7)) return false;

    uint8_t n1, n2;
    bool c1=false, c2=false;

    bool ok1 = hamming74_decode(cw1, n1, c1);
    bool ok2 = hamming74_decode(cw2, n2, c2);

    totalCodewords += 2;
    totalBitsRx += 14;

    if (!ok1 || !ok2) return false;
    if (c1) correctedCodewords++;
    if (c2) correctedCodewords++;

    uint8_t byteVal = (n1 << 4) | (n2 & 0x0F);
    outAscii += (char)byteVal;

    if (outAscii.length() > 200) break; // saugiklis
  }
  return true;
}

// ---------------- Frame parsing ----------------
// Format:
// [0]=0x55 [1]=0xAA [2]=seq [3]=len [4..4+len-1]=payload [4+len]=crc8
bool parseFrame(const uint8_t* buf, size_t n, uint8_t &seq, const uint8_t* &payload, size_t &plen) {
  if (n < 2 + 1 + 1 + 1) return false;
  if (buf[0] != 0x55 || buf[1] != 0xAA) return false;

  seq = buf[2];
  plen = buf[3];
  if (4 + plen + 1 > n) return false;

  uint8_t crcRx = buf[4 + plen];
  uint8_t crcCalc = crc8_simple(buf, 4 + plen);
  if (crcRx != crcCalc) return false;

  payload = &buf[4];
  return true;
}

// ---------------- OLED helpers ----------------
void showOnOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0,0);
  display.print("RSSI: "); display.print(lastRssi); display.println(" dBm");
  display.print("OK/Bad: "); display.print(framesOk); display.print("/"); display.println(framesBad);
  display.print("Status: "); display.println(oledStatus);

  if (oledError.length() > 0) {
    display.print("Err: "); display.println(oledError);
  }

  display.println("Morse:");
  display.println(oledMorse.substring(0, 21));
  display.println("Text:");
  display.println(oledText.substring(0, 21));

  display.display();
}

void setStatus(const String& status, const String& err,
               const String& morse, const String& text) {
  oledStatus = status;
  oledError  = err;
  if (morse.length() > 0 || status == "RX OK") oledMorse = morse;
  if (text.length()  > 0 || status == "RX OK") oledText  = text;
  showOnOLED();
}

// ---------------- BLE scan callback ----------------
class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(NimBLEAdvertisedDevice* dev) override {
    bool nameOk = dev->haveName() && (strcmp(dev->getName().c_str(), TARGET_NAME) == 0);
    bool svcOk  = dev->isAdvertisingService(SERVICE_UUID);

    // jei nori griežtai tik pagal service, palik tik svcOk
    if (svcOk || nameOk) {
      advDevice = dev;
      NimBLEDevice::getScan()->stop();
    }
  }
};

// ---------------- Notify handler ----------------
void onNotify(NimBLERemoteCharacteristic* c, uint8_t* data, size_t len, bool isNotify) {
  (void)c; (void)isNotify;

  // RSSI po connect (getRssi) :contentReference[oaicite:3]{index=3}
  if (pClient && pClient->isConnected()) {
    int r = pClient->getRssi();
    if (r != 0) lastRssi = r; // 0 gali reikšti klaidą pagal docs
  }

  uint8_t seq;
  const uint8_t* payload;
  size_t plen;

  if (!parseFrame(data, len, seq, payload, plen)) {
    framesBad++;
    setStatus("RX FAIL", "CRC/PREAMBLE", "", "");
    return;
  }

  String morseAscii;
  if (!decodeHammingStreamToBytes(payload, plen, morseAscii)) {
    framesBad++;
    setStatus("RX FAIL", "HAMMING", "", "");
    return;
  }

  String text = morseStringToText(morseAscii);

  framesOk++;
  lastSeq = seq;
  lastGoodFrameMs = millis();

  // (Optional) jei nori, gali pridėti ir BER/Eff į Serial
  float pCorr = (totalCodewords > 0) ? (float)correctedCodewords / (float)totalCodewords : 0.0f;
  float berEst = pCorr / 7.0f;
  usefulBits += (uint32_t)text.length() * 8;
  float eff = (totalBitsRx > 0) ? (float)usefulBits / (float)totalBitsRx : 0.0f;

  Serial.println("---- RX OK ----");
  Serial.print("SEQ: "); Serial.println(seq);
  Serial.print("RSSI: "); Serial.print(lastRssi); Serial.println(" dBm");
  Serial.print("Morse: "); Serial.println(morseAscii);
  Serial.print("Text : "); Serial.println(text);
  Serial.print("BER~  : "); Serial.println(berEst, 8);
  Serial.print("Eff~  : "); Serial.println(eff, 4);

  setStatus("RX OK", "", morseAscii, text);
}

// ---------------- Connect + Subscribe ----------------
bool connectAndSubscribe() {
  if (!advDevice) return false;

  // Kurti naują client
  pClient = NimBLEDevice::createClient();
  if (!pClient) return false;

  if (!pClient->connect(advDevice)) {
    NimBLEDevice::deleteClient(pClient);
    pClient = nullptr;
    return false;
  }

  // RSSI getteris dokumentuotas NimBLEClient :contentReference[oaicite:4]{index=4}
  lastRssi = pClient->getRssi();

  NimBLERemoteService* svc = pClient->getService(SERVICE_UUID);
  if (!svc) return false;

  pRemoteChar = svc->getCharacteristic(CHAR_UUID);
  if (!pRemoteChar) return false;

  if (!pRemoteChar->canNotify()) return false;

  // subscribe(true, callback) yra oficialus API :contentReference[oaicite:5]{index=5}
  bool ok = pRemoteChar->subscribe(true, onNotify);
  if (!ok) return false;

  lastGoodFrameMs = millis();
  return true;
}

void startScan() {
  advDevice = nullptr;
  setStatus("SCANNING", "", "", "");

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->clearResults();
  scan->setActiveScan(true);
  scan->start(0, false); // 0 = skenuoti kol sustabdysim
}

void setup() {
  Serial.begin(115200);

  // OLED init (ESP32 tipinės I2C kojos SDA=21 SCL=22)
  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // jei tavo OLED adresas 0x3D — pakeisk čia
    Serial.println("OLED init failed");
    while(true) delay(1000);
  }
  display.clearDisplay();
  display.display();

  setStatus("BOOT", "", "", "");

  NimBLEDevice::init("MORSE_RX");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new ScanCallbacks(), false);

  startScan();
}

void loop() {
  // Jei suradom device ir nesam prisijungę — jungiamės
  if (advDevice && (!pClient || !pClient->isConnected())) {
    setStatus("CONNECTING", "", "", "");

    if (connectAndSubscribe()) {
      setStatus("CONNECTED", "", "", "");
    } else {
      setStatus("CONN FAIL", "RESCAN", "", "");
      if (pClient) {
        NimBLEDevice::deleteClient(pClient);
        pClient = nullptr;
      }
      startScan();
    }
  }

  // Jei buvom prisijungę, bet atsijungė
  if (pClient && !pClient->isConnected()) {
    setStatus("DISCONNECTED", "RESCAN", "", "");
    NimBLEDevice::deleteClient(pClient);
    pClient = nullptr;
    startScan();
  }

  // NO DATA timeout (prisijungęs, bet negauna gerų kadrų)
  if (pClient && pClient->isConnected()) {
    unsigned long now = millis();
    if (now - lastGoodFrameMs > NO_DATA_TIMEOUT_MS) {
      setStatus("NO DATA", "WAITING", "", "");
      // kad neperrašinėtų ekrano kas 50 ms:
      lastGoodFrameMs = now - (NO_DATA_TIMEOUT_MS - 1000);
    }
  }

  delay(50);
}
