#include <LibAPRS.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>   // untuk ultoa/ltoa

// ========= Konfigurasi LibAPRS =========
#define ADC_REFERENCE  REF_5V
#define OPEN_SQUELCH   true

// ========= PTT =========
#define PTT_PIN          3         // D3 (INT1) pada Nano/Uno
#define PTT_ACTIVE_HIGH  1         // 1=PTT aktif saat HIGH, 0=aktif saat LOW
const uint16_t PREKEY_MS   = 800;  // tunggu sebelum kirim
const uint16_t POSTKEY_MS  = 800;  // tahan PTT setelah kirim
const uint16_t TX_GUARD_MS = 900;  // jeda tambahan agar modulasi tuntas

// ========= Identitas & interval =========
#define MY_CALLSIGN   "YC2UTC"     // GANTI sesuai callsign
#define MY_SSID       7            // contoh: -7 untuk tracker
const unsigned long TX_INTERVAL_MS = 30000;  // 30 detik

// ========= Pin GPS (SoftwareSerial) =========
#define GPS_RX_PIN 12   // Arduino menerima di sini  (hubungkan ke TX GPS)
#define GPS_TX_PIN 11   // Arduino kirim ke GPS (opsional)

// ========= GPS & buffer =========
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;
char lat_nmea[12];
char lon_nmea[12];

unsigned long last_tx = 0;

// ====== Stub callback LibAPRS (wajib, meski TX-only) ======
void aprs_msg_callback(struct AX25Msg *msg) {}

// ====== Helper PTT ======
inline void ptt_on()  { digitalWrite(PTT_PIN, PTT_ACTIVE_HIGH ? HIGH : LOW); }
inline void ptt_off() { digitalWrite(PTT_PIN, PTT_ACTIVE_HIGH ? LOW  : HIGH); }

// ====== Konversi derajat desimal -> "DDMM.mmN" ======
void toNMEA_Lat(double lat, char* out) {
  char hemi = (lat >= 0) ? 'N' : 'S';
  double alat = fabs(lat);
  int deg = (int)alat;
  double minf = (alat - deg) * 60.0;
  int mm = (int)minf;
  int frac = (int)round((minf - mm) * 100.0);   // dua digit
  if (frac == 100) { frac = 0; mm += 1; }
  if (mm == 60)   { mm = 0; deg += 1; }
  // DDMM.mm + hemisphere (tanpa spasi)
  snprintf(out, 12, "%02d%02d.%02d%c", deg, mm, frac, hemi);
}

// ====== Konversi derajat desimal -> "DDDMM.mmE" ======
void toNMEA_Lon(double lon, char* out) {
  char hemi = (lon >= 0) ? 'E' : 'W';
  double alon = fabs(lon);
  int deg = (int)alon;
  double minf = (alon - deg) * 60.0;
  int mm = (int)minf;
  int frac = (int)round((minf - mm) * 100.0);
  if (frac == 100) { frac = 0; mm += 1; }
  if (mm == 60)   { mm = 0; deg += 1; }
  // DDDMM.mm + hemisphere (tanpa spasi)
  snprintf(out, 12, "%03d%02d.%02d%c", deg, mm, frac, hemi);
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);

  // Siapkan PTT pin
  pinMode(PTT_PIN, OUTPUT);
  ptt_off();

  // Inisialisasi LibAPRS
  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
  APRS_setCallsign((char*)MY_CALLSIGN, MY_SSID);
  APRS_setPath1("WIDE1", 2);   // = WIDE1-1
  // APRS_setPath2("WIDE2", 1);   // = WIDE2-1
  APRS_setPreamble(500);       // tambah agar awal paket tak kepotong
  APRS_setTail(50);
  APRS_setSymbol('>');         // simbol mobil

  Serial.println(F("LibAPRS Nano Tracker siap"));
}

void loop() {
  // Feed GPS ke parser
  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  // Kirim tiap interval jika GPS valid
  if (millis() - last_tx >= TX_INTERVAL_MS) {
    last_tx = millis();

    if (gps.location.isValid()) {
      // Format koordinat APRS (uncompressed)
      toNMEA_Lat(gps.location.lat(), lat_nmea);
      toNMEA_Lon(gps.location.lng(), lon_nmea);
      APRS_setLat(lat_nmea);
      APRS_setLon(lon_nmea);

      // ---------- Siapkan COMMENT tanpa %f ----------
      // Speed (knots), altitude (meters), satellites
      char spd_str[8], alt_str[12], sat_str[6];

      if (gps.speed.isValid()) {
        unsigned long spd_kt = (unsigned long)(gps.speed.knots() + 0.5); // bulatkan
        ultoa(spd_kt, spd_str, 10);
      } else {
        strcpy(spd_str, "-");
      }

      if (gps.altitude.isValid()) {
        long alt_m = (long)(gps.altitude.meters() + 0.5); // bulatkan
        ltoa(alt_m, alt_str, 10);
      } else {
        strcpy(alt_str, "-");
      }

      if (gps.satellites.isValid()) {
        unsigned long sats = gps.satellites.value();
        ultoa(sats, sat_str, 10);
      } else {
        strcpy(sat_str, "-");
      }

      char comment[96];
      // contoh: "spd=12kt alt=45m sats=8 Nano+LibAPRS"
      snprintf(comment, sizeof(comment),
               "spd=%skt alt=%sm sats=%s",
               spd_str, alt_str, sat_str);
      // ----------------------------------------------

      // Debug: cetak posisi & comment yang akan dikirim
      Serial.print(F("Loc: "));
      Serial.print(lat_nmea); Serial.print(F(","));
      Serial.println(lon_nmea);
      Serial.print(F("Cmt: "));
      Serial.println(comment);

      // --- Urutan TX aman untuk HT ---
      ptt_on();
      delay(PREKEY_MS);

      // Hentikan baca GPS saat TX (hindari gangguan timing AFSK di AVR)
      gpsSerial.end();

      APRS_sendLoc(comment, strlen(comment));

      delay(POSTKEY_MS);
      ptt_off();

      // Guard time agar modulasi benar-benar selesai
      delay(TX_GUARD_MS);

      // Lanjutkan baca GPS
      gpsSerial.begin(9600);

      // Debug
      Serial.print(F("TX @ "));
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(", "));
      Serial.print(gps.location.lng(), 6);
      Serial.println();
    } else {
      Serial.println(F("GPS belum fix; skip beacon"));
    }
  }
}