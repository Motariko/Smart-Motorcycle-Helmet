#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <esp_now.h>
#include <WiFiManager.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "config.h"

#define V_L 26  // พินสำหรับ R
#define V_R 27  // พินสำหรับ L
#define B_Newwifi 25 //ปุ่มสำหรับรับค่าการรับรหัสwifiใหม่
#define B_shutdown 14 //ปุ่มปิดเครื่องและส่งข้อมูลรวม
#define buzzerPin 13

#define NOTE_C4  262
#define NOTE_E4  330 
const char* serverUrl = "http://192.168.43.178:5000/data";

typedef struct struct_message {
    float distance_L;
    float distance_R;
    float kalAngleX;  // เพิ่มค่ามุมจาก Kalman Filter
    float kalAngleY;  // เพิ่มค่ามุมจาก Kalman Filter
    float Accelerer;
} struct_message;

struct_message receivedData;

WiFiManager WIFI;
TinyGPSPlus gps;
HardwareSerial ss(1); // ใช้ UART1

unsigned long previousMillis_L = 0;
unsigned long previousMillis_R = 0;
const long interval = 200;  
unsigned long lastButtonPressTime = 0;
const long debounceDelay = 500;  // ป้องกันการกดซ้ำใน 500 ms

bool VBState_L = LOW;
bool VBState_R = LOW;

int count_L = 0;
int count_R = 0;
int count_BWF = 0;
float lat_GPS = 0;
float lng_GPS = 0;
float speed_GPS = 0;

bool Nub_L = false;
bool Nub_R = false;
bool Nub_BWF = false;

bool needToSendData = false;
bool send_A_flag = false;
bool send_AC_flag = false;

unsigned long lastSendTime = 0;  
const long sendInterval = 5000;

/*const char* root_ca = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIEADCCAuigAwIBAgIID+rOSdTGfGcwDQYJKoZIhvcNAQELBQAwgYsxCzAJBgNV\n" \
"BAYTAlVTMRkwFwYDVQQKExBDbG91ZEZsYXJlLCBJbmMuMTQwMgYDVQQLEytDbG91\n" \
"ZEZsYXJlIE9yaWdpbiBTU0wgQ2VydGlmaWNhdGUgQXV0aG9yaXR5MRYwFAYDVQQH\n" \
"Ew1TYW4gRnJhbmNpc2NvMRMwEQYDVQQIEwpDYWxpZm9ybmlhMB4XDTE5MDgyMzIx\n" \
"MDgwMFoXDTI5MDgxNTE3MDAwMFowgYsxCzAJBgNVBAYTAlVTMRkwFwYDVQQKExBD\n" \
"bG91ZEZsYXJlLCBJbmMuMTQwMgYDVQQLEytDbG91ZEZsYXJlIE9yaWdpbiBTU0wg\n" \
"Q2VydGlmaWNhdGUgQXV0aG9yaXR5MRYwFAYDVQQHEw1TYW4gRnJhbmNpc2NvMRMw\n" \
"EQYDVQQIEwpDYWxpZm9ybmlhMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKC\n" \
"AQEAwEiVZ/UoQpHmFsHvk5isBxRehukP8DG9JhFev3WZtG76WoTthvLJFRKFCHXm\n" \
"V6Z5/66Z4S09mgsUuFwvJzMnE6Ej6yIsYNCb9r9QORa8BdhrkNn6kdTly3mdnykb\n" \
"OomnwbUfLlExVgNdlP0XoRoeMwbQ4598foiHblO2B/LKuNfJzAMfS7oZe34b+vLB\n" \
"yrP/1bgCSLdc1AxQc1AC0EsQQhgcyTJNgnG4va1c7ogPlwKyhbDyZ4e59N5lbYPJ\n" \
"SmXI/cAe3jXj1FBLJZkwnoDKe0v13xeF+nF32smSH0qB7aJX2tBMW4TWtFPmzs5I\n" \
"lwrFSySWAdwYdgxw180yKU0dvwIDAQABo2YwZDAOBgNVHQ8BAf8EBAMCAQYwEgYD\n" \
"VR0TAQH/BAgwBgEB/wIBAjAdBgNVHQ4EFgQUJOhTV118NECHqeuU27rhFnj8KaQw\n" \
"HwYDVR0jBBgwFoAUJOhTV118NECHqeuU27rhFnj8KaQwDQYJKoZIhvcNAQELBQAD\n" \
"ggEBAHwOf9Ur1l0Ar5vFE6PNrZWrDfQIMyEfdgSKofCdTckbqXNTiXdgbHs+TWoQ\n" \
"wAB0pfJDAHJDXOTCWRyTeXOseeOi5Btj5CnEuw3P0oXqdqevM1/+uWp0CM35zgZ8\n" \
"VD4aITxity0djzE6Qnx3Syzz+ZkoBgTnNum7d9A66/V636x4vTeqbZFBr9erJzgz\n" \
"hhurjcoacvRNhnjtDRM0dPeiCJ50CP3wEYuvUzDHUaowOsnLCjQIkWbR7Ni6KEIk\n" \
"MOz2U0OBSif3FTkhCgZWQKOOLo1P42jHC3ssUZAtVNXrCk3fw9/E15k8NPkBazZ6\n" \
"0iykLhH1trywrKRMVw67F44IE8Y=\n" \
"-----END CERTIFICATE-----\n";*/


void New_wifiuser(){
  WIFI.autoConnect("SmartHM setingwifi");
  if (WiFi.status() == WL_CONNECTED){
      ESP.restart();
  }

}

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    struct_message newData;
    memcpy(&newData, incomingData, sizeof(newData));

    bool send_A = false, send_AC = false;

    // ตรวจสอบค่ามุม (kalAngleY) ว่าเปลี่ยนแปลงจากสถานะ 0 หรือ 1
    if (newData.kalAngleY != receivedData.kalAngleY || newData.kalAngleX != receivedData.kalAngleX) {
        send_A_flag = true; // มุม Y เปลี่ยนแปลง
    }

    // ตรวจสอบค่าความเร่ง (Accelerer) ว่าเปลี่ยนแปลงจากสถานะ 0 หรือ 1
    if (newData.Accelerer != receivedData.Accelerer) {
        send_AC_flag = true; // ความเร่งเปลี่ยนแปลง
    }
    // อัปเดตค่าล่าสุด
    receivedData = newData;

    // ตั้ง Flag ให้ loop() ส่งข้อมูลแทน
    if (send_A_flag || send_AC_flag) {
        needToSendData = true;
    }
}

void setup() {
    Serial.begin(115200);
    ss.begin(9600, SERIAL_8N1, 16, 17); // GPS: TX=16, RX=17
    WiFi.mode(WIFI_STA);
    pinMode(V_L, OUTPUT);
    pinMode(V_R, OUTPUT);
    pinMode(B_Newwifi, INPUT_PULLUP);
    pinMode(B_shutdown, INPUT_PULLUP);
    pinMode(buzzerPin, OUTPUT);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)B_shutdown, LOW);

    tone(buzzerPin, NOTE_C4, 1000); // เสียงตอนเริ่ม
    delay(1000); // รอให้ครบเวลา (หรือใช้ millis() ถ้าต้องการ non-blocking)
    noTone(buzzerPin);
    
    if (esp_now_init() != 0) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);

}

void sendDataToServer(bool D, bool A, bool AC) {
    if(WiFi.status() != WL_CONNECTED){
        WIFI.autoConnect("SmartHM setingwifi");
    }

    if (WiFi.status() == WL_CONNECTED) {
        //WiFiClientSecure client;
        //client.setCACert(root_ca);  // ❗ ปิดการตรวจสอบใบรับรอง SSL (เหมือนกับ http.setInsecure()) ค่ะ

        HTTPClient http;
        http.begin(SERVER_URL);
        http.addHeader("Content-Type", "application/json");
        http.addHeader("User-Agent", "ESP32Client");

        String jsonPayload = "{";
        if (D) {
            jsonPayload += "\"distance_L\": " + String(count_R) + ",";
            jsonPayload += "\"distance_R\": " + String(count_L) + ",";
            // ลบเครื่องหมาย , ตัวสุดท้าย
            jsonPayload.remove(jsonPayload.length() - 1);

            jsonPayload += "}";

            int httpResponseCode = http.POST(jsonPayload);
            Serial.print("Server Response: ");
            Serial.println(httpResponseCode);
            http.end();
        }
        if (A) {
            String jsonPayload = "{";
            jsonPayload += "\"kalAngle\": \"🚑เกิดอุบัติเหตุรถล้ม🚑\",";
            jsonPayload += "\"lat_GPS\": " + String(lat_GPS, 6) + ",";
            jsonPayload += "\"lng_GPS\": " + String(lng_GPS, 6);
            jsonPayload += "}";

            int httpResponseCode = http.POST(jsonPayload);
            Serial.print("Server Response: ");
            Serial.println(httpResponseCode);
            http.end();
            ESP.restart();
        }
        if (AC) {
            String jsonPayload = "{";
            jsonPayload += "\"Accelerer\": \"🚨เกิดอุบัติเหตุรถชน🚨\",";
            jsonPayload += "\"lat_GPS\": " + String(lat_GPS, 6) + ",";
            jsonPayload += "\"lng_GPS\": " + String(lng_GPS, 6);
            jsonPayload += "}";

            int httpResponseCode = http.POST(jsonPayload);
            Serial.print("Server Response: ");
            Serial.println(httpResponseCode);
            http.end();
            ESP.restart();
        }

        
    } else {
        Serial.println("❌ WiFi Disconnected! Cannot send data.");
    }

}



void loop() {
    unsigned long currentMillis = millis();
    if (digitalRead(B_shutdown) == LOW) {
        if (!Nub_BWF && (currentMillis - lastButtonPressTime > debounceDelay)) {
            count_BWF++;
            Nub_BWF = true;
            lastButtonPressTime = currentMillis;
        }
    } else {
        Nub_BWF = false;
    }

    if (ss.available() > 0) {
        gps.encode(ss.read());
    
        if (gps.location.isUpdated()) {
        Serial.print("ละติจูด: ");
        Serial.println(gps.location.lat(), 6);
        lat_GPS = gps.location.lat();
        
        Serial.print("ลองจิจูด: ");
        Serial.println(gps.location.lng(), 6);
        lng_GPS = gps.location.lng();
        
        Serial.print("ความเร็ว (km/h): ");
        Serial.println(gps.speed.kmph());
        speed_GPS = gps.speed.kmph();

        Serial.println("-----------------------------");
        }

    }

    if( digitalRead(B_Newwifi) == LOW ){
      New_wifiuser();
      }

    if ((count_BWF%2) != 0){
      tone(buzzerPin, NOTE_E4, 1000);  // เสียง C4
      WIFI.setConfigPortalTimeout(5);
      WIFI.autoConnect("SmartHM setingwifi");
      if (WiFi.status() != WL_CONNECTED){
        esp_deep_sleep_start();
      }else{
          sendDataToServer(1, 0, 0);
          esp_deep_sleep_start();
      }

    }

    if (needToSendData) {
        sendDataToServer(0, send_A_flag, send_AC_flag);
        needToSendData = false;
        send_A_flag = false;
        send_AC_flag = false;
    }
   
    if (receivedData.distance_L < 100 && receivedData.distance_L != -1) {
        if (currentMillis - previousMillis_L >= interval) {
            previousMillis_L = currentMillis;
            VBState_L = !VBState_L;
            digitalWrite(V_L, VBState_L);
            if (!Nub_L){
              count_L++;
              Nub_L = true;
            }
        }
    } else {
        digitalWrite(V_L, LOW);
        Nub_L = false;  // รีเซ็ตเมื่อระยะกลับมาเกิน 100 cm
    }

    if (receivedData.distance_R < 100 && receivedData.distance_R != -1) {
        if (currentMillis - previousMillis_R >= interval) {
            previousMillis_R = currentMillis;
            VBState_R = !VBState_R;
            digitalWrite(V_R, VBState_R);
            if (!Nub_R){
              count_R++;
              Nub_R = true;
            }
        }
    } else {
        digitalWrite(V_R, LOW);
        Nub_R = false;  // รีเซ็ตเมื่อระยะกลับมาเกิน 100 cm
    }

}
