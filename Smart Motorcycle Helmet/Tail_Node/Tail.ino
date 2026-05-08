#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Kalman.h"
#include "config.h"

#define TRIG_L D5
#define ECHO_L D6
#define TRIG_R D7
#define ECHO_R D8

// โครงสร้างข้อมูลที่ส่งไป
typedef struct struct_message {
    float distance_L;
    float distance_R;
    float kalAngleX;  // เพิ่มค่ามุมจาก Kalman Filter
    float kalAngleY;  // เพิ่มค่ามุมจาก Kalman Filter
    float Accelerer;
} struct_message;

struct_message dataToSend; // สร้างตัวแปรเก็บค่า

unsigned long previousMillisMPU = 0;  // ตัวแปรเก็บเวลาของ MPU6050
unsigned long previousMillisUltrasonic = 0;  // ตัวแปรเก็บเวลาของ Ultrasonic sensor
const long intervalMPU = 10;           // ตั้งเวลาให้รีเฟรชข้อมูลจาก MPU6050 ทุกๆ 10 ms
const long intervalUltrasonic = 500;   // ตั้งเวลาให้รีเฟรชข้อมูลจาก Ultrasonic sensor ทุกๆ 500 ms
unsigned long currentMillis;

Adafruit_MPU6050 mpu;
Kalman kalmanX, kalmanY;  // Kalman filter สำหรับแกน X และ Y
Kalman kalmanAccX, kalmanAccY, kalmanAccZ; // Kalman filter สำหรับค่าความเร่ง

float accAngleX, accAngleY; // มุมจาก Accelerometer
float kalAngleX, kalAngleY; // มุมจาก Kalman Filter
float accX_filtered, accY_filtered, accZ_filtered; // ค่าความเร่งที่ผ่าน Kalman Filter
float lastAccX = 0, lastAccY = 0, lastAccZ = 0; // เก็บค่าความเร่งก่อนหน้า
const float IMPACT_THRESHOLD = 90.0;
float last_kalAngleX = 0;
float last_kalAngleY = 0;
float last_Accelerer = 0;

// ใส่ MAC Address ของ ESP8266 ที่จะรับข้อมูล
uint8_t receiverMAC[6];

// ฟังก์ชันเมื่อส่งข้อมูลสำเร็จหรือไม่สำเร็จ
void OnDataSent(uint8_t *mac_addr, uint8_t status) {
    Serial.print("Delivery Status: ");
    Serial.println(status == 0 ? "Success" : "Fail");
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA); // ตั้งค่า Wi-Fi เป็นโหมด Station
    while (!Serial);

    // เริ่มต้น I2C สำหรับ ESP8266 (ใช้ Wire.h ปกติได้)
    Wire.begin(4, 5); // กำหนดขา SDA และ SCL (GPIO4 เป็น SDA, GPIO5 เป็น SCL)

    // เริ่มต้น MPU6050
    mpu.begin();

    // เริ่มต้น ESP-NOW
    if (esp_now_init() != 0) { // ESP8266 ใช้ 0 แทน ESP_OK
        Serial.println("ESP-NOW Init Failed");
        return;
    }
    
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER); // กำหนดบทบาทเป็น Controller
    esp_now_register_send_cb(OnDataSent); // ตั้งค่าฟังก์ชัน callback


    // จับคู่กับ ESP8266 ที่รับข้อมูล
    memcpy(receiverMAC, RECEIVER_MAC, 6); // ช่อง 1, ไม่มีการเข้ารหัส

    // เพิ่มบรรทัดนี้กลับเข้าไปนะคะเหมี่ยว
    esp_now_add_peer(receiverMAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
    
    // ตั้งค่าค่าเริ่มต้นของ Kalman Filter
    kalmanX.setAngle(0);  
    kalmanY.setAngle(0);
    kalmanAccX.setAngle(0);
    kalmanAccY.setAngle(0);
    kalmanAccZ.setAngle(0);

    // ตั้งค่าการทำงานของเซ็นเซอร์
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    pinMode(TRIG_L, OUTPUT);
    pinMode(ECHO_L, INPUT);
    pinMode(TRIG_R, OUTPUT);
    pinMode(ECHO_R, INPUT);
}

void loop() {
    currentMillis = millis(); // เก็บเวลาปัจจุบันจาก millis()

    // ตรวจสอบเวลาในการรีเฟรชข้อมูลจาก MPU6050 (ทุกๆ 50ms)
    if (currentMillis - previousMillisMPU >= intervalMPU) {
        previousMillisMPU = currentMillis;  // เก็บเวลาปัจจุบันเพื่อใช้ในการคำนวณเวลาครั้งถัดไป

        // อ่านค่าจาก MPU6050
        sensors_event_t accel, gyro, temp;
        mpu.getEvent(&accel, &gyro, &temp);

        accX_filtered = kalmanAccX.getAngle(accel.acceleration.x, 0, 0.01);
        accY_filtered = kalmanAccY.getAngle(accel.acceleration.y, 0, 0.01);
        accZ_filtered = kalmanAccZ.getAngle(accel.acceleration.z, 0, 0.01);

        float rateX = abs(accX_filtered - lastAccX) / 0.01; // อัตราการเปลี่ยนแปลงของค่าความเร่ง X
        float rateY = abs(accY_filtered - lastAccY) / 0.01; // อัตราการเปลี่ยนแปลงของค่าความเร่ง Y
        float rateZ = abs(accZ_filtered - lastAccZ) / 0.01; // อัตราการเปลี่ยนแปลงของค่าความเร่ง Z

        float maxRate = max(rateX, max(rateY, rateZ));

        // คำนวณมุมจาก Accelerometer
        accAngleX = atan2(accel.acceleration.y, accel.acceleration.z) * RAD_TO_DEG;
        accAngleY = atan2(-accel.acceleration.x, accel.acceleration.z) * RAD_TO_DEG;

        // คำนวณมุมจาก Gyroscope
        float gyroXrate = gyro.gyro.x * RAD_TO_DEG;
        float gyroYrate = gyro.gyro.y * RAD_TO_DEG;

        // ใช้ Kalman Filter เพื่อกรองค่ามุม
        kalAngleX = kalmanX.getAngle(accAngleX, gyroXrate, 0.01);
        kalAngleY = kalmanY.getAngle(accAngleY, gyroYrate, 0.01);

        // เช็คเงื่อนไขการส่งข้อมูลเมื่อมุมมากกว่า 65 หรือ น้อยกว่า -65
         if (kalAngleX <= -65 || kalAngleX >= 65 || kalAngleY <= -65 || kalAngleY >= 65) {
            dataToSend.kalAngleX = 1;
            dataToSend.kalAngleY = 1;
            if( last_kalAngleX != dataToSend.kalAngleX && last_kalAngleY != dataToSend.kalAngleY){
              esp_now_send(receiverMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));
              Serial.println("Kalman X");
              Serial.print(kalAngleX);
              Serial.println("Kalman Y");
              Serial.print(kalAngleY);
              last_kalAngleX = dataToSend.kalAngleX;
              last_kalAngleY = dataToSend.kalAngleY;
            }
         }
         else{
            dataToSend.kalAngleX = 0;
            dataToSend.kalAngleY = 0;
            if( last_kalAngleX != dataToSend.kalAngleX && last_kalAngleY != dataToSend.kalAngleY){
              esp_now_send(receiverMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));
              Serial.println("Kalman X");
              Serial.print(kalAngleX);
              Serial.println("Kalman Y");
              Serial.print(kalAngleY);
              last_kalAngleX = dataToSend.kalAngleX;
              last_kalAngleY = dataToSend.kalAngleY;
            }
         }

         if (maxRate > IMPACT_THRESHOLD) {
            dataToSend.Accelerer = 1;
            if( dataToSend.Accelerer != last_Accelerer ){
              esp_now_send(receiverMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));
              Serial.println("⚠️ Impact Detected! 🚨");
              Serial.print(maxRate);
              last_Accelerer = dataToSend.Accelerer;
            }
         }
         else{
            dataToSend.Accelerer = 0;
            if( dataToSend.Accelerer != last_Accelerer ){
              esp_now_send(receiverMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));
              Serial.print(maxRate);
              last_Accelerer = dataToSend.Accelerer;
            }
         }
        lastAccX = accX_filtered;
        lastAccY = accY_filtered;
        lastAccZ = accZ_filtered;
    }

    // ตรวจสอบเวลาในการรีเฟรชข้อมูลจาก Ultrasonic sensor (ทุกๆ 500ms)
    if (currentMillis - previousMillisUltrasonic >= intervalUltrasonic) {
        previousMillisUltrasonic = currentMillis;  // เก็บเวลาปัจจุบันเพื่อใช้ในการคำนวณเวลาครั้งถัดไป

        // วัดระยะทางด้านซ้าย
        digitalWrite(TRIG_L, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_L, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_L, LOW);
        float duration_L = pulseIn(ECHO_L, HIGH, 38000);
        dataToSend.distance_L = (duration_L == 0) ? -1 : duration_L / 58.0;

        // วัดระยะทางด้านขวา
        digitalWrite(TRIG_R, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_R, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_R, LOW);
        float duration_R = pulseIn(ECHO_R, HIGH, 38000);
        dataToSend.distance_R = (duration_R == 0) ? -1 : duration_R / 58.0;

        // ส่งข้อมูลไปยัง ESP8266
        esp_now_send(receiverMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));

        // แสดงข้อมูลที่วัดได้
        Serial.print("Distance Left: ");
        Serial.print(dataToSend.distance_L);
        Serial.print(" cm | Distance Right: ");
        Serial.print(dataToSend.distance_R);
        Serial.println(" cm");
    }
}

