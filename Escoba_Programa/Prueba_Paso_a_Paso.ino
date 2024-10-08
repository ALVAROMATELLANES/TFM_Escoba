#include "FS.h"
#include <LittleFS.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <cstring> // Para usar memcpy
#include <WiFi.h>
#include <time.h>
#include <TinyPICO.h>


// Definición de los pines del ADC para cada sensor FSR
#define ADC_FSR_1 32 // Rojo - Superior
#define ADC_FSR_2 33 // Verde - Superior
#define ADC_FSR_3 27 // Azul - Superior
#define ADC_FSR_4 15 // Rojo - Inferior
#define ADC_FSR_5 14 // Verde - Inferior
#define ADC_FSR_6 4  // Azul - Inferior
#define BUFFER_SIZE 21  // Tamaño del buffer óptimo para una FLASH de 256 bytes de tamaño de pagina


#define FORMAT_LITTLEFS_IF_FAILED true  
#define LowBatteryLevel 3.7
#define CriticalBatteryLevel 3.5
#define PIN_PULSADOR 21
#define MAX_RETRIES 10

struct DataStruct {
    uint16_t FSR1, FSR2, FSR3, FSR4, FSR5, FSR6;
};

struct TimeStruct {
    int year;
    uint8_t month;
    uint8_t monthDay;
    uint8_t hours;
    uint8_t min;
    uint8_t sec;
    uint16_t mSec;
    uint32_t offset;
    char event[6];
};

const char* timeFilePath = "/timeData.bin";
const char* sensorFilePath = "/sensorData.bin";

DataStruct dataBuffer[BUFFER_SIZE];
uint16_t n_muestras = 0;
SemaphoreHandle_t bufferSemaphore;

TaskHandle_t adquisicionTaskHandle = NULL;
TaskHandle_t escrituraTaskHandle = NULL;

//const char* ssid = "Orange-5D6C";
//const char* password = "fsz6mcnZ";
const char* ssid = "Mate";
const char* password = "minicrak6";
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;   //Poner a 3600 en horario de verano y 0 resto del año

bool acquiring = false;
TinyPICO tp = TinyPICO();

unsigned long ntpToFirstSampleStart = 0;
unsigned long lastSampleToNtpEnd = 0;

void resetADC2() {
    // Reconfigurar los canales de ADC2 después de desactivar el WiFi
    adc2_config_channel_atten(ADC2_CHANNEL_7, ADC_ATTEN_DB_11); // GPIO27 - FSR 3
    adc2_config_channel_atten(ADC2_CHANNEL_3, ADC_ATTEN_DB_11); // GPIO15 - FSR 4
    adc2_config_channel_atten(ADC2_CHANNEL_6, ADC_ATTEN_DB_11); // GPIO14 - FSR 5
    adc2_config_channel_atten(ADC2_CHANNEL_0, ADC_ATTEN_DB_11); // GPIO4  - FSR 6

    Serial.println("ADC2 reconfigurado.");
}

void setup() {
    Serial.begin(115200);
    // Configuración del ADC1
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11); // GPIO32 - FSR 1
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11); // GPIO33 - FSR 2

    // Configuración del ADC2
    adc2_config_channel_atten(ADC2_CHANNEL_7, ADC_ATTEN_DB_11); // GPIO27 - FSR 3
    adc2_config_channel_atten(ADC2_CHANNEL_3, ADC_ATTEN_DB_11); // GPIO15 - FSR 4
    adc2_config_channel_atten(ADC2_CHANNEL_6, ADC_ATTEN_DB_11); // GPIO14 - FSR 5
    adc2_config_channel_atten(ADC2_CHANNEL_0, ADC_ATTEN_DB_11); // GPIO4 - FSR 6

    pinMode(PIN_PULSADOR, INPUT_PULLDOWN);

    bufferSemaphore = xSemaphoreCreateMutex();

    // Intentar montar el sistema de archivos hasta que tenga éxito
    int retry_count = 0;
    bool mounted = false;
    while (!mounted && retry_count < MAX_RETRIES) {
        if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
            Serial.println("LittleFS Mount Failed. Retrying...");
            tp.DotStar_SetPixelColor(255, 0, 0);  // Red for error
            delay(1000);  // Longer delay between retries
            retry_count++;
        } else {
            Serial.println("LittleFS Mount Succeeded");
            tp.DotStar_SetPixelColor(0, 255, 0);  // Green for success
            mounted = true;
        }
    }
    if (!mounted) {
        Serial.println("Failed to mount LittleFS after max retries. Halting.");
        tp.DotStar_SetPixelColor(255, 0, 0);  // Solid red for critical error
        while (true);  // Halt the system
    }

    
    tp.DotStar_SetPixelColor(0, 255, 0); // Led verde
}

void loop() {
    static unsigned long lastDebounceTime = 0;
    static unsigned long lastPressTime = 0;
    static uint8_t buttonPressCount = 0;
    static bool lastButtonState = LOW;   
    static bool stableButtonState = LOW;
    static const unsigned long debounceDelay = 150;
    static const unsigned long multiPressInterval = 3000;

    bool currentButtonState = digitalRead(PIN_PULSADOR);
    unsigned long currentTime = millis();

    if (Serial.available() && !acquiring) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        if (command.equals("transmitir")) {
            transmitStructuredData();
        } else if (command.equals("Formatear")) {
            deleteFile(sensorFilePath);
            deleteFile(timeFilePath);
        } 
    }

    float BatteryLevel = tp.GetBatteryVoltage();
    if (BatteryLevel > CriticalBatteryLevel) {
        if (currentButtonState != lastButtonState) {
            lastDebounceTime = currentTime;
        } 

        if ((currentTime - lastDebounceTime) > debounceDelay) {
            if (currentButtonState != stableButtonState) {
                stableButtonState = currentButtonState;
                if (stableButtonState == HIGH) {
                    lastPressTime = currentTime;
                    buttonPressCount++;
                    Serial.print("Button pressed. Count: ");
                    Serial.println(buttonPressCount);
                }
            }
        }

        lastButtonState = currentButtonState;

        if (currentTime - lastPressTime > multiPressInterval && lastPressTime != 0) {
            if (buttonPressCount == 1 && !acquiring) {
                connectToWiFi();
                initNTP();
                ntpToFirstSampleStart = millis(); // Mover aquí el cálculo de inicio
                printLocalTime("start", true);
                xTaskCreatePinnedToCore(adquisicionTask, "AdquisicionTask", 2048, NULL, 1, &adquisicionTaskHandle, 0);
                xTaskCreatePinnedToCore(escrituraTask, "EscrituraTask", 4096, NULL, 1, &escrituraTaskHandle, 1);
                acquiring = true;
                Serial.println("Adquisición iniciada");
                if (BatteryLevel > LowBatteryLevel) {
                    tp.DotStar_SetPixelColor(0, 0, 255);  // Led azul
                } else {
                    tp.DotStar_SetPixelColor(255, 0, 0);  // Led rojo
                }
            } else if (buttonPressCount == 1 && acquiring) {
                Serial.println("Adquisición ya en curso");
            } else if (buttonPressCount > 2 && acquiring) {
                if (adquisicionTaskHandle != NULL) {
                    vTaskDelete(adquisicionTaskHandle);
                    adquisicionTaskHandle = NULL;
                }
                if (escrituraTaskHandle != NULL) {
                    vTaskDelete(escrituraTaskHandle);
                    escrituraTaskHandle = NULL;
                }
                lastSampleToNtpEnd = millis(); 
                acquiring = false;
                connectToWiFi();
                initNTP();
                printLocalTime("stop", false);
                Serial.println("Adquisición detenida");
                guardarDatosPendientes();
                if (BatteryLevel > LowBatteryLevel) {
                    tp.DotStar_SetPixelColor(0, 255, 0); // Led verde
                } else {
                    tp.DotStar_SetPixelColor(255, 0, 0);  // Led rojo
                }
            } else if (buttonPressCount > 2 && !acquiring) {
                Serial.println("Adquisición no está activa");
            }

            buttonPressCount = 0;
            lastPressTime = 0;
        }

    } else {
        if (adquisicionTaskHandle != NULL) {
            vTaskDelete(adquisicionTaskHandle);
            adquisicionTaskHandle = NULL;
        }
        if (escrituraTaskHandle != NULL) {
            vTaskDelete(escrituraTaskHandle);
            escrituraTaskHandle = NULL;
        }
        bool charging = tp.IsChargingBattery();
        while(!charging){
          tp.DotStar_SetPixelColor(255, 0, 0);  // Led rojo
          delay(500);
          tp.DotStar_Clear();  
          delay(500);
        } 
    }
}

void adquisicionTask(void *pvParameters) {
    const int num_muestras = 10;  // Número de muestras para el sobremuestreo
    while (1) {
        if (xSemaphoreTake(bufferSemaphore, portMAX_DELAY) == pdTRUE) {
            if (n_muestras < BUFFER_SIZE) {
                // Variables para almacenar las lecturas temporales
                int sum_FSR1 = 0, sum_FSR2 = 0, sum_FSR3 = 0, sum_FSR4 = 0, sum_FSR5 = 0, sum_FSR6 = 0;
                int adc2_value;
                 WiFi.mode(WIFI_OFF); // Apaga el WiFipara liberar el ADC2
                  while (WiFi.getMode() != WIFI_OFF) {
                    delay(1); // Espera activa, ajusta según sea necesario
                  }
                // Sobremuestreo
                for (int i = 0; i < num_muestras; i++) {
                    sum_FSR1 += adc1_get_raw(ADC1_CHANNEL_4);  // GPIO32
                    sum_FSR2 += adc1_get_raw(ADC1_CHANNEL_5);  // GPIO33
                    
                    adc2_get_raw(ADC2_CHANNEL_7, ADC_WIDTH_BIT_12, &adc2_value);
                    sum_FSR3 += adc2_value;  // GPIO27

                    adc2_get_raw(ADC2_CHANNEL_3, ADC_WIDTH_BIT_12, &adc2_value);
                    sum_FSR4 += adc2_value;  // GPIO15

                    adc2_get_raw(ADC2_CHANNEL_6, ADC_WIDTH_BIT_12, &adc2_value);
                    sum_FSR5 += adc2_value;  // GPIO14

                    adc2_get_raw(ADC2_CHANNEL_0, ADC_WIDTH_BIT_12, &adc2_value);
                    sum_FSR6 += adc2_value;  // GPIO4
                }

                // Promedio de las muestras
                dataBuffer[n_muestras].FSR1 = sum_FSR1 / num_muestras;
                dataBuffer[n_muestras].FSR2 = sum_FSR2 / num_muestras;
                dataBuffer[n_muestras].FSR3 = sum_FSR3 / num_muestras;
                dataBuffer[n_muestras].FSR4 = sum_FSR4 / num_muestras;
                dataBuffer[n_muestras].FSR5 = sum_FSR5 / num_muestras;
                dataBuffer[n_muestras].FSR6 = sum_FSR6 / num_muestras;
                n_muestras++;

            }
            xSemaphoreGive(bufferSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Simula tiempo de adquisición
    }
}

void escrituraTask(void *pvParameters) {
    DataStruct localBuffer[BUFFER_SIZE];  // Buffer local para almacenar datos temporalmente

    while (1) {
        if (xSemaphoreTake(bufferSemaphore, portMAX_DELAY) == pdTRUE) {
            if (n_muestras == BUFFER_SIZE) {
                // Copiar datos al buffer local
                memcpy(localBuffer, dataBuffer, sizeof(localBuffer));
                n_muestras = 0;  // Reiniciar el contador de muestras
                xSemaphoreGive(bufferSemaphore);  // Liberar el semáforo inmediatamente después de copiar

                // Proceder con la escritura del archivo sin retener el semáforo
                File file = LittleFS.open(sensorFilePath, FILE_APPEND);
                if (file) {
                    size_t writeSize = sizeof(localBuffer);
                    if (file.write((uint8_t*)localBuffer, writeSize) != writeSize) {
                        Serial.println("Error writing data to file");
                    } else {
                        //Serial.println("Data successfully written to file");
                    }
                    file.close();
                } else {
                    Serial.println("Failed to open file for writing");
                }
            } else {
                xSemaphoreGive(bufferSemaphore);  // Asegurarse de liberar el semáforo si no hay suficientes datos
            }
        }
        vTaskDelay(pdMS_TO_TICKS(250));  // Simula tiempo de adquisición
    }
}


void deleteFile(const char* path) {
    if (LittleFS.remove(path)) {
        Serial.println("File successfully deleted");
    } else {
        Serial.println("Failed to delete file");
    }
}

void transmitStructuredData() {
    File filetime = LittleFS.open(timeFilePath, "r");
    if (!filetime) {
        Serial.println("Failed to open time file for reading");
        return;
    }

    TimeStruct timeData;
    Serial.println("Time data:");
    while (filetime.read((uint8_t*)&timeData, sizeof(TimeStruct)) == sizeof(TimeStruct)) {
        Serial.print(timeData.year); Serial.print("-");
        Serial.print(timeData.month); Serial.print("-");
        Serial.print(timeData.monthDay); Serial.print("T");
        Serial.print(timeData.hours); Serial.print(":");
        Serial.print(timeData.min); Serial.print(":");
        Serial.print(timeData.sec); Serial.print(".");
        Serial.print(timeData.mSec); Serial.println("+01:00");
        Serial.print("Offset: "); Serial.print(timeData.offset); Serial.print(" ms ");
        Serial.println(timeData.event);
    }
    filetime.close();

    File file = LittleFS.open(sensorFilePath, "r");
    if (!file) {
        Serial.println("Failed to open sensor file for reading");
        return;
    } 

    DataStruct data;
    Serial.println("Datos de las FSR:");
    Serial.print("FSR1; "); Serial.print("FSR2; "); Serial.print("FSR3; "); Serial.print("FSR4; "); Serial.print("FSR5; "); Serial.println("FSR6 ");
    while (file.read((uint8_t*)&data, sizeof(DataStruct)) == sizeof(DataStruct)) {
        Serial.print(data.FSR1);
        Serial.print("; "); Serial.print(data.FSR2);
        Serial.print("; "); Serial.print(data.FSR3);
        Serial.print("; "); Serial.print(data.FSR4);
        Serial.print("; "); Serial.print(data.FSR5);
        Serial.print("; "); Serial.println(data.FSR6);
    }
    file.close();
}


void connectToWiFi() {
    Serial.print("Connecting to WiFi..");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("Connected.");
}

void initNTP() {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    Serial.println("NTP time set");
}

void printLocalTime(const char* event, bool isStart) {
    unsigned long offsetMillis = millis();

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        return;
    }

    struct timeval tv;
    gettimeofday(&tv, NULL);

    TimeStruct dataTime;

    dataTime.year = 1900 + timeinfo.tm_year;
    dataTime.month = timeinfo.tm_mon + 1;
    dataTime.monthDay = timeinfo.tm_mday;
    dataTime.hours = timeinfo.tm_hour;
    dataTime.min = timeinfo.tm_min;
    dataTime.sec = timeinfo.tm_sec;
    dataTime.mSec = tv.tv_usec / 1000;
    strncpy(dataTime.event, event, sizeof(dataTime.event));

    WiFi.mode(WIFI_OFF); // Apaga el WiFipara liberar el ADC2
    while (WiFi.getMode() != WIFI_OFF) {
        delay(1); // Espera activa, ajusta según sea necesario
    }

    resetADC2();

    if (isStart) {
        dataTime.offset = millis() - offsetMillis;  // Time from NTP to first sample
    } else {
        dataTime.offset = offsetMillis - lastSampleToNtpEnd;  // Time from last sample to NTP
    }

    Serial.print("Event: "); Serial.println(event);
    File file = LittleFS.open(timeFilePath, FILE_APPEND);
    if (file) {
        if (file.write((uint8_t*)&dataTime, sizeof(TimeStruct)) != sizeof(TimeStruct)) {
            Serial.println("Error writing data to file");
        }
        file.close();
    } else {
        Serial.println("Failed to open file for writing");
    }
}

void guardarDatosPendientes() {
    if (xSemaphoreTake(bufferSemaphore, portMAX_DELAY) == pdTRUE) {
        if (n_muestras > 0) {  // Si hay datos pendientes en el buffer
            DataStruct localBuffer[BUFFER_SIZE];
            memcpy(localBuffer, dataBuffer, sizeof(DataStruct) * n_muestras);
            
            File file = LittleFS.open(sensorFilePath, FILE_APPEND);
            if (file) {
                size_t writeSize = sizeof(DataStruct) * n_muestras;
                if (file.write((uint8_t*)localBuffer, writeSize) != writeSize) {
                    Serial.println("Error writing pending data to file");
                } else {
                    Serial.println("Pending data successfully written to file");
                }
                file.close();
            } else {
                Serial.println("Failed to open file for writing pending data");
            }
            n_muestras = 0;  // Reiniciar el contador de muestras
        }
        xSemaphoreGive(bufferSemaphore);
    }
}

