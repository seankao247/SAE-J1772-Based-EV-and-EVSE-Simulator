#include <Arduino.h>

#include <driver/adc.h>
#include <driver/i2s.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <PubSubClient.h>
#define MQTT true
#define SCREEN_WIDTH 128 // OLED width,  in pixels
#define SCREEN_HEIGHT 64 // OLED height, in pixels
// create an OLED display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


TaskHandle_t pwmTask;   // 定義 PWM task 的 handle
TaskHandle_t readTask;  // 定義讀取 task 的 handle
TaskHandle_t printTask; // 定義print task 的 handle

const int PWM_OUT = 14;        // 設置 PWM 輸出引腳為 GPIO14
const int PWM_FREQ = 1000;    // 設置 PWM 頻率為 1 kHz
const int PWM_CHANNEL = 0;    // 設置 PWM 訊號通道為 0
const int PWM_RESOLUTION = 8; // 設置 PWM 分辨率為 8 位
const int OUT1 = 17;
const int OUT2 = 16;
//const int PWM_IN = 12;        // 設置 PWM 輸入引腳為 GPIO12
float pwm_voltage = 0;
int PWM_current = 60;
bool MQTT_ready = false;
// WiFi
const char *ssid = "NEAT_2.4G";     // Enter your WiFi name
const char *password = "221b23251"; // Enter WiFi password

// MQTT Broker   
const char *mqtt_broker = "192.168.1.169";
const char *topic = "esp32/test";
const char *mqtt_username = "Sim_EVSE1";
const char *mqtt_password = "Sim_EVSE1";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

void oled_display(String displayword)
{
  oled.clearDisplay(); // clear display

  oled.setTextSize(2);         // set text size
  oled.setTextColor(WHITE);    // set text color
  oled.setCursor(0, 10);       // set position to display
  oled.println(displayword); // set text
  oled.display();              // display on OLED
}

void connectWiFiAndMQTT()
{
  // connecting to a WiFi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  // connecting to a mqtt broker
  client.setServer(mqtt_broker, mqtt_port);
  while (!client.connected())
  {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password))
    {
      Serial.println("Public emqx mqtt broker connected");
    }
    else
    {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
  // publish and subscribe
  // client.publish(topic, "[EVSE] MQTT ready");
  // oled_display("[EVSE] MQTT ready");
  MQTT_ready = true;
  client.subscribe(topic);
}


void pwmTaskFunction(void *pvParameters)
{  
  int duty_int ;
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_OUT, PWM_CHANNEL);

  for (;;)
  {
    duty_int = round(4.25 * PWM_current);
    ledcWrite(PWM_CHANNEL, duty_int);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
void readPwmTask(void *pvParameters)
{
  for (;;) // 無限循環
  {
    int analogValue=adc1_get_raw(ADC1_CHANNEL_5);
    // adc2_get_raw( ADC2_CHANNEL_4, ADC_WIDTH_12Bit, &analogValue);
    //int analogValue = analogRead(PWM_IN); // 讀取 PWM 訊號的輸入值
    //Serial.println((analogValue * 3.3) / 4095.0);
    if (analogValue != 0)
    {
      if (analogValue > pwm_voltage) // 更新最大電壓值
      {

        pwm_voltage = analogValue;
      }
    }
    //vTaskDelay(100 / portTICK_PERIOD_MS); // 延遲 10 毫秒
  }
}
void cpvTaskFunction(void *pvParameters)
{
  delay(3000);
  float temp = 0;
  char status ='X';
  for (;;)
  {
    pwm_voltage = 0;
    delay(1000);
      temp = pwm_voltage;
      float adcvalue = (temp * 3.3) / 4095.0;
      Serial.println(adcvalue);
      if (adcvalue <1)
      {
        if(status != 'F'){
            String strpub = "["+String(mqtt_username) +"]"+ "Status F error" ;
            if (MQTT){
              if(client.publish(topic, strpub.c_str()))status='F';
            }
            else {
              Serial.println(strpub);
              status='F';
            }
            PWM_current = 60;
            oled_display("[Status F] error");
        }
        digitalWrite(OUT1, LOW);
        digitalWrite(OUT2, LOW);
      }
      else if (adcvalue > 1 && adcvalue < 2)
      {
        if(status != 'C'){
            String strpub = "["+String(mqtt_username) +"]"+ "Status C Charging" ;
            if (MQTT){
              if(client.publish(topic, strpub.c_str()))status='C';
            }
             else {
              Serial.println(strpub);
              status='C';
           }
            PWM_current = 10;
            oled_display("[Status C] Charging");
        }
        digitalWrite(OUT1, HIGH);
        digitalWrite(OUT2, HIGH);
      }
      else if (adcvalue > 2 && adcvalue < 3)
      {
        if(status != 'B'){
            String strpub = "["+String(mqtt_username) +"]"+ "Status B Vehicle detected" ;
            if (MQTT){
              if(client.publish(topic, strpub.c_str()))status='B';
            }
             else {
              Serial.println(strpub);
              status='B';
           }
            PWM_current = 10;
            oled_display("[Status B] Vehicle detected");
        }       
        digitalWrite(OUT1, LOW);
        digitalWrite(OUT2, LOW);
      }
      else if (adcvalue >= 3)
      {
        if(status != 'A'){
            String strpub = "["+String(mqtt_username) +"]"+ "Status A Standby" ;
            if (MQTT){
              if(client.publish(topic, strpub.c_str()))status='A';
            }
             else {
              Serial.println(strpub);
              status='A';
           }
            PWM_current = 60;
            oled_display("[Status A] Standby");
        }
        digitalWrite(OUT1, LOW);
        digitalWrite(OUT2, LOW);
      }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
void mqtt(void *pvParameters)
{
  connectWiFiAndMQTT();
  for (;;) // 無限循環
  {
    // 檢查 Wi-Fi 和 MQTT 連線狀態
    if (!client.connected())
    {
      // 重新連線
      connectWiFiAndMQTT();
    }

    // 處理 MQTT 客戶端的事件
    client.loop();
    
    // 加入延遲
    
    vTaskDelay(500 / portTICK_PERIOD_MS); // 500 毫秒的延遲
  }
}

void setup()
{
  Serial.begin(115200);   
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_5,ADC_ATTEN_DB_11);
  //Serial.println("[EVSE State] setup");
  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);
  digitalWrite(OUT1, LOW);
  digitalWrite(OUT2, LOW);
  analogReadResolution(12); // 設定 ADC 解析度為 12 位元

  // initialize OLED display with I2C address 0x3C
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("failed to start SSD1306 OLED"));
  }
  else{
    delay(2000);// wait two seconds for initializing
    if(MQTT)oled_display("[EVSE] waiting MQTT...");
  
  }

  xTaskCreatePinnedToCore(pwmTaskFunction, "pwmOutputTask", 1000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(readPwmTask, "pwmReadTask", 1000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(cpvTaskFunction, "cpTask", 2000, NULL, 1, NULL, 0);
  if (MQTT) xTaskCreatePinnedToCore(mqtt, "mqttTask", 3000, NULL, 2, NULL, 0);
}

void loop()
{
  //
}
