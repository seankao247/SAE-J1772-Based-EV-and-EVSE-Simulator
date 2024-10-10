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
bool MQTT_ready = false;
int en_charge_state = LOW;
int buttonState;             // the current reading from the input pin
int lastButtonState = HIGH;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;

const int buttonPin = 18;    // the number of the pushbutton pin
float pwm_voltage = 0;
const int pwmPin = 4;  // PWM訊號輸入腳位
const int en_charge = 16;  // en_charge訊號輸入腳位
bool connect = false;

// WiFi
const char *ssid = "NEAT_2.4G";     // Enter your WiFi name
const char *password = "221b23251"; // Enter WiFi password

// MQTT Broker
const char *mqtt_broker = "192.168.1.169";
const char *topic = "esp32/test";
const char *mqtt_username = "Sim_EV1";
const char *mqtt_password = "Sim_EV1";
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
  // client.publish(topic, "[EV] MQTT ready");
  // oled_display("MQTT ready");
  // delay(2000);
  client.subscribe(topic);
  MQTT_ready = true;
}

void dutycyclefunc(void *pvParameters)
{
  int pwmduty ;
  delay(1000);
  char EV_State='X';
  float dutycycle,MaxCurrent;
  String show;
  for (;;) // 無限循環
  {
      pwmduty = pulseIn(pwmPin, HIGH, 100000);
      Serial.println(pwmduty);
      
      dutycycle = pwmduty%1000 ;
      MaxCurrent = dutycycle*0.06;
      Serial.print("MaxCurrent");
      Serial.println(MaxCurrent);
      if(!connect){
        if(EV_State!='A'){
          show = "[Status A] Standby" ;
          String strpub = "["+String(mqtt_username) +"]"+ "Status A Standby" ;
          if (MQTT){
            if(client.publish(topic, strpub.c_str()))EV_State='A';
          }
          else {
            Serial.println(strpub);
            EV_State='A';
          }
        }    
      }
      else{
          if(en_charge_state == LOW && EV_State!='B'){
            show = "[Status B] Connected"  ;
            String strpub = "["+String(mqtt_username) +"]"+ " Status B Connected" ;
            if (MQTT){
              if(client.publish(topic, strpub.c_str()))EV_State='B';
            }
            else {
            Serial.println(strpub);
            EV_State='B';
            }
          }
          else if(en_charge_state == HIGH ){
            show = "[StatusC] Max:" + String(MaxCurrent) + "A" ;
            String strpub = "["+String(mqtt_username) +"]"+" Status C Charging Max:" +String(MaxCurrent) + "A" ;
            if (MQTT){
                if(client.publish(topic, strpub.c_str()))EV_State='C';
            }
            else {
                Serial.println(strpub);
                EV_State='C';
            }
            if (MaxCurrent==0)en_charge_state=LOW;
          }
      }
      
      oled_display(show);
        vTaskDelay(500 / portTICK_PERIOD_MS);
      
  }
}

void btnctlfunc(void *pvParameters)
{
  int reading;
  for (;;) // 無限循環
  {
    // read the state of the switch into a local variable:
    reading = digitalRead(buttonPin);

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH), and you've waited long enough
    // since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
      // whatever the reading is at, it's been there for longer than the debounce
      // delay, so take it as the actual current state:

      // if the button state has changed:
      if (reading != buttonState) {
        buttonState = reading;

        // only toggle the LED if the new button state is HIGH
        if (buttonState == LOW) {
          en_charge_state = !en_charge_state;
        }
      }
    }

    // set the LED:
    digitalWrite(en_charge, en_charge_state);
    // if(en_charge_state==HIGH){
    //   Serial.println("Charging");
    // }
    // else{
    //   Serial.println("Connect");
    // }

    // save the reading. Next time through the loop, it'll be the lastButtonState:
    lastButtonState = reading;
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
void readPPTask(void *pvParameters)
{
  for (;;) // 無限循環
  { 
    int analogValue=adc1_get_raw(ADC1_CHANNEL_5);
    //adc2_get_raw(ADC2_CHANNEL_4, ADC_WIDTH_12Bit, &analogValue);
    float V=(analogValue * 3.3) / 4095.0;
    // Serial.print("V=");
    // Serial.println((analogValue * 3.3) / 4095.0);
    // int analogValue = analogRead(PWM_IN); // 讀取 PWM 訊號的輸入值
    if (V >3){
        // Serial.println("connect = false");
        connect = false;
    }
    else if(V>2 && V <3){  //閘鎖按下
      connect = false;
    }
    else if(V>1 && V <2){  //閘鎖按下
      // Serial.println("connect = true");
      connect = true;
    }
    // else{
    //   Serial.println("PP error!!");
    //   connect = false;
    // }

    vTaskDelay(500 / portTICK_PERIOD_MS); // 延遲 10 毫秒
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
void setup() {
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_5,ADC_ATTEN_DB_11);
  pinMode(buttonPin, INPUT_PULLUP); // 設置GPIO18為輸入模式，並啟用內部上拉電阻
  pinMode(pwmPin, INPUT);  // 設定腳位為輸入模式
  pinMode(en_charge, OUTPUT);
  digitalWrite(en_charge, en_charge_state);
  Serial.begin(115200);      // 初始化序列埠

  // initialize OLED display with I2C address 0x3C
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("failed to start SSD1306 OLED"));
  }
  else{
    delay(2000);         // wait two seconds for initializing
    if(MQTT)oled_display("waiting MQTT...");
  }
  xTaskCreatePinnedToCore(dutycyclefunc, "dutycyclefunc", 2000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(btnctlfunc, "btnctlfunc", 1000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(readPPTask, "readPPTask", 2000, NULL, 2, NULL, 0);
  if (MQTT) xTaskCreatePinnedToCore(mqtt, "mqttTask", 3000, NULL, 2, NULL, 1);
}

void loop() {
  //
}



