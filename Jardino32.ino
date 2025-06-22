/*
 * Jardino  MBC 2017-2022
 * Board :  NodeMCU-32S
 * 
 * /home/mbaz/Arduino/Sketchbook/Jardino32/Jardino32.ino.nodemcu-32s.bin
 * V 1.0
 * 
 *  8 Outputs
 *  6 non-volatile programs
 *  Non-volatile configuration
 *  LCD 20x4 + Rotary encoder for manual control 
 *  MQTT remote control (Android app)
 *  NTP time synchronization
 *  Output current monitoring
 *  WEB Weather correction 
 *  OTA firmware update
 *
 * V 1.1 ( 04/2019)
 *  ESP32
 *  WEB Weather correction (OpenWeatherMap API)
 *  ICACHE_RAM_ATTR needed
 *  new NTPClient
 *  
 * V 1.2 ( 05/2020)
 *   Change PIN numbers
 *   Disable current monitoring and control
 *   
 * V 1.3 ( 05/2021)
 *   New WiFi credentials
 *   Version included in info menu.
 *   
 * V 1.4 ( 2/2022)
 *   New WiFi credentials
 *   Fix info menu.
 *   
 * V 1.5 ( 5/2022)
 *   Random MQTT client id
 *   Uncomment resetModule call
 *   Loop MQTT each 50 ms ( minimum)
 *   Info Screen fix
 *   
 * V 1.6 ( 5/2022)
 * FIX: Schedule fails when correction is received and then durations are recalculated when watering is on
 * Change of correction criteria
 * g_correction added ( sched.c)
 * 
 * V 1.7 ( 8/2022)
 * Restart after  timeout if WiFi connection fails at startup (NOT IMPLEMENTED)
 * 
 * V 1.8 ( 5/2025)
 * New MQTT broker ( casa.splasma.es )
 * Fix: Blocking when MQTT is not available
 * 
 *  * V 1.9 ( 6/2025)
 * Correction is reduced to -75% ( from -100%, no irrigation)  when hard rain probability is greater than 80%
 */

#define VERSION "V1.9 2025"

/***************************************************/

#include <Arduino.h>   
#include <WiFi.h>
#include <DNSServer.h>        //Local DNS Server
#include <ESPmDNS.h>
#include <WebServer.h> //Local WebServer used to serve the configuration portal

#include <PubSubClient.h>
#include <EEPROM.h>
#include <TimeLib.h>
#include <Time.h>
#include <Timezone.h>
#include <Wire.h> 
//#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal_PCF8574.h>


// WiFi OTA update
#include <ArduinoOTA.h>
#include <ArduinoJson.h>

#include <driver/adc.h>


int CK_Test,DK_Test;


#include "esp_system.h"
#include "esp_task_wdt.h"

//#include "WundergroundClient.h"
#include "OpenWeatherMapCurrent.h"
#include "OpenWeatherMapForecast.h"
#include "types.h"

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;


/***************************************************/
//#define RESTORE_DEFAULT_CONFIG
#define DEBUGSERIAL Serial


//const char* ssid[]     = {"MOVISTAR_FC98","Wifi_Chalet"};
//const char* password[] = {"EvuCYxzHy3V47V3TH3Hv","mbazmbaz"};

const char* ssid[]     = {"Wifi_Chalet"};
const char* password[] = {"mbazmbaz"};

const int n_SSID = sizeof(ssid)/sizeof(const char *);

const char* wifi_ssid = NULL;

#define mqtt_server "casa.splasma.es"
#define mqtt_server_port 8883
#define mqtt_user "mbaz"
#define mqtt_password "Duende_8888"


const char* device_out_topic = "devices/jardino0/out";
const char* device_in_topic  = "devices/jardino0/in";


/**
 * Weather Open Map Settings
 */
const String  OWM_API_KEY = "44b2cfe2db022991128a20b22347f6a1";
const String  OWM_LAT= "40.697558";
const String  OWM_LON= "-3.932339";

// Lat,lon coordinates are used instead city location ( Cerceda)
/**
 * Wunderground Settings (DEPRECATED)
 */
 /*
const String  WUNDERGROUND_API_KEY = "7195dd88ada0e445";
const boolean IS_METRIC = true;
const String  WUNDERGROUND_LAT= "40.697558";
const String  WUNDERGROUND_LONG= "-3.932339";
const String  WUNDERGROUND_LANGUAGE = "ES";
*/


/* 
 *  NodeMCU-32S pinout map
 *  
*/

/*
   Node-MCU  
   I2C LCD 
     (SDA)
     (SCL)

     ROTARY
     (CK)
     (DT)             
     (Switch)
*/

#define LCD_PIN_SDA   23 // SDA i2C
#define LCD_PIN_SCL   18 // SCL i2c


const short int ENCODER_CK   = 32; // not PULLUP
const short int ENCODER_DT   = 33; // not PULLUP
const short int ENCODER_PUSH = 25; // not PULLUP

const short int PWM_OUT = 26;//  

const short int CURRENT_ADC = 36; // SVP GPIO36 ADC1_CH0
//const short int CURRENT_ADC = 34; // SVP GPIO34 (SENSOR2)


// Channel pins
/*

V 1.0
//const short int RL_1 = 13;
//const short int RL_2 = 4;
//const short int RL_3 = 16;
//const short int RL_4 = 17;
//const short int RL_5 = 21;
//const short int RL_6 = 22;

V 1.2

const short int RL_6 = 13;
const short int RL_1 = 4;
const short int RL_2 = 16;
const short int RL_3 = 17;
const short int RL_4 = 21;
const short int RL_5 = 22;

*/

const short int RL_MAIN = 15;
const int ChannelOutput[] = {4,16,17,21,22,13} ;
const int ActiveChannels = MAX_ZONES; // 6 zones

int lenChannelOutput = sizeof(ChannelOutput)/sizeof(ChannelOutput[0]);

const short int BUILTIN_LED1 = 2; // GPIO02
const short int BUILTIN_LED2 = 3; // GPIO016

//#define LED_OFF()  {digitalWrite(BUILTIN_LED1,LOW);}
//#define LED_ON()   {digitalWrite(BUILTIN_LED1,HIGH);}

#define LED_ON()  
#define LED_OFF() 
 
void callback(char* topic, byte* payload, unsigned int length);

// Initiate the WundergoundClient
//WundergroundClient wunderground(IS_METRIC);
OpenWeatherMapCurrent weather;

// Initiate MQTT client
WiFiClient espClient;

PubSubClient client(mqtt_server,mqtt_server_port,callback,espClient);

// Initiate LCD 
LiquidCrystal_PCF8574 lcd(0x27);



// Global variables
// ============================
int local_hours = 0;
int local_minutes = 0;
int local_seconds = 0;

struct tm local_time;
time_t utc_time = 0;

time_t ts = 0;
time_t uptime = 0;
time_t last_ntp = 0;
//unsigned int current_mA = 0;
unsigned int error_count = 0;
unsigned int ground_humidity = 0;

bool          OTA_mode = false;
unsigned int  OTA_progress = 0;
unsigned long OTA_timeout = 0;
unsigned int  OTA_total = 0;


unsigned long nextNTPRefresh     = 0;
unsigned long nextSecondRefresh  = 0; 
unsigned long nextWeatherRefresh = 0;
unsigned long nextMQTTRefresh    = 0;

#define IDLE_TIMEOUT_SEC 15
#define WAITING_WIFI_TIMEOUT_SEC (5*60)
#define THRESHOLD_ENCODER 1
#define REFRESH_NTP      (60*60*1000)   // 1 hour
#define REFRESH_WEATHER  (15*60*1000)   // 15 min
#define REFRESH_MQTT     (500)         // 500 ms
#define REFRESH_LONG_MQTT (30000)         // 30 s

typedef enum { NULL_SCR = 0,
               MAIN_SCR, 
               IDLE_SCR, 
               OTA_SCR,
               WATERING_SCR, 
               WATERING_CANCEL_SCR, 
               MENU_SCR,
               ONOFF_SCR,
               SETTIME_SCR,
               MANUAL_SCR, 
               CYCLE_SCR,
               INFO_SCR,
               ERROR_SCR} Screen;

typedef enum { INPUT_NULL = 0, ROT_R, ROT_L, ROT_PUSH} Input;

typedef bool (*InputCallback)(Input);
typedef char* (*LabelPrinter)(void);

typedef struct _MenuItem
{
 
  const char* label;
  LabelPrinter printer;
  InputCallback callback;
   
} MenuItem;


struct _status
{
  bool ON;
  int correction;  // Weather forecast correction ( % )
} statusJardino = { true,0 };

struct _config
{
  
  bool    correction_ON;
  uint8_t delayBZones;
  bool    check_current;
  
} configJardino = { false,0,false };


volatile int lastEncoded = 0;
volatile long encoderValue = 0;

// Encoder variables
long lastencoderValue = 0;

int lastMSB = 0;
int lastLSB = 0;
int counterIdle = IDLE_TIMEOUT_SEC;
bool watering = false;

Screen screen = MAIN_SCR;
Screen nextScreen = MAIN_SCR;

TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     //Central European Summer Time
TimeChangeRule CET  = {"CET ", Last, Sun, Oct, 2, 60};      //Central European Standard Time
Timezone CE(CEST, CET);

// ============================
// Prototypes
// ============================

void showMainScreen(Input);
void showWateringScreen(Input);
void showMenuScreen(Input);
void showOnOffScreen(Input);
void showManualScreen(Input);
void showWateringCancelScreen(Input);
void showWateringErrorScreen(Input);
void showInfoScreen(Input);
void showOTAScreen(Input);
void showCycleScreen(Input input);
void ISR_Encoder();
void calculate_correction();
bool setup_wifi();

int max(int a,int b) {return ((a)>(b)?(a):(b)); }
int min(int a,int b) {return ((a)<(b)?(a):(b)); }

// ============================================
void  save_system_config()
{
    EEPROM.begin(512); // EEPROM emulated in flash, 512 bytes
    int addr = EEPROM_CONFIG_AREA;
    EEPROM.write(addr++,EEPROM_SIGNATURE); // Signature
    EEPROM.write(addr++,(statusJardino.ON ? 1 : 0));
    EEPROM.write(addr++,(configJardino.correction_ON? 1 : 0));
    EEPROM.write(addr++,configJardino.delayBZones);
    EEPROM.write(addr++,(configJardino.check_current? 1 : 0));
    EEPROM.end();
}
// ============================================
// ============================================
void  save_system_time()
{
    EEPROM.begin(512); // EEPROM emulated in flash, 512 bytes
    int addr = EEPROM_TIME_AREA;
    EEPROM.write(addr++,EEPROM_SIGNATURE); // Signature
    byte b1 = ((utc_time >> 0) & 0xFF);
    byte b2 = ((utc_time >> 8) & 0xFF);
    byte b3 = ((utc_time >> 16) & 0xFF);
    byte b4 = ((utc_time >> 24) & 0xFF);
    EEPROM.write(addr++,b1);
    EEPROM.write(addr++,b2);
    EEPROM.write(addr++,b3);
    EEPROM.write(addr++,b4);        

    EEPROM.end();
}
// ============================================

void  load_system_config()
{
  EEPROM.begin(512); // EEPROM emulated in flash, 512 bytes
  int addr = EEPROM_CONFIG_AREA;
  byte v;
  v = EEPROM.read(addr++);
  if ( v == EEPROM_SIGNATURE)
  {
    statusJardino.ON             = (EEPROM.read(addr++) > 0);
    configJardino.correction_ON  = (EEPROM.read(addr++) > 0);
    configJardino.delayBZones    = EEPROM.read(addr++);
    configJardino.check_current  = (EEPROM.read(addr++) > 0);
    Serial.printf("Restore configuration from EEPROM : ON %d   Weather correction %d  Check current : %d\n",
          statusJardino.ON,
          configJardino.correction_ON,configJardino.check_current
          );
  }
  
  EEPROM.end();
}

// ============================================

void  load_system_time()
{
  EEPROM.begin(512); // EEPROM emulated in flash, 512 bytes
  int addr = EEPROM_TIME_AREA;
  byte v;
  v = EEPROM.read(addr++);
  if ( v == EEPROM_SIGNATURE)
  {
    byte b1 = EEPROM.read(addr++);
    byte b2 = EEPROM.read(addr++);
    byte b3 = EEPROM.read(addr++);
    byte b4 = EEPROM.read(addr++);
    utc_time = ((b1 << 0) & 0x000000FF) + ((b2 << 8) & 0x0000FF00)+ ((b3 << 16) & 0x00FF0000) + ((b4 << 24) & 0xFF000000);
    Serial.printf("Restore time from EEPROM : %d\n",utc_time);

  }
  
  EEPROM.end();
}
// ============================================
// WDT Emulator

void wdtEnable(uint32_t sec)
{
  esp_task_wdt_init(sec, true); // No panic
  enableLoopWDT();
}


void wdtFeed()
{
  //  ESP.wdtEnable(5000); // 5 seconds
  feedLoopWDT();
}


//int readCurrent(){
//  adc1_config_width(ADC_WIDTH_BIT_10);   //Range 0-1023 
//  adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);  //ADC_ATTEN_DB_11 = 0-3,6V
//  return adc1_get_raw( ADC1_CHANNEL_0 ); //Read analog
//}


void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

void refreshLocalTime()
{
   
  if (WiFi.status() != WL_CONNECTED)
     return;
     
  struct tm timeStruct;
  if (getLocalTime(&timeStruct))
  {
    local_hours   = timeStruct.tm_hour;
    local_minutes = timeStruct.tm_min;
    local_seconds = timeStruct.tm_sec; 
  }
}


// ============================================



void set_output(int n,bool val) // index n 1..6
{
    if (n > lenChannelOutput || n <= 0) return;

    //if (val)
    //  Serial.printf("Zone :%d  Relay: %d\n",n,ChannelOutput[n-1]);
    //digitalWrite(RL_MAIN,val);
    //delay(0.5);
    if (val)
      digitalWrite(ChannelOutput[n-1],HIGH); // 
    else
      digitalWrite(ChannelOutput[n-1],LOW);
}

void reset_outputs()
{
    digitalWrite(RL_MAIN,LOW);
    delay(100);
    for (int i = 0; i < lenChannelOutput; i++)
    {
         digitalWrite(ChannelOutput[i],LOW);
         delay(50);
    }
}

// setting PWM properties
const int PWM_freq = 5000;
const int PWM_channel = 0;
const int PWM_resolution = 10; //Resolution 8, 10, 12, 15

void PWM_set(int dutycycle)
{
  //PWM Value varries from 0 to 1023  
  ledcWrite(PWM_channel, (dutycycle * 1023)/100);
}

// ============================================
void IRAM_ATTR resetModule() {
  ets_printf("Jardino SW reboot\n");
  esp_restart();
}



// ============================================

void setup(void) {

  
 // pinMode(BUILTIN_LED, OUTPUT);
  //(GPIOs support interrupt except D16 )
  pinMode(ENCODER_CK, INPUT_PULLUP); 
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_PUSH, INPUT_PULLUP);

  pinMode(RL_MAIN,OUTPUT);
  digitalWrite(RL_MAIN,LOW);
  pinMode(ChannelOutput[0],OUTPUT);
  pinMode(ChannelOutput[1],OUTPUT);
  pinMode(ChannelOutput[2],OUTPUT);
  pinMode(ChannelOutput[3],OUTPUT);
  pinMode(ChannelOutput[4],OUTPUT);
  pinMode(ChannelOutput[5],OUTPUT);

  // All outputs OFF
  reset_outputs();

  // PWM
  pinMode(PWM_OUT,OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(PWM_channel, PWM_freq, PWM_resolution);
  
  // attach the channel to the GPIO2 to be controlled
  ledcAttachPin(PWM_OUT, PWM_channel);

  PWM_set(50);
  
  Serial.begin(115200);
  Serial.println();
  Serial.print("JARDINO ");
  Serial.println(VERSION);
  
  if (!adcAttachPin(CURRENT_ADC)) {
    Serial.println("Error : ADC pin.");
  }
  analogSetPinAttenuation(CURRENT_ADC,ADC_11db); // Full scale 0-3.9V
        
  if (!Wire.begin(LCD_PIN_SDA,LCD_PIN_SCL)) {
    Serial.println("Error : I2C init.");
  } //
  
  //Wire.begin();
  Wire.beginTransmission(0x27);
  
  int error = Wire.endTransmission();

  if (error != 0) {
    Serial.print("Error: ");
    Serial.print(error);
    Serial.println(": LCD not found.");
  } // if


  lcd.begin(20, 4); // initialize the lcd


  // define lcd custom icons ( 8 max)
  int _icon[8];
  
  // No WiFi icon
  #define NO_WIFI_CHAR 0
  _icon[0] = B00000;
  _icon[1] = B10100;
  _icon[2] = B01000;
  _icon[3] = B10100;
  _icon[4] = B00001;
  _icon[5] = B00101;
  _icon[6] = B00101;
  _icon[7] = B10101;
  lcd.createChar(NO_WIFI_CHAR, _icon);
  
 // WiFi1 icon
 #define WIFI1_CHAR 1
  _icon[0] = B00000;
  _icon[1] = B00000;
  _icon[2] = B00000;
  _icon[3] = B00001;
  _icon[4] = B00001;
  _icon[5] = B00101;
  _icon[6] = B00101;
  _icon[7] = B10101;
  lcd.createChar(WIFI1_CHAR, _icon);

   // WiFi2 icon
  #define WIFI2_CHAR 2
  _icon[0] = B00000;
  _icon[1] = B00001;
  _icon[2] = B00001;
  _icon[3] = B00101;
  _icon[4] = B00101;
  _icon[5] = B10101;
  _icon[6] = B10101;
  _icon[7] = B10101;
  lcd.createChar(WIFI2_CHAR, _icon);

    // Selected icon
  #define SELECT_CHAR 3
  _icon[0] = B10000;
  _icon[1] = B11000;
  _icon[2] = B11100;
  _icon[3] = B11110;
  _icon[4] = B11100;
  _icon[5] = B11000;
  _icon[6] = B10000;
  _icon[7] = B00000;
  lcd.createChar(SELECT_CHAR, _icon);

    // Down icon
   #define DOWN_CHAR 4
  _icon[0] = B00000;
  _icon[1] = B00000;
  _icon[2] = B00000;
  _icon[3] = B00000;
  _icon[4] = B00000;
  _icon[5] = B11111;
  _icon[6] = B01110;
  _icon[7] = B00100;
  lcd.createChar(DOWN_CHAR, _icon);  

    // Up icon
  #define UP_CHAR 5
  _icon[0] = B00100;
  _icon[1] = B01110;
  _icon[2] = B11111;
  _icon[3] = B00000;
  _icon[4] = B00000;
  _icon[5] = B00000;
  _icon[6] = B00000;
  _icon[7] = B00000;
  lcd.createChar(UP_CHAR, _icon);
    
  // Rain icon
  #define RAIN_CHAR 6
  _icon[0] = B00000;
  _icon[1] = B00000;
  _icon[2] = B00110;
  _icon[3] = B01001;
  _icon[4] = B11111;
  _icon[5] = B00000;
  _icon[6] = B10101;
  _icon[7] = B10101;
  lcd.createChar(RAIN_CHAR, _icon);

  // Block icon
  #define BLOCK_CHAR 7
  _icon[0] = B01110;
  _icon[1] = B11111;
  _icon[2] = B11111;
  _icon[3] = B11111;
  _icon[4] = B11111;
  _icon[5] = B11111;
  _icon[6] = B11111;
  _icon[7] = B11111;
  lcd.createChar(BLOCK_CHAR, _icon);

  // Print a message to the LCD.
  //lcd.backlight();
  lcd.setBacklight(100);
  lcd.home(); 
  lcd.clear();
  lcd.setCursor(5,0);
  lcd.print("Jardino32");
  lcd.setCursor(5,1);
  lcd.print(VERSION);
  
  //save_system_config();  //Force system config

  // Encoder setup 
  //rencoder.Begin(10); // Start, with a re-bias of 10

  attachInterrupt(ENCODER_CK, ISR_Encoder, CHANGE);
  attachInterrupt(ENCODER_DT, ISR_Encoder, CHANGE);
 
 
  // ======  RESTORE CONFIGURATION ========== 
#ifdef RESTORE_DEFAULT_CONFIG
  save_system_config();
#else
  load_system_config();
#endif
  
  sched_set_correction (configJardino.correction_ON);
  sched_set_delay_zones(configJardino.delayBZones);
  
  // ================= WIFI ============= 

  lcd.setCursor(1,3);
  lcd.print("WiFi ");
  if (wifi_ssid) lcd.print(wifi_ssid);
  lcd.print(".");


/***
  WiFiManager wifiManager;
  //wifiManager.autoConnect("Jardino-Config");

  //exit after config instead of connecting
  //wifiManager.setBreakAfterConfig(true);
  wifiManager.setConfigPortalTimeout(60);
  
  if (!wifiManager.autoConnect("Jardino-Config")) {
    //Serial.println("failed to connect, we should reset as see if it connects");
    lcd.print("Fault...RESET");
    delay(2000);
    lcd.setBacklight(0);
    delay(5000);
    ESP.reset();
    delay(5000);
  }

#if defined(DEBUGSERIAL)
  DEBUGSERIAL.println(F("WiFi connected"));
  DEBUGSERIAL.print(F("Local IP: "));
  DEBUGSERIAL.println(WiFi.localIP());
#endif
***/
  // ================= WIFI =============
  setup_wifi();



  if (WiFi.status() == WL_CONNECTED)
  {
    lcd.print("OK");

    delay(500);
    //init and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    lcd.setCursor(1,3);
    lcd.print("NTP...     ");
  
    long tries = 5;
    while (utc_time == 0 && tries--)
    {
      //time(&utc_time);
      utc_time = time(nullptr);
      last_ntp = utc_time;
      delay(1000);
    }

    printLocalTime();

    lcd.setCursor(8,3);
    if (utc_time)
    {
      save_system_time(); // Store time in EEPROM
      lcd.print("OK");
    }
    else
    {
      lcd.print("FAIL");
    }
    
    delay (500); 
    lcd.setCursor(1,3);
    lcd.print("MQTT...    ");
    connect_MQTT();

    lcd.setCursor(8,3);
    if (client.connected())
      lcd.print("OK");
    else
      lcd.print("FAIL");
    delay(500);

    if (configJardino.correction_ON)
    {
      lcd.setCursor(1,3);
      lcd.print("Weather...    ");

      nextWeatherRefresh  = millis() + REFRESH_WEATHER; // 15 min.
      calculate_correction();      
      delay(500);
    }

    // Start OTA
    setup_OTA();
  }
  else
  {  
    lcd.print("No WiFi");  
  }

  
  if (utc_time == 0) // If no time, restore from EEPROM
    load_system_time();

  //local_hours   = hour(CE.toLocal(utc_time));
  //local_minutes = minute(utc_time);
  //local_seconds = second(utc_time);
  refreshLocalTime();
  uptime = 0;
  
  sched_setup();
   
  lcd.clear();

  Serial.println("Jardino ready.");
  Serial.printf("Local time %02d:%02d:%02d\n",local_hours,local_minutes,local_seconds);
  delay(1000);
  

  wdtEnable(15); // 15 seconds WDT
}



// ============================================
// ============================================


bool setup_wifi() {

  short blink = 0;
  int tries = 100;
  
  delay(10);
  // We start by connecting to a WiFi network

  WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n_SSID ; j++){
      if (WiFi.SSID(i)== ssid[j] ) {

        wifi_ssid = ssid[j];
        
        long timeout = millis() + WAITING_WIFI_TIMEOUT_SEC*1000;
        WiFi.begin(ssid[j],password[j]);

        while (WiFi.status() != WL_CONNECTED && (millis() < timeout)) //trying to connect 
        {
          delay(500);
          
          Serial.print(".");
          if (blink)
            digitalWrite(BUILTIN_LED1,LOW);
          else
            digitalWrite(BUILTIN_LED1,HIGH);
          blink = !blink;
        }
        break;
      }
    }
  }

  LED_OFF();

  Serial.println("");
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.printf("WiFi %s connected\n",wifi_ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.printf("Signal: %d dBm\n", WiFi.RSSI());
    return true;
  } else {
    Serial.println("WiFi not connected");
    return false;
  }
}

//--------------------------------------------------------------------------------------------
// OTA Initialization
//--------------------------------------------------------------------------------------------

void setup_OTA()
{


  // Port defaults to 3232
  //ArduinoOTA.setPort(3232);


  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("Jardino32_OTA");
  
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    OTA_progress = (progress / (total / 100));
    OTA_total = total;
    char buff[16];
    Serial.printf("Progress: %u%%  %d bytes\n", OTA_progress,total);

    // Refresh screen
    lcd.setCursor(4,2);
    sprintf(buff,"%3d %% (%d)",OTA_progress,OTA_total);
    lcd.print(buff);
    // ====================
    wdtFeed();
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    } else
    {
      Serial.println();
    }
  });
  ArduinoOTA.begin();
}
// ============================================
// ============================================

void connect_MQTT() {
  
  // Loop until we're connect_MQTTed
  int tries = 5;
  char buff[32];
  Serial.println("Connecting to MQTT broker...");
  sprintf(buff,"ESPJard_%d",millis()%1000); // Random client name
  
  while (tries-- && !client.connected()) {
    wdtFeed();

    LED_OFF();

    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(buff, mqtt_user, mqtt_password)) {
      Serial.println();
      Serial.print("Connected to MQTT server as ");
      Serial.println(buff);
      client.subscribe(device_in_topic);   //
      LED_ON();
 
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 1 seconds");
      //LED_ON();
      delay(1000);
    }
  }
}
// ============================================
// ============================================

// ================ MQTT callback =============

char message_buff[64];

void callback(char* topic, byte* payload, unsigned int length) 
{
 
  // create character buffer with ending null terminator (string)
  int i;
  for(i=0; i<length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
 
  String dataReceived = String(message_buff);

  Serial.println("Message arrived topic: " + String(topic) + "(" + dataReceived + ")");
  nextScreen = MAIN_SCR;
  counterIdle = IDLE_TIMEOUT_SEC;

  // =================================================================================
  // Set program
  // =================================================================================

  if (dataReceived.startsWith("P")) // "P nprg,active,HH,MM,days,z1,z2,z3,z4,z5,z6" 
  {
    int np,hh,mm,days;
    bool active;
    int z[MAX_ZONES];

    //int fields[MAX_ZONES+5];
    int nfield = 0;
    char* field = strtok(message_buff+2, ",");
    while (field != 0 && nfield < MAX_ZONES+5)
    {
        if (nfield == 0)
        {
          np = atoi(field);
        }
        else if (nfield == 1)
        {
          active = (atoi(field) == 1);
        }
        else if (nfield == 2)
        {
          hh = atoi(field);
        }
        else if (nfield == 3)
        {
          mm = atoi(field);  
        }
        else if (nfield == 4)
          days = atoi(field);   
        else               
          z[nfield-5] = atoi(field);

        //Serial.printf("%d : %s\n",nfield,field);
        nfield++;
        field = strtok(NULL, ",");
    }

    if ( np <= 0 || np > MAX_PROGRAMS) return;
    if ( hh < 0 || hh > 59) return;
    if ( mm < 0 || mm > 59) return;
    if ( days <= 0 || days > 6) return;  
    for (int i = 0; i < MAX_ZONES; i++)
    {   
      if ( z[i] < 0 || z[i] > MAX_DURATION) return;
    }

    Serial.println("Storing program...");
    sched_set_program(np,active,hh,mm,days,z);
    
  }
  // =================================================================================
  else if (dataReceived.startsWith("CYCLE ")) // "CYCLE program [1..6]" 
  {
    int p = dataReceived.substring(6).toInt();
    sched_cycle(p); 
  }
  // =================================================================================
  else if (dataReceived.equals("CANCEL")) // "CANCEL" 
  {
    sched_cancel();
    publish_remaining_time(0,0,0); // Cancel event
  }
  // =================================================================================
  else if (dataReceived.equals("SET ON")) // "SET ON" 
  {
    statusJardino.ON = true;
    //nextScreen  = MAIN_SCR;
    save_system_config();
  }
  // ================================================================================= 
  else if (dataReceived.equals("SET OFF")) // "SET OFF" 
  {
    statusJardino.ON = false;
    //nextScreen  = MAIN_SCR;
    save_system_config();
    //sched_cancel();
    //publish_remaining_time(0,0); // Cancel event
  }
  // =================================================================================
  else if (dataReceived.startsWith("CONFIG ")) // "CONFIG correction (0:No,1:Yes),delay_bz(sec)" 
  {
    configJardino.correction_ON = (dataReceived.substring(7).toInt() > 0);
    int delayZ = dataReceived.substring(9).toInt();
    if (delayZ >= 0 or delayZ <= 10) ;
      configJardino.delayBZones = delayZ;

    // Apply changes
    save_system_config();

    calculate_correction();
    sched_set_delay_zones(configJardino.delayBZones);
  }
  // =================================================================================
  else if (dataReceived.startsWith("RCONFIG ")) // "RCONFIG cookie" 
  {
      String response;

      char buff[64];
      sprintf(buff,"R4,%d,%d,%d,%d,%d",
                  (statusJardino.ON?1:0),
                  configJardino.delayBZones,
                  (configJardino.correction_ON?1:0),
                  statusJardino.correction,
                  uptime);
      response += buff;
                     
      response += "," + dataReceived.substring(8);
      Serial.println(response.c_str());
      client.publish(device_out_topic, response.c_str());
  }
  // =================================================================================
  else if (dataReceived.startsWith("MAN ")) // "MAN Zone Time(sec)" 
  {
    if (statusJardino.ON)
    {
      int z = dataReceived.substring(4).toInt();
      int t = dataReceived.substring(6).toInt();
      sched_manual(z,t); 
      //nextScreen  = MAIN_SCR;;
    }
  }
  // =================================================================================  
  else if (dataReceived.startsWith("M ")) // "M z1,z2,z3,z4,z5,z6" 
  {

    int z[MAX_ZONES];

    int fields[MAX_ZONES];
    int nfield = 0;
    char* field = strtok(message_buff+2, ",");
    while (field != 0 && nfield < MAX_ZONES)
    {              
      z[nfield] = atoi(field);
      nfield++;
      field = strtok(NULL, ",");
    }
 
    for (int i = 0; i < MAX_ZONES; i++)
    {   
      if ( z[i] < 0 || z[i] > MAX_DURATION) return;
    }

    sched_manuals(z);
    
  }
  // ================================================================================= 
  else if (dataReceived.startsWith("READPRG ")) // "READPRG nprg cookie" 
  {
    String response;

    int np = dataReceived.substring(8).toInt();
    if ( np <= 0 || np > MAX_PROGRAMS) return;
  
    const ProgramInfo *pp = sched_get_program(np);
    if ( pp != NULL)
    {
      char buff[64];
      sprintf(buff,"R1,P%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                    pp->id,
                    (pp->active ? 1 : 0),
                    pp->start_time,
                    pp->repeat_days,
                    pp->zones[0].duration,
                    pp->zones[1].duration,
                    pp->zones[2].duration,
                    pp->zones[3].duration,
                    pp->zones[4].duration,
                    pp->zones[5].duration);
        response += buff;
    }
    else
    {
      response = "NOPRG";
    }
    
    response += "," + dataReceived.substring(10);
    Serial.println(response.c_str());
    client.publish(device_out_topic, response.c_str());
  }
  // =================================================================================  
  else if (dataReceived.startsWith("READALL ")) // "READALL cookie" 
  {
    String response;
    response  = "R0," ;
    response += (statusJardino.ON ? "ON" : "OFF");
    const ProgramInfo *pp = sched_get_program_ordered(0);
    if ( pp != NULL)
    {
      char buff[64];
      sprintf(buff,",P%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                    pp->id,
                    (pp->active ? 0 : 1),
                    pp->start_time,
                    pp->repeat_days,
                    pp->zones[0].duration,
                    pp->zones[1].duration,
                    pp->zones[2].duration,
                    pp->zones[3].duration,
                    pp->zones[4].duration,
                    pp->zones[5].duration);
        response += buff;
    }

    response += "," + dataReceived.substring(8);
    Serial.println(response.c_str());
    client.publish(device_out_topic, response.c_str());

    int ztime[MAX_ZONES];
    sched_get_durations(ztime);
    for (int nz = 0; nz < MAX_ZONES; nz++)
    {
      String res;
      char buff[64];
      sprintf(buff,"R2,%d,%s,%d",(nz+1),sched_get_zone_name(nz+1),ztime[nz]);
      res = buff;
    
      res += "," + dataReceived.substring(8);
      Serial.println(res.c_str());
      client.publish(device_out_topic, res.c_str()); 
    }
  } 
  // =================================================================================  
  else if (dataReceived.startsWith("SETZ ")) // "SETZ N name " 
  {
      String response;

      int nz = dataReceived.substring(6).toInt();
      if ( nz <= 0 || nz > MAX_ZONES) return;
      sched_set_zone_name(dataReceived.substring(8).c_str(),nz);
      
      //response += sched_get_zone_name(nz);
    
      //response += "," + dataReceived.substring(8);
      //Serial.println(response.c_str());
      //client.publish(device_out_topic, response.c_str());
  } 
  // =================================================================================  
  else
  {
    Serial.println("Incorrect MQTT command");
  }
}

// =====================================================
// Publish watering remaining time
// =====================================================

static bool cancel_event_published = true;

void publish_remaining_time(int zone, int rmin,int rsec)
{
  
  String event;

  // Cancel event when zone = 0
  if (zone == 0) 
  {
      if (cancel_event_published) 
      {
        return;
      }
      else // Publish End of Time Event
      {
        cancel_event_published = true;
        event = "EE";
        Serial.println(event.c_str());
        client.publish(device_out_topic, event.c_str());
        return;
      }
  }
  else
  {
     cancel_event_published = false;
  }

  // Publishes each 5 seconds
  if (rsec % 5 == 0)
  {
    char buff[64];
    sprintf(buff,"ET,%d,%d,%d",zone,rmin,rsec);
    event = buff;
    Serial.println(event.c_str());
 
    client.publish(device_out_topic, event.c_str());
  }
}




// =====================================================
// =====================================================

void refreshScreen(Input input)
{
  switch (screen)
  {
        case MAIN_SCR            : lcd.setBacklight(255); showMainScreen(input); break;
        case IDLE_SCR            : lcd.setBacklight(0);showMainScreen(input); break;
        case WATERING_SCR        : showWateringScreen(input); break;
        case WATERING_CANCEL_SCR : showWateringCancelScreen(input); break;
        case MENU_SCR            : showMenuScreen(input); break; 
        case ONOFF_SCR           : showOnOffScreen(input); break;  
        case MANUAL_SCR          : showManualScreen(input);break; 
        case CYCLE_SCR           : showCycleScreen(input);break;
        case INFO_SCR            : showInfoScreen(input);break; 
        case OTA_SCR             : showOTAScreen(input);break; 
        case ERROR_SCR           : lcd.setBacklight(255);showErrorScreen(input);break;
        case NULL_SCR            : break;        
  }
}

// =====================================================

unsigned long nextTime0 = millis() + 5000;
int ni= 0;
Input input_simulator()
{
  unsigned long time0 = millis();

  if ((long)(time0 - nextTime0) >= 0)
  {
      nextTime0  = time0 + 2000;
      if ((ni++ % 3) == 0)
        return ROT_PUSH;
      else  
        return ROT_L;
  }
  else
    return INPUT_NULL;
}

// =====================================================
// LOOP
// =====================================================

int lastEncoderValue = 0;
int lastEncoderValue1 = 0;
int repeat = 0;
int rot_button = 1;
int lastButtonState = 1;
int debounceDelay = 50;
int lastDebounceTime = 0;
//bool refreshScreen = true;
bool lastJardinoStatus = statusJardino.ON;
int  prev_out = 0;



// ===============================================
// loop
// ===============================================

void loop() {

  unsigned long timeNow;
  Input input = INPUT_NULL;



  // ===============================================
  // Watchdog timer
  // ===============================================
  wdtFeed();


  // ===============================================
  // Wifi task management
  // ===============================================
  if (WiFi.status() == WL_CONNECTED && !OTA_mode)
  {
      // MQTT loop
      timeNow = millis();
      if ((long)(timeNow - nextMQTTRefresh) >= 0) {
        if (!client.connected()) {
          // Reconnect with MQTT broker
          connect_MQTT();
          if (!client.connected()) nextMQTTRefresh  = timeNow + REFRESH_LONG_MQTT; //  ms
        }    
        else {     
          nextMQTTRefresh  = timeNow + REFRESH_MQTT; //  ms
          client.loop();
        }
      }



      // NTP Time refresh
      if ((long)(timeNow - nextNTPRefresh) >= 0)
      {
        nextNTPRefresh  = timeNow + REFRESH_NTP; // 1 hour

        time(&last_ntp);
        
        if (last_ntp != 0)
        {
          utc_time = last_ntp;
          refreshLocalTime();
          save_system_time(); // Store time in EEPROM
        }
      }

      // Weather conditions refresh
      if ((long)(timeNow - nextWeatherRefresh) >= 0)
      {
        nextWeatherRefresh  = timeNow + REFRESH_WEATHER; // 1 hour
        if (!watering) calculate_correction();     // No correction is applied if watering
      }
  }
 
  // ===============================================
  // ON/OFF page management
  // ===============================================
  // Change of mode ON<->OFF
  
  if (lastJardinoStatus != statusJardino.ON)
  {
      lastJardinoStatus = statusJardino.ON;

      if (!statusJardino.ON)
      { 
        sched_cancel();
        publish_remaining_time(0,0,0); // Publish Cancel event
      }
      // ====================
      // Refresh screen 
      // ====================
      lcd.clear();
      refreshScreen(INPUT_NULL);
  }
  
  // ===============================================
  // Watering page management
  // ===============================================
   
  // Change of page or watering running notification
  if (statusJardino.ON && 
      sched_get_current() != NULL) // !NULL => watering time
  {
    lcd.setBacklight(255);
    counterIdle = IDLE_TIMEOUT_SEC;

    // If not watering pages goto them...
    if (screen != WATERING_SCR && 
        screen != WATERING_CANCEL_SCR)
    {
      nextScreen = MAIN_SCR; // Next screen when finish
      screen = WATERING_SCR;
      lcd.clear();

      // ====================
      // Refresh screen to show watering
      // ====================
      refreshScreen(INPUT_NULL);
 
    }
    // Change to cancel watering allowed in this state
    else if  (nextScreen == WATERING_CANCEL_SCR)
    {
      screen = WATERING_CANCEL_SCR;
      nextScreen = NULL_SCR;  // To avoid continous refresh
      
      // ====================
      // Refresh screen to show watering cancel area
      // ====================
      refreshScreen(INPUT_NULL);
    }   
    // Return from cancel watering allowed in this state
    //else if  (screen == WATERING_CANCEL_SCR && nextScreen == MAIN_SCR)
    //{
    //  screen = WATERING_SCR;
    //} 
 
  }

  // Refresh screen if changed
  // else if (nextScreen != NULL_SCR && (nextScreen != screen)) // Refesh if page changed
  else if (nextScreen != screen)
  {

    screen     = nextScreen;
    //nextScreen = NULL_SCR;
    lcd.clear();

    // ====================
    // Refresh screen
    // ====================
    refreshScreen(INPUT_NULL);
 
  }

  // ===============================================
  // Rotary input management
  // ===============================================
  bool push = false;
  timeNow = millis();

  // Process encoder read value
  int reading= digitalRead(ENCODER_PUSH);

  if (reading == 0)
  {
    counterIdle = IDLE_TIMEOUT_SEC;
  }
    
  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = timeNow;
  }

  // Long press -> RESET
  if (reading == 0 && ((timeNow - lastDebounceTime) > 3000)) 
  {
    resetModule();
  }
  
  if ((timeNow - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != rot_button) {
      rot_button = reading;
      
      if (rot_button == 0)
      {
        push = true;
        input = ROT_PUSH;
      }
    }
  }
  lastButtonState = reading;


  /*************/
  if (!push && encoderValue != 0 )
  {
    //Serial.printf("Encode: %d\n",encoderValue);
    if (lastEncoderValue == encoderValue && lastEncoderValue1 == encoderValue)
    {
      counterIdle = IDLE_TIMEOUT_SEC;
      if (encoderValue > 0) 
        input = ROT_L;
      else if (encoderValue < 0)
        input = ROT_R;
    }

    lastEncoderValue1 = lastEncoderValue;
    lastEncoderValue  = encoderValue;
    encoderValue = 0;
  }
  /*************/
/***
  if (!push && encoderValue != 0 )
  {
    if (abs(lastEncoderValue-encoderValue) > THRESHOLD_ENCODER)
    {
      counterIdle = IDLE_TIMEOUT_SEC;
      if (encoderValue > 0) 
        input = ROT_L;
      else if (encoderValue < 0)
        input = ROT_R;
    }
    lastEncoderValue  = encoderValue;
    encoderValue = 0;
  }
****/
  ////////////////SIMULATOR 
  ///input = input_simulator();
  
  if ( input != INPUT_NULL)
  {
      counterIdle = IDLE_TIMEOUT_SEC;
      if (screen == IDLE_SCR)
      {
        nextScreen = screen = MAIN_SCR;
        refreshScreen(INPUT_NULL);
      }
      else
      {
        refreshScreen(input);
      }
  }


  // ===============================================
  // OTA Mode
  // ===============================================
  if (OTA_mode)
  {
    // Handle OTA
    ArduinoOTA.handle();
    
    // ====================
    // Refresh screen
    // ====================
    refreshScreen(input);

    return;
  }
  
  // ===============================================
  // Periodic tasks (each second) 
  // ===============================================
  timeNow = millis();

  if ((long)(timeNow - nextSecondRefresh) >= 0)
  {
      nextSecondRefresh  = timeNow + 1000;
  
    
      // ====================
      // Update Software RTC
      // ====================
      uptime++;
      swRTC_Update(1);

      // ====================
      // Schedule and ouput control
      // ====================     
      if (statusJardino.ON) 
      { 
          watering = sched_loop();
          int outn = get_relay_active() ; // returns 1..N
          if (outn > 0)
          {
            digitalWrite(RL_MAIN,HIGH);
            delay(100);
            if (outn != prev_out)
            {
              if (prev_out > 0)
                set_output(prev_out,false); // switch OFF previous output
              prev_out = outn;
            }
            set_output(outn,true); // switch ON
          }
          else
          {
            prev_out = 0;
            reset_outputs(); // All outputs OFF
          }
      }
      else
      {
          set_all_relays_off();
          prev_out = 0;
          reset_outputs(); // All outputs OFF
      }

      
      // ====================     
      // Screen idle timeout
      // ====================
      if (counterIdle-- == 0)
      {
        //lcd.setBacklight(0);

        counterIdle = IDLE_TIMEOUT_SEC;
      
        if (screen != IDLE_SCR or 
            screen != WATERING_SCR)
        {
           nextScreen = IDLE_SCR;                
        }
      }


      // ====================
      // Read ADC ( 12 bits, Vref = 1.1 V. 
      // Att = 11 dB-> ADC ESP32 range: 0.1 V ... 3.9 V 
      //    The input voltage of ADC will be reduced to about 1/3.6 
      // Vsense = I * 3.3 ohmios
      // ADC = (Vsense * 4096) / 1.1 
      //   = > Current_mA  = 1100 * (ADC / (4096 * 3.3))
      // ====================
/*** CURRENT MONITORING DISABLED
      current_mA = (analogRead(CURRENT_ADC) * 1100 )/ 13516; // Revisar
     
      if (configJardino.check_current)
      {
        if (current_mA < 100)
        {
          error_count = 0;
        }
        else if (!statusJardino.ON or (get_relay_active() == 0)) 
        {
          if (++error_count > 2 && screen != ERROR_SCR)
          {
            nextScreen = ERROR_SCR;
            Serial.print("ERROR !!! : overcurrent "); 
            Serial.println(current_mA); 
            error_count = 0;
            //lcd.clear();
          }
        }
      }
*/
      // ====================
      // Refresh screen
      // ====================
      refreshScreen(INPUT_NULL);

 
      
  }

  // ===============================================
  
  // ====================
  // Watchdog timer
  // ====================
  wdtFeed();
  
  yield();

}




// ===============================================
// ===============================================
void swRTC_Update(int step_sec)
{
  local_seconds   += step_sec;
  utc_time += step_sec;
  if (local_seconds > 59)
  {
      local_seconds = 0;
      local_minutes++;
      if (local_minutes > 59)
      {
        local_minutes = 0;
        local_hours++;
        if (local_hours > 23)
        {
          local_hours = 0;
        }
      }              
  }
}

// ===============================================

#define TEMP_HOT      30
#define TEMP_COOL     20
#define HARD_RAIN_MM  6
#define LIGHT_RAIN_MM 2

void calculate_correction()
{
  statusJardino.correction = 0; // Default correction

  if (configJardino.correction_ON and  (WiFi.status() == WL_CONNECTED)) 
  {

    
#if defined(WU)
    wunderground.updateConditionsLatLon(WUNDERGROUND_API_KEY, WUNDERGROUND_LANGUAGE,WUNDERGROUND_LAT, WUNDERGROUND_LONG);
    wunderground.updateForecastLanLon(WUNDERGROUND_API_KEY, WUNDERGROUND_LANGUAGE,WUNDERGROUND_LAT, WUNDERGROUND_LONG);
 
    if (wunderground.getCurrentTemp().length() > 0)
    {
      int current_temp = wunderground.getCurrentTemp().toInt();
      int PoP_1 = wunderground.getPoP(1).toInt();
      int precipitationToday = wunderground.getPrecipitationToday().substring(0,wunderground.getPrecipitationToday().indexOf('m')).toInt();
      int forecastHighTemp_1 = wunderground.getForecastHighTemp(2).toInt();
      
      Serial.printf("WU Forecast: T %d -> %d , Rain today %d mm Prob. %d %%\n",
                  current_temp,forecastHighTemp_1,precipitationToday,PoP_1);

      if      ( PoP_1 >= 80 and precipitationToday >= ___HARD_RAIN_MM) // > 80% High Rain
        statusJardino.correction = -75;
      else if ( PoP_1 >= 60 and precipitationToday >= LIGHT_RAIN_MM) // > 60% Ligth rain
        statusJardino.correction = -50;
      else if ( PoP_1 >= 60 and current_temp < TEMP_COOL) // >60% of probability of rain  and cool
        statusJardino.correction = -25;
      else if (current_temp >= TEMP_HIGH or forecastHighTemp_1 >= TEMP_TOO_DRY) // Dry, high temperature
        statusJardino.correction  = +25;
      else
        statusJardino.correction  = 0;
    }
#endif    


    weather.setMetric(true);
    weather.setLanguage("es");
    OpenWeatherMapCurrentData data;
    data.rain_1h = 0.0;
    data.rain_3h = 0.0;
    
    weather.updateCurrent(&data, OWM_API_KEY, OWM_LAT, OWM_LON);
    
    bool raining = data.main.startsWith("Rain");
    
    {

      if (!raining)
          Serial.printf("OWM Forecast: Temp %d  Rain %f,%f\n",(int)data.temp,data.rain_1h,data.rain_3h);
      else
          Serial.printf("OWM Forecast: Temp %d  Raining\n",(int)data.temp);
          
      float rain = data.rain_1h;
      if (rain == 0.0)  
        rain = data.rain_3h;
        
      if (rain >= HARD_RAIN_MM || raining) // 8mm
      {
        //statusJardino.correction = -100;
        statusJardino.correction = -50;
        ground_humidity += 5;
      }
      else if (rain >= LIGHT_RAIN_MM) // 1mm
      {
        statusJardino.correction = -25;
        ground_humidity += 1;
      }
      else if (data.temp > TEMP_HOT or data.tempMax > TEMP_HOT + 2)
      {
        statusJardino.correction  = +25;
        ground_humidity = max(ground_humidity-10,0);
      }
      else
      {
        statusJardino.correction  = 0;
        if (ground_humidity > 0) ground_humidity -= 2;
      }
    }


    Serial.printf("Calculated correction value : %d %%\n",statusJardino.correction);
    if (ground_humidity > 20) statusJardino.correction = -75;
    Serial.printf("Ground humidity       value : %d\n",ground_humidity);  
  }
  else
  { 
    Serial.printf("NO correction value : %d %%\n",statusJardino.correction);     
  }
  
  sched_set_correction(statusJardino.correction);
  
}
// ==========================================
// ==========================================
// GUI LCD
// ==========================================
// ==========================================


// ==========================================
// SCREEN : MAIN
// ==========================================
const char* dow[] = {"xxx","Dom","Lun","Mar","Mie","Jue","Vie","Sab"};
const char* moy[] = {"xxx","Ene","Feb","Mar","Abr","May","Jun","Jul","Ago","Sep","Oct","Nov","Dic"};

// ==========================================


void showMainScreen(Input input)
{  
  char buffer_str[30];
  static int  prev_min = -1;

  if (input == ROT_PUSH)
  {
    nextScreen = MENU_SCR;
    return;
  }
   
  // LINE 1
  // =============================
  long wifi_rssi = -100;
  byte pix = NO_WIFI_CHAR; // No Wifi

  
  time_t localt = utc_time; //default value
     
  if (WiFi.status() == WL_CONNECTED)
  {
      wifi_rssi = WiFi.RSSI();
    
      if(wifi_rssi < -85)
        pix = WIFI1_CHAR;
      else
        pix = WIFI2_CHAR;

      localt = CE.toLocal(utc_time);
  }  
  
  lcd.setCursor(0,0);
  lcd.write(pix);
    
  sprintf(buffer_str,"%02d:%02d:%02d",local_hours,local_minutes,local_seconds);    
  lcd.setCursor(3,0);
  lcd.print(buffer_str);
  sprintf(buffer_str,"%2d %s",day(localt),moy[month(localt)]);
  lcd.setCursor(13,0);
  lcd.print(buffer_str);

 
  // LINE 2
  // =============================
  
  if (!statusJardino.ON)
  {  

    lcd.setCursor(8,2);
    lcd.print("OFF");

  }
  else
  {
    
    time_t t ;
    int idp ;

    // Next schedule
    const ProgramInfo* prg = sched_get_program_ordered(0);
    if (prg->active)
    {
      t = CE.toLocal(prg->start_time);
      idp = prg->id;
      
      sprintf(buffer_str,"P%d %02d:%02d %s %2d",
            idp,
            prg->hour_start,prg->minute_start,
            dow[weekday(t)],day(t));
    }
    else
    {
      sprintf(buffer_str,"* No program *");
    }
    lcd.setCursor(0,1);
    lcd.print(buffer_str);

    prg = sched_get_program_ordered(1);
    if (prg->active)
    {
        t = CE.toLocal(prg->start_time);
        idp = prg->id;
        sprintf(buffer_str,"P%d %02d:%02d %s %2d",
          idp,
          prg->hour_start,prg->minute_start,
          dow[weekday(t)],day(t));
    }
    else
    {
        sprintf(buffer_str,"  ");
        lcd.setCursor(8,2);
        lcd.print("   ");
    }
    lcd.setCursor(0,2);
    lcd.print(buffer_str);    

    // LINE 3
    // =============================
    lcd.setCursor(0,3);
    if (configJardino.correction_ON)
    {
      if (statusJardino.correction >= 0)
          sprintf(buffer_str,"+%02d%%",statusJardino.correction);
      else
          sprintf(buffer_str,"%03d%%",statusJardino.correction);
      lcd.print(buffer_str);
    }
    else
    {
      lcd.print("     ");
    }


    // Zones info line
    int z[6];
    char b[4];
    sched_get_durations(z);
    strcpy(buffer_str,"");

    lcd.setCursor(5,3);
    for (int i=0; i < ActiveChannels; i++)
    {
      /*
      if (z[i] > 0)
        sprintf(b,"%02d ",z[i]/60); // Minutes
      else
        sprintf(b,"__ ");
      strcat(buffer_str,b);
      */
      if (z[i] > 0)
      {
        lcd.write(BLOCK_CHAR);
        lcd.print(" ");
      }
      else
        lcd.print("_ ");
    }
             
    // lcd.setCursor(1,3);
    //lcd.print(buffer_str);
    
  }

   publish_remaining_time(0,0,0); // Cancel the publishing of remaining time event
}


// ==========================================
// SCREEN: WATERING
// ==========================================


const ZoneInfo* last_zone = NULL;

void showWateringScreen(Input input)
{  
  char buffer_str[40];
  static bool blink_zone = false;

  const ZoneInfo* zone = sched_get_current();

  if (input == ROT_PUSH)
  {
    nextScreen = WATERING_CANCEL_SCR;
    return;
  }
 
  if (zone == NULL)
  {
    last_zone = NULL;
    return; 
  }

   
  // Show ZONE with Remaining time
  // =============================
  sprintf(buffer_str,"ZONA %d   %02d:%02d",zone->relay,zone->countdown/60,zone->countdown%60);
  lcd.setCursor(3,1);
  lcd.print(buffer_str);

  // TEST !!!!!!!!!!!!!!!!!!!!!!
  //sprintf(buffer_str,"mA: %5d",current_mA);
  //lcd.setCursor(1,0);
  //lcd.print(buffer_str);

  //zoneNameLabel->setLabel(sched_get_zone_name(zone->relay));

  // Publish watering remaining time
  publish_remaining_time(zone->relay, zone->countdown/60,zone->countdown%60);

  // Zones info line
  int z[6];
  char b[4];
  sched_get_durations(z);
  strcpy(buffer_str,"");

  lcd.setCursor(5,3);
  for (int i=0; i < ActiveChannels; i++)
  {
      if (z[i] > 0)
      {
        if (zone->relay == i+1) {
          blink_zone = !blink_zone;
          if (blink_zone)
            lcd.write(BLOCK_CHAR);
          else
            lcd.write(' ');
        }
        else
        {
          lcd.write(BLOCK_CHAR); // Block Custom char
        }
        lcd.write(' ');
      }
      else
        lcd.print("_ ");
  }

  
   
}

// ==========================================
// MENU 
// ========================================== 
#define NO_EDIT 0xFFFF
uint32_t menu_pos = 0;
uint32_t edit_pos = NO_EDIT;


MenuItem* prev_menu = NULL;
int initial_menu = 0;

void menuScreen(Input _input, MenuItem* menu,int menus)
{
  if (prev_menu != menu)
  {
    prev_menu  = menu;
    menu_pos = 0;    
    edit_pos = NO_EDIT;
  }
  
  if (_input == ROT_PUSH)
  {
    // Select Menu
    if (menu[menu_pos].printer == NULL)
    {
      if (menu[menu_pos].callback(_input)) return;
    }
    // Editable menu
    else
    {
      if (edit_pos == NO_EDIT)
        edit_pos = menu_pos;
      else
        edit_pos = NO_EDIT;
    }
  }
  else if (_input == ROT_R)
  {
    if (edit_pos == NO_EDIT)
    {
      if (menu_pos < menus-1)
        menu_pos++;
      //menu_pos = (menu_pos + 1) % menus;
    }
    else
    {
      menu[menu_pos].callback(_input);
    }
  }
  else  if (_input == ROT_L) 
  {
    if (edit_pos == NO_EDIT)
    {
      if (menu_pos > 0)
        menu_pos--;
      //else
      //  menu_pos = menus - 1;
    }
    else
    {
      menu[menu_pos].callback(_input);
    }
  }

  int start_menu = max(0,menu_pos - 3);
  int end_menu   = start_menu + min(menus,4);
  for (int i = start_menu; i < end_menu; i++)
  {
    lcd.setCursor(0,i-start_menu);
    if (i == edit_pos)
      lcd.print("*");
    else if (i == menu_pos)
      lcd.write(SELECT_CHAR); // select custom char
    else
      lcd.print(" ");

    lcd.print(" ");    
    
    if (menu[i].printer != NULL)
      lcd.print(menu[i].printer());
    else
      lcd.print(menu[i].label);

    // Print up/down icon
    if (i == start_menu)
    {
      lcd.setCursor(19,0);
      if (start_menu !=0) 
          lcd.write(UP_CHAR); // UP custom char
      else
          lcd.print(" ");
    }

    if (i == end_menu-1)
    {
      lcd.setCursor(19,3);
      if (end_menu < menus) 
          lcd.write(DOWN_CHAR); // select DOWN custom char
      else
          lcd.print(" ");
    }
    
  }

}

// ==========================================
// SCREEN: MENU
// ==========================================

bool cb_menu_return(Input)
{
  nextScreen = MAIN_SCR;
  return true;
}

// ==========================================
bool cb_menu_go_onoff(Input)
{
  nextScreen = ONOFF_SCR;
  return true;
}

// ==========================================
bool cb_menu_go_manual(Input)
{
  nextScreen = MANUAL_SCR;
  return true;
}

// ==========================================
bool cb_menu_go_cycle(Input)
{
  nextScreen = CYCLE_SCR;
  return true;
}
// ==========================================
bool cb_menu_go_info(Input)
{
  nextScreen = INFO_SCR;
  return true;
}

// ==========================================
bool cb_menu_go_OTA(Input)
{
  nextScreen = OTA_SCR;
  OTA_timeout = millis() + 120000;
  
  OTA_mode = true;

  return true;
}

MenuItem menuList[] = {{"VOLVER",NULL,cb_menu_return},
                       {"ON/OFF",NULL,cb_menu_go_onoff},
                       {"MANUAL",NULL,cb_menu_go_manual},
                       {"CICLO ",NULL,cb_menu_go_cycle},
                       {"INFO  ",NULL,cb_menu_go_info},
                       {"OTA   ",NULL,cb_menu_go_OTA}};
                       

void showMenuScreen(Input input)
{
  menuScreen(input,menuList,sizeof(menuList)/sizeof(menuList[0]));
}


// ==========================================
// SCREEN: MENU ON-OFF
// ==========================================

bool on_off;

bool cb_man_return(Input)
{
  nextScreen = MAIN_SCR;
  return true;
}


// ==========================================
bool cb_man_on(Input)
{

  statusJardino.ON = true;
  save_system_config();

  nextScreen = MAIN_SCR;
  return true;
}

// ==========================================
bool cb_man_off(Input)
{

  statusJardino.ON = false;
  save_system_config();

  nextScreen = MAIN_SCR;
  return true;
}


MenuItem onoffList[] = {{"VOLVER",NULL,cb_man_return},
                       {"ON",NULL,cb_man_on},
                       {"OFF",NULL,cb_man_off}};

                       

void showOnOffScreen(Input input)
{
  menuScreen(input,onoffList,3);
}


// ==========================================
// SCREEN: MENU MANUAL
// ==========================================



int zone = 1;
int tpo = 30;
char buffPrinter[20];

// Printers

char* pr_zone()
{
  sprintf(buffPrinter,"Zona:   %d",zone);
  return buffPrinter;
}


char* pr_time()
{
  sprintf(buffPrinter,"Tiempo: %02d:%02d",tpo/60,tpo%60);
  return buffPrinter;
}

// Callbacks

bool cb_upd_zone(Input inp)
{
  if (inp == ROT_R)
  {
    zone++;
    if (zone > MAX_ZONES) zone = 1;
  }
  else if (inp == ROT_L)
  {
    if (zone == 1) 
      zone = MAX_ZONES;
    else
      zone--;  
  }
  return false;
}
bool cb_upd_time(Input inp)
{
  if (inp == ROT_R)
  {
    tpo += 30;
    if (tpo > 300) tpo = 30;
  }
  else if (inp == ROT_L)
  {
    if (tpo <= 30) 
      tpo = 300;
    else
      tpo -= 30;
  }
  return false;
}

bool cb_manual_ok(Input)
{
  if (statusJardino.ON) sched_manual(zone,tpo);
  nextScreen = MAIN_SCR;
  return true;
}

bool cb_manual_cancel(Input)
{
  nextScreen = MAIN_SCR;
  return true;
}

MenuItem manualList[] = {
                       {NULL,pr_zone,cb_upd_zone},
                       {NULL,pr_time,cb_upd_time},
                       {"REGAR",NULL,cb_manual_ok},
                       {"VOLVER",NULL,cb_manual_cancel}};

void showManualScreen(Input input)
{
  menuScreen(input,manualList,4);
}

// ==========================================
// SCREEN: MENU CYCLE
// ==========================================

int prg = 1;

// Printers

char* pr_prg()
{
  sprintf(buffPrinter,"Programa:   %d",prg);
  return buffPrinter;
}


// Callbacks

bool cb_upd_prg(Input inp)
{
  if (inp == ROT_R)
  {
    prg++;
    if (prg > MAX_PROGRAMS) prg = 1;
  }
  else if (inp == ROT_L)
  {
    if (prg == 1) 
      prg = MAX_PROGRAMS;
    else
      prg--;  
  }
  return false;
}

bool cb_cycle_ok(Input)
{
  if (statusJardino.ON) sched_cycle(prg);
  nextScreen = MAIN_SCR;
  return true;
}

bool cb_cycle_cancel(Input)
{
  nextScreen = MAIN_SCR;
  return true;
}

MenuItem cycleList[] = {
                       {NULL,pr_prg,cb_upd_prg},
                       {"REGAR",NULL,cb_cycle_ok},
                       {"VOLVER",NULL,cb_cycle_cancel}};

void showCycleScreen(Input input)
{
  menuScreen(input,cycleList,sizeof(cycleList)/sizeof(cycleList[0]));
}


// ==========================================
// AREA: MENU CANCEL WATERING
// ==========================================


int cursor_cancel_menu = 0;
void showWateringCancelScreen(Input input)
{
  if (input == ROT_PUSH)
  {
    if (cursor_cancel_menu == 1) // Cancel selected
    {
      sched_cancel();
      nextScreen = MAIN_SCR;
    }
    else
    {
      lcd.setCursor(2,2);
      lcd.print("               ");
      screen = WATERING_SCR;
    }
   
    return;
  }
  else if (input == ROT_L)
  {
    cursor_cancel_menu = 0;
  }
  else  if (input == ROT_R) 
  {
    cursor_cancel_menu = 1;
  }

  showWateringScreen(INPUT_NULL);
  
  lcd.setCursor(2,2);
  if (cursor_cancel_menu == 0)
    lcd.print("CANCELAR  <NO> ");
  else
    lcd.print("CANCELAR  <SI> ");   

}

// ==========================================
// ERROR SCREEN
// ==========================================

void showErrorScreen(Input input)
{
  reset_outputs();
  sched_cancel();

  lcd.setCursor(0,0);
  lcd.print("ERROR OUTPUT !!!! ");
  //lcd.setCursor(0,1);
  //lcd.printf("I(mA): %04d  ",current_mA);
  lcd.setCursor(0,2);
  lcd.print("PRESS TO RESET");

  if (input == ROT_PUSH)
  {
      nextScreen = MAIN_SCR;
  }

}

// ==========================================
// OTA SCREEN ( Refresh in OTAloop callback)
// ==========================================

void showOTAScreen(Input input)
{
  char buff[16];
  
  reset_outputs();
  sched_cancel();
 

  lcd.setCursor(4,0);
  lcd.print("OTA UPDATE ");
  lcd.setCursor(2,1);
  if (WiFi.status() == WL_CONNECTED)
    lcd.print(WiFi.localIP());
  else
    lcd.print(" Not connected.");

  lcd.setCursor(4,2);
  sprintf(buff,"%3d %% (%d)",OTA_progress,OTA_total);
  lcd.print(buff);

  lcd.setCursor(2,3);
  lcd.print("Press to exit");

  if (input == ROT_PUSH || millis() > OTA_timeout)
  {
    OTA_mode = false;
    nextScreen = MAIN_SCR;
  }

}

// ==========================================
// SCREEN INFO
// ==========================================

int first_info_line = 0;
void showInfoScreen(Input input)
{
  char buffer_str[64];
   
  if (input == ROT_PUSH)
  {
    nextScreen = MAIN_SCR;
    lcd.clear();
    return;
  }
  else if (input == ROT_L)
  {
    first_info_line = 0;
    lcd.clear();
  }
  else  if (input == ROT_R) 
  {
    first_info_line = 2;
    lcd.clear();
  }

  int display_info_line = 0;
  int info_line = first_info_line;
  
  while (display_info_line < 4)
  {
    lcd.setCursor(0,display_info_line++);
    
    switch(info_line)
    {
      case 0:
        lcd.print("IP  :");
        if (WiFi.status() == WL_CONNECTED)
          lcd.print(WiFi.localIP());
        else
          lcd.print("----");
        break;

      case 1:
        if (WiFi.status() == WL_CONNECTED){
          sprintf(buffer_str,"SSID:%s",wifi_ssid);
          lcd.print(buffer_str); 
        }
        else
          lcd.print("----");
        break;         
        
      case 2:
        lcd.print("COR :"); 
        if (statusJardino.correction != 0)
        {
          lcd.print(statusJardino.correction);
          lcd.print("%");
        }
        else
          lcd.print("+0%");
        break;

      case 3:
        sprintf(buffer_str,"NTP :%02d:%02d %02d/%s/%02d",hour(CE.toLocal(last_ntp)),
                                    minute(last_ntp),
                                    day(last_ntp),moy[month(last_ntp)],year(last_ntp)-2000);  
        lcd.print(buffer_str);   
        break; 

      case 4:
        sprintf(buffer_str,"UP  :%03d %02d:%02d",uptime/(3600*24),
                                    (uptime/3600)% 24,
                                    (uptime/60)%60);    
        lcd.print(buffer_str); 
        break;

      case 5:
        sprintf(buffer_str,"VER :%s",VERSION);

        lcd.print(buffer_str); 
        break;
       
    }
    info_line++;
  }
}

// ==========================================================
// Rotary decoder TEST
// ==========================================================
typedef int32_t rotary_encoder_position_t;

/**
 * @brief Enum representing the direction of rotation.
 */
typedef enum
{
    ROTARY_ENCODER_DIRECTION_NOT_SET = 0,        ///< Direction not yet known (stationary since reset)
    ROTARY_ENCODER_DIRECTION_CLOCKWISE,
    ROTARY_ENCODER_DIRECTION_COUNTER_CLOCKWISE,
} rotary_encoder_direction_t;

// Used internally
///@cond INTERNAL
#define TABLE_COLS 4
typedef uint8_t table_row_t[TABLE_COLS];
///@endcond

/**
 * @brief Struct represents the current state of the device in terms of incremental position and direction of last movement
 */
typedef struct
{
    rotary_encoder_position_t position;    ///< Numerical position since reset. This value increments on clockwise rotation, and decrements on counter-clockewise rotation. Counts full or half steps depending on mode. Set to zero on reset.
    rotary_encoder_direction_t direction;  ///< Direction of last movement. Set to NOT_SET on reset.
} rotary_encoder_state_t;

/**
 * @brief Struct carries all the information needed by this driver to manage the rotary encoder device.
 *        The fields of this structure should not be accessed directly.
 */
typedef struct
{
    const table_row_t * table;              ///< Pointer to active state transition table
    uint8_t table_state;                    ///< Internal state
    volatile rotary_encoder_state_t state;  ///< Device state
} rotary_encoder_info_t;

/**
 * @brief Struct represents a queued event, used to communicate current position to a waiting task
 */
typedef struct
{
    rotary_encoder_state_t state;  ///< The device state corresponding to this event
} rotary_encoder_event_t;



#define TABLE_ROWS 7

#define DIR_NONE 0x0   // No complete step yet.
#define DIR_CW   0x10  // Clockwise step.
#define DIR_CCW  0x20  // Anti-clockwise step.

// Create the half-step state table (emits a code at 00 and 11)
#define R_START       0x0
#define H_CCW_BEGIN   0x1
#define H_CW_BEGIN    0x2
#define H_START_M     0x3
#define H_CW_BEGIN_M  0x4
#define H_CCW_BEGIN_M 0x5

static const uint8_t _ttable_half[TABLE_ROWS][TABLE_COLS] = {
    // 00                  01              10            11                   // BA
    {H_START_M,            H_CW_BEGIN,     H_CCW_BEGIN,  R_START},            // R_START (00)
    {H_START_M | DIR_CCW,  R_START,        H_CCW_BEGIN,  R_START},            // H_CCW_BEGIN
    {H_START_M | DIR_CW,   H_CW_BEGIN,     R_START,      R_START},            // H_CW_BEGIN
    {H_START_M,            H_CCW_BEGIN_M,  H_CW_BEGIN_M, R_START},            // H_START_M (11)
    {H_START_M,            H_START_M,      H_CW_BEGIN_M, R_START | DIR_CW},   // H_CW_BEGIN_M
    {H_START_M,            H_CCW_BEGIN_M,  H_START_M,    R_START | DIR_CCW},  // H_CCW_BEGIN_M
};

// Create the full-step state table (emits a code at 00 only)
#  define F_CW_FINAL  0x1
#  define F_CW_BEGIN  0x2
#  define F_CW_NEXT   0x3
#  define F_CCW_BEGIN 0x4
#  define F_CCW_FINAL 0x5
#  define F_CCW_NEXT  0x6

static const uint8_t table_full[TABLE_ROWS][TABLE_COLS] = {
    // 00        01           10           11                  // BA
    {R_START,    F_CW_BEGIN,  F_CCW_BEGIN, R_START},           // R_START
    {F_CW_NEXT,  R_START,     F_CW_FINAL,  R_START | DIR_CW},  // F_CW_FINAL
    {F_CW_NEXT,  F_CW_BEGIN,  R_START,     R_START},           // F_CW_BEGIN
    {F_CW_NEXT,  F_CW_BEGIN,  F_CW_FINAL,  R_START},           // F_CW_NEXT
    {F_CCW_NEXT, R_START,     F_CCW_BEGIN, R_START},           // F_CCW_BEGIN
    {F_CCW_NEXT, F_CCW_FINAL, R_START,     R_START | DIR_CCW}, // F_CCW_FINAL
    {F_CCW_NEXT, F_CCW_FINAL, F_CCW_BEGIN, R_START},           // F_CCW_NEXT
};

volatile uint8_t table_state = 0;


ICACHE_RAM_ATTR
void ISR_Encoder() {
    uint8_t event = 0;
    int LSB = digitalRead(ENCODER_CK); //MSB = most significant bit
    int MSB = digitalRead(ENCODER_DT); //LSB = least significant bit

    int pin_state = (MSB << 1) | LSB;

    table_state = table_full[table_state & 0x7][pin_state];

    event = table_state;

    if (event == F_CW_NEXT)
      encoderValue = -1;
    else if (event == F_CCW_NEXT)
      encoderValue = +1;
    else
      encoderValue = 0;

}
// =====================================================
// Rotary Encoder interrupt handler
// =====================================================
ICACHE_RAM_ATTR
void _ISR_Encoder() {

  int LSB = digitalRead(ENCODER_CK); //MSB = most significant bit
  int MSB = digitalRead(ENCODER_DT); //LSB = least significant bit

  uint8_t encoded = lastEncoded & 3;
  if (MSB) encoded |= 4;
  if (LSB) encoded |= 8;
  lastEncoded = (encoded >> 2);

  //Serial.printf("en: %d\n",encoded);
  
  switch (encoded) {
    
      case 1: case 7: case 8: case 14:
        encoderValue = +1; // ++
        return;
      case 2: case 4: case 11: case 13: 
        encoderValue = -1; // --
        return;
      case 3: case 12:
        encoderValue = +1; // +=2
        return;
      case 6: case 9:
        encoderValue = -1; // -=2
        return;
      case 15:
        encoderValue = 0; // Valor no contemplado
        return;
  }

}

// ==========================================================

volatile unsigned int encoder0Pos = 0;

void __doEncoder() {
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
  if (digitalRead(ENCODER_CK) == HIGH) {   // found a low-to-high on channel A
    if (digitalRead(ENCODER_DT) == LOW) {  // check channel B to see which way
                                             // encoder is turning
      encoder0Pos = encoder0Pos - 1;         // CCW
    } 
    else {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
  }
  else                                        // found a high-to-low on channel A
  { 
    if (digitalRead(ENCODER_DT) == LOW) {   // check channel B to see which way
                                              // encoder is turning  
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }

  }

  //Serial.println (encoder0Pos, DEC);
}

#ifdef JSON_COMMANDS
//--------------------------------------------------------------------------------------------
// JSON parser
//--------------------------------------------------------------------------------------------

int jsonend = 0;
boolean startJson = false;
#define JSON_BUFF_DIMENSION 1024

// JSON COMMANDS:
// { "command" : "readall"    , 
//   "seq" : 1 }

// { "command" : "readconfig" , 
//   "seq" : 1 }

// { "command" : "readprg"    , 
//   "prg" : 1, 
//   "seq" : 1 }

// { "command" : "manual"     , 
//   "zone_durations" :[20,30,30,30,30,30], 
//   "seq" : 1 }

// { "command" : "program"     , 
//   "prg"     : 1,
//   "start_hour : 1,
//   "start_minute : 1,
//   "days : 1,
//   "zone_durations" :[20,30,30,30,30,30], 
//   "seq" : 1 }

// { "command" : "cancel" , 
//   "seq" : 1 }

// { "command" : "set_mode" ,
//   "on_off"  : true, 
//   "seq" : 1 }

// { "command" : "set_config" ,
//   "correction"  : true, 
//   "delay_z"  : 1,
//   "check_current"  : false,
//   "seq" : 1 }

void parseJson_Command(const char * jsonString) {
  //StaticJsonBuffer<JSON_BUFF_DIMENSION> jsonBuffer;
  const size_t bufferSize = JSON_BUFF_DIMENSION;
  DynamicJsonBuffer jsonBuffer(bufferSize);

  // FIND FIELDS IN JSON TREE
  JsonObject& root = jsonBuffer.parseObject(jsonString);
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return;
  }

  String command = root["command"];
  if (command.equals("readll"))
  {
    Serial.println("Command readall");
  }
  
}

#endif
