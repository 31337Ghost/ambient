#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <SoftwareSerial.h>
#include <NeoPixelBus.h>
#include <NeoPixelAnimator.h>
#include <ArduinoJson.h>
#include "FS.h"

#define VERSION "0.0.2"

#define COMMAND_BUFFER_SIZE 128
#define CONFIG_PATH           "/colors.bin"
#define CONFIG_CURRENT_PATH   "/current.bin"
#define COLOR_COUNT 32
#define FADE_TIME   800

#define UPGRADE_FIRMWARE_TIMEOUT 600000
#define UPGRADE_BUFLEN 64

#define PIXEL_TURN_STEP     1
#define PIXEL_TURN_DURATION_STEP  2
#define PIXEL_TAIL_TURN_DURATION_STEP 10
#define TURN_TIMEOUT        500
#define PIN_PIXEL           2
#define PIN_TURN_LEFT       13  // D7 GPIO13
#define PIN_TURN_RIGHT      4   // D2 GPIO4
#define FREE                0   // D3 GPIO0 need to be used together with GPIO2:
#define FREE2               2   // D4 GPIO2 http://www.instructables.com/id/ESP8266-Using-GPIO0-GPIO2-as-inputs/
#define PIN_BRAKE           5   // D1 GPIO5
//#define BUTTON_PIN 5          // removed due to not need

const int TURN_FILL_ANIMATION  = 1;
const int TURN_FINISH_STEPS = 30;
const int TAIL_TURN_FINISH_STEPS = 30;

const int PIXEL_COUNT_TAIL = 24;
const int PIXEL_COUNT_TAIL_TURN = PIXEL_COUNT_TAIL / 2;
const int PIXEL_TURN_COUNT = 20;      // leds count running as turn signal
const int PIXEL_COUNT_SIDE = 60;      // leds count on side to ambient and turn processing
const int PIXEL_TURN_FILL_COUNT = 2;  // leds count, to fill by
const int PIXEL_TAIL_TURN_FILL_COUNT = 1; // same for tail turn
const int PIXEL_COUNT = PIXEL_COUNT_SIDE * 2 + PIXEL_COUNT_TAIL;
const int PIXEL_TURN_LEFT_START = 1;
const int PIXEL_TURN_LEFT_END   = PIXEL_COUNT_SIDE;
const int PIXEL_TURN_RIGHT_START = PIXEL_TURN_LEFT_END + 1;
const int PIXEL_TURN_RIGHT_END   = PIXEL_TURN_LEFT_END + PIXEL_COUNT_SIDE - 1;
const int PIXEL_TAIL_START = PIXEL_TURN_RIGHT_END + 1;
const int PIXEL_TAIL_END = PIXEL_TAIL_START + PIXEL_COUNT_TAIL - 1;
const int PIXEL_TAIL_TURN_RIGHT_END   = PIXEL_TAIL_START;
const int PIXEL_TAIL_TURN_RIGHT_START = PIXEL_TAIL_START + PIXEL_COUNT_TAIL_TURN - 1;
const int PIXEL_TAIL_TURN_LEFT_START =  PIXEL_TAIL_TURN_RIGHT_START + 1;                      // 133
const int PIXEL_TAIL_TURN_LEFT_END   = PIXEL_TAIL_TURN_RIGHT_START + PIXEL_COUNT_TAIL_TURN;   // 143

const int PIXEL_AMBIENT_START = PIXEL_TURN_LEFT_START;
const int PIXEL_AMBIENT_END = PIXEL_TURN_RIGHT_END;

const char *ssid = "Ambient";
const char *password = "00000000";
const char* update_path = "/";
const char* update_username = "admin";
const char* update_password = "admin";

const char command_success    = 0x01;
const char command_fail       = 0x00;
const char command_ping       = 0x01;
const char command_current    = 0x02;
const char command_write      = 0x03;
const char command_render     = 0x04;
const char command_list       = 0x05;
const char command_set        = 0x06;
const char command_get        = 0x07;
const char command_version    = 0x08;
const char command_upgrade    = 0x09;
const char command_end        = 0x0A; // \n
const char command_clear      = 0x0B;

typedef struct
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t w;
} RGBW_Color;

RGBW_Color    colors[COLOR_COUNT];
uint8_t       colorCurrent = 0;
RGBW_Color    colorCurrentRGBW;
RGBW_Color    colorNeedRGBW;

RgbwColor colorTurnRGBW(255, 126, 0, 0);
RgbwColor colorOldRGBW(0, 0, 0, 0);
RgbwColor colorNeedRGBWfaded;
RgbwColor black(0, 0, 0, 0);
RgbwColor colorParkRGBW(128, 0, 0, 0);
RgbwColor colorBreakRGBW(255, 0, 0, 0);
RgbwColor colorTailRGBW(128, 0, 0, 0);
RgbwColor colorTailRGBWfaded;

unsigned int  should_write_delay = 10000;
unsigned long should_write_at = 0;

const size_t COMMAND_JSON_SIZE = JSON_OBJECT_SIZE(6) + 50;

const uint8_t   AnimationChannels = 5;
char            command_buffer[COMMAND_BUFFER_SIZE] = {0};
int             command_buffer_position = 0;
unsigned long   lastNow = 0;

bool            isUpgrading = false;
unsigned long   upgradeSize = 0;
unsigned long   upgradeTimeoutAt = 0;
String          upgradeMD5 = "";
uint8_t         upgradeBuf[UPGRADE_BUFLEN];
unsigned int    upgradeBufPos = 0;
unsigned int    blocksRecieved = 0;

SoftwareSerial BT_Serial(D5, D6);

NeoPixelBus<NeoGrbwFeature, Neo800KbpsMethod> neoPixel(PIXEL_COUNT, PIN_PIXEL);
NeoPixelAnimator animations(AnimationChannels);
NeoGamma<NeoGammaTableMethod> colorGamma;

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater(true);

uint8_t buttonLastState = HIGH;
uint8_t turnLeftLastState = HIGH;
uint8_t turnRightLastState = HIGH;

unsigned long turnLeftDebounceTimeoutAt = 0;
unsigned long turnRightDebounceTimeoutAt = 0;
#define TURN_DEBOUNCE_TIMEOUT     300

unsigned long turnLeftTimeoutAt = 0;
unsigned int turnLeftStepsLeft = 0;
unsigned long turnRightTimeoutAt = 0;
unsigned int turnRightStepsLeft = 0;

unsigned int turnTailRightStepsLeft = 0;
unsigned int turnTailLeftStepsLeft = 0;

uint8_t brakeLastState = HIGH;
uint8_t brakeCurrentState = false;
unsigned long brakeDebounceTimeoutAt = 0;
#define BRAKE_DEBOUNCE_TIMEOUT     150

struct MyAnimationState
{
  RgbwColor StartingColor;
  RgbwColor EndingColor;
};

MyAnimationState animationState[AnimationChannels];

void AnimUpdateTailTurnRight(const AnimationParam& param);
void AnimUpdateTurnLeft(const AnimationParam& param);
void AnimUpdateTurnRight(const AnimationParam& param);
void AnimUpdateFade(const AnimationParam& param);

void init_button();
void init_colors();
bool write_colorCurrent();
bool write_config();
bool read_config();
void disable_wifi();

void setup() {
  disable_wifi();
  init_button();

  Serial.begin(115200);
  Serial.println();
  Serial.println("Initializing...");
  Serial.flush();

  neoPixel.Begin();
  neoPixel.Show();

  delay(1000);
  Serial.println("Mounting FS...");
  if (!SPIFFS.begin()) {
    Serial.println("Failed to mount file system");
    return;
  }

  Serial.println("Files in /:");
  Dir dir = SPIFFS.openDir("/");
  while (dir.next()) {
    Serial.print(dir.fileName());
    Serial.print(" ");
    File f = dir.openFile("r");
    Serial.println(f.size());
  }

  if (!read_config()) {
    init_colors();
    write_config();
  }

  for (int i = 0; i < COLOR_COUNT; i++) {
    Serial.printf("%d %d %d %d\n", colors[i].r, colors[i].g, colors[i].b, colors[i].w);
  }

  colorCurrentRGBW = colorNeedRGBW = colors[0];

  Serial.printf("Rendering %d color from pallete\n", colorCurrent);
  processRender(colorCurrent);

  neoPixelPark();

  BT_Serial.begin(9600);
  delay(1000);
  BT_Serial.println("AT+NAMEAmbient");

  Serial.println("Init complete");
}

void loop() {
  // If in upgrading mode - only upgrading supported
  if (isUpgrading) {
    httpServer.handleClient();
    return;
  }

  if (brakeLastState != digitalRead(PIN_BRAKE)) {
    if (!brakeDebounceTimeoutAt) {
      brakeLastState = digitalRead(PIN_BRAKE);
      brakeDebounceTimeoutAt = millis() + BRAKE_DEBOUNCE_TIMEOUT;
      if (brakeLastState == LOW ) {
        Serial.println("Brake pressed");
        neoPixelBreak();
        brakeCurrentState = true;
      } else if (brakeLastState == HIGH) {
        Serial.println("Brake released");
        neoPixelPark();
        brakeCurrentState = false;
      }
    }
  }

  if (brakeDebounceTimeoutAt && millis() > brakeDebounceTimeoutAt) {
    brakeDebounceTimeoutAt = 0;
  }

  if (turnLeftLastState != digitalRead(PIN_TURN_LEFT)) {
    turnLeftLastState = digitalRead(PIN_TURN_LEFT);
    if (turnLeftLastState == LOW && !turnLeftDebounceTimeoutAt) {
      turnLeftDebounceTimeoutAt = millis() + TURN_DEBOUNCE_TIMEOUT;
      Serial.println("Turn left");
      neoPixelTurnLeft();
    }
  }

  if (turnLeftDebounceTimeoutAt && millis() > turnLeftDebounceTimeoutAt) {
    turnLeftDebounceTimeoutAt = 0;
  }

  if (turnLeftTimeoutAt && millis() > turnLeftTimeoutAt) {
    turnLeftTimeoutAt = 0;
    Serial.println("Turn left ended");
    // Finaly set all strip not faded
    neoPixelClearTo(colorNeedRGBW, PIXEL_TURN_LEFT_START, PIXEL_TURN_LEFT_END);
  }

  if (turnRightLastState != digitalRead(PIN_TURN_RIGHT)) {
    turnRightLastState = digitalRead(PIN_TURN_RIGHT);
    if (turnRightLastState == LOW && !turnRightDebounceTimeoutAt) {
      turnRightDebounceTimeoutAt = millis() + TURN_DEBOUNCE_TIMEOUT;
      Serial.println("Turn right");
      neoPixelTurnRight();
    }
  }

  if (turnRightDebounceTimeoutAt && millis() > turnRightDebounceTimeoutAt) {
    turnRightDebounceTimeoutAt = 0;
  }

  if (turnRightTimeoutAt && millis() > turnRightTimeoutAt) {
    turnRightTimeoutAt = 0;
    Serial.println("Turn right ended");
    // Finaly set all strip not faded
    neoPixelClearTo(colorNeedRGBW, PIXEL_TURN_RIGHT_START, PIXEL_TURN_RIGHT_END);
  }

  /*if (buttonLastState != digitalRead(BUTTON_PIN)) {
    buttonLastState = digitalRead(BUTTON_PIN);
    if (buttonLastState == HIGH) {
      Serial.println("Touched");
      colorCurrent++;
      if (colorCurrent >= COLOR_COUNT) {
        colorCurrent = 0;
      } else if (colors[colorCurrent].r == 0
                 && colors[colorCurrent].g == 0
                 && colors[colorCurrent].b == 0
                 && colors[colorCurrent].w == 0 ) {
        colorCurrent = 0;
      }
      colorNeedRGBW = colors[colorCurrent];
      neoPixelFadeTo(colorNeedRGBW);
      processCurrent();
      should_write_at = millis() + should_write_delay;
    }
    }*/

  if (should_write_at && millis() > should_write_at ) {
    should_write_at = 0;
    write_colorCurrent();
  }

  if (BT_Serial.available()) {
    char _char = BT_Serial.read();
    if (_char == '\n') {

      Serial.print(command_buffer);
      Serial.println();

      StaticJsonBuffer<COMMAND_JSON_SIZE> jsonBuffer;
      JsonObject& command = jsonBuffer.parseObject(command_buffer);

      //command.prettyPrintTo(Serial);

      if (command.containsKey("cmd")) {
        if (command["cmd"] == "get") {
          if (command.containsKey("n")) {
            unsigned int n = command.get<unsigned int>("n");
            Serial.printf("Get command for n %d\n", n);
            processGet(n);
          }
        } else if (command["cmd"] == "set") {
          if (command.containsKey("n") && command.containsKey("r")
              && command.containsKey("g") && command.containsKey("b")
              && command.containsKey("w"))
          {
            unsigned int n = command.get<unsigned int>("n");
            unsigned int r = command.get<unsigned int>("r");
            unsigned int g = command.get<unsigned int>("g");
            unsigned int b = command.get<unsigned int>("b");
            unsigned int w = command.get<unsigned int>("w");
            Serial.printf("Set command for n %d to color (R=%d, G=%d, B=%d, W=%d)\n", n, r, g, b, w);
            processSet(n, r, g, b, w);
          }
        } else if (command["cmd"] == "list") {
          Serial.println("List command found");
          processList();
        } else if (command["cmd"] == "render") {
          if (command.containsKey("n")) {
            unsigned int n = command.get<unsigned int>("n");
            Serial.printf("Render n %d color\n", n);
            processRender(n);
          } else {
            if (command.containsKey("r") && command.containsKey("g")
                && command.containsKey("b") && command.containsKey("w"))
            {
              unsigned int r = command.get<unsigned int>("r");
              unsigned int g = command.get<unsigned int>("g");
              unsigned int b = command.get<unsigned int>("b");
              unsigned int w = command.get<unsigned int>("w");
              Serial.printf("Render color (R=%d, G=%d, B=%d, W=%d)\n", r, g, b, w);
              processRender(r, g, b, w);
            }
          }
        } else if (command["cmd"] == "write") {
          Serial.println("Write command found");
          processWriteAll();
        } else if (command["cmd"] == "current") {
          Serial.println("Current command found");
          processCurrent();
        } else if (command["cmd"] == "ping") {
          Serial.println("Ping command found");
          processPing();
        } else if (command["cmd"] == "version") {
          Serial.println("Version command found");
          processVersion();
        } else if (command["cmd"] == "upgrade") {
          if (command.containsKey("size") && command.containsKey("md5")) {
            unsigned long sz = command.get<unsigned long>("size");
            String md5 = command.get<String>("md5");
            Serial.printf("Upgrade, new FW size: %d, MD5: %s\n", sz, md5.c_str());
            processUpgrade(sz, md5);
          }
        } else if (command["cmd"] == "clear") {
          Serial.println("Clear command found");
          processClear();
        }
      }

      memset(command_buffer, 0, COMMAND_BUFFER_SIZE);
      command_buffer_position = 0;
    } else {
      command_buffer[command_buffer_position++] = _char;
    }
  }

  if (animations.IsAnimating()) {
    animations.UpdateAnimations();
    neoPixel.Show();
  }
}

void AnimUpdateTailTurnLeft(const AnimationParam& param) {
  if (param.state == AnimationState_Completed) {
    if (turnTailLeftStepsLeft > 0) {
      turnTailLeftStepsLeft--;
      animations.RestartAnimation(param.index);

      Serial.printf("%d %d %d %d\n", turnTailLeftStepsLeft, PIXEL_TAIL_TURN_FILL_COUNT, PIXEL_TAIL_TURN_LEFT_START, PIXEL_TAIL_TURN_LEFT_END);

      neoPixel.ShiftRight(PIXEL_TAIL_TURN_FILL_COUNT, PIXEL_TAIL_TURN_LEFT_START, PIXEL_TAIL_TURN_LEFT_END);

    } else {
      // Ending reached
      if (brakeCurrentState) {
        neoPixelClearTo(colorBreakRGBW, PIXEL_TAIL_TURN_LEFT_START, PIXEL_TAIL_TURN_LEFT_END);
      } else {
        neoPixelClearTo(colorParkRGBW, PIXEL_TAIL_TURN_LEFT_START, PIXEL_TAIL_TURN_LEFT_END);
      }
    }
  }
}

void AnimUpdateTailTurnRight(const AnimationParam& param) {
  if (param.state == AnimationState_Completed) {
    if (turnTailRightStepsLeft > 0) {
      turnTailRightStepsLeft--;
      animations.RestartAnimation(param.index);

      neoPixel.ShiftLeft(PIXEL_TAIL_TURN_FILL_COUNT, PIXEL_TAIL_TURN_RIGHT_END, PIXEL_TAIL_TURN_RIGHT_START);

    } else {
      // Ending reached
      if (brakeCurrentState) {
        neoPixelClearTo(colorBreakRGBW, PIXEL_TAIL_TURN_RIGHT_END, PIXEL_TAIL_TURN_RIGHT_START); // reversed
      } else {
        neoPixelClearTo(colorParkRGBW, PIXEL_TAIL_TURN_RIGHT_END, PIXEL_TAIL_TURN_RIGHT_START); // reversed
      }
    }
  }
}

void AnimUpdateTurnLeft(const AnimationParam& param) {
  if (param.state == AnimationState_Completed) {
    if (turnLeftStepsLeft > 0) {
      turnLeftStepsLeft--;
      animations.RestartAnimation(param.index);

      if (TURN_FILL_ANIMATION) {
        neoPixel.ShiftLeft(PIXEL_TURN_FILL_COUNT, PIXEL_TURN_LEFT_START, PIXEL_TURN_LEFT_END);
      } else {
        neoPixel.RotateLeft(PIXEL_TURN_STEP, PIXEL_TURN_LEFT_START, PIXEL_TURN_LEFT_END);
      }

      turnLeftTimeoutAt = millis() + TURN_TIMEOUT;
    } else {
      // Ending reached
      neoPixelClearTo(colorNeedRGBWfaded, PIXEL_TURN_LEFT_START, PIXEL_TURN_LEFT_END);
    }
  }
}

void AnimUpdateTurnRight(const AnimationParam& param) {
  if (param.state == AnimationState_Completed) {
    if (turnRightStepsLeft > 0) {
      turnRightStepsLeft--;
      animations.RestartAnimation(param.index);

      if (TURN_FILL_ANIMATION) {
        neoPixel.ShiftRight(PIXEL_TURN_FILL_COUNT, PIXEL_TURN_RIGHT_START, PIXEL_TURN_RIGHT_END);
      } else {
        neoPixel.RotateRight(PIXEL_TURN_STEP, PIXEL_TURN_RIGHT_START, PIXEL_TURN_RIGHT_END);
      }

      turnRightTimeoutAt = millis() + TURN_TIMEOUT;
    } else {
      // Ending reached
      neoPixelClearTo(colorNeedRGBWfaded, PIXEL_TURN_RIGHT_START, PIXEL_TURN_RIGHT_END);
    }
  }
}

void AnimUpdateFade(const AnimationParam& param) {
  RgbwColor updatedColor = RgbwColor::LinearBlend(
                             animationState[param.index].StartingColor,
                             animationState[param.index].EndingColor,
                             param.progress);
  neoPixelClearTo(updatedColor, PIXEL_AMBIENT_START, PIXEL_AMBIENT_END);
}

void neoPixelPark() {
  colorTailRGBW = colorParkRGBW;
  neoPixelClearTo(colorParkRGBW, PIXEL_TAIL_START, PIXEL_TAIL_END);
}

void neoPixelBreak() {
  colorTailRGBW = colorBreakRGBW;
  neoPixelClearTo(colorBreakRGBW, PIXEL_TAIL_START, PIXEL_TAIL_END);
}

void neoPixelTurnLeft() {
  animations.StopAnimation(1);
  RgbwColor base(colorNeedRGBW.r, colorNeedRGBW.g, colorNeedRGBW.b, colorNeedRGBW.w);
  RgbwColor base_faded = RgbwColor::LinearBlend(
                           base,
                           black,
                           0.7f);
  colorNeedRGBWfaded = base_faded;
  neoPixelClearTo(colorNeedRGBWfaded, PIXEL_TURN_LEFT_START, PIXEL_TURN_LEFT_END);

  animations.StopAnimation(3);
  RgbwColor baseTail_faded = RgbwColor::LinearBlend(
                           colorTailRGBW,
                           black,
                           0.7f);
  colorTailRGBWfaded = baseTail_faded;
  neoPixelClearTo(colorTailRGBWfaded, PIXEL_TAIL_TURN_LEFT_START, PIXEL_TAIL_TURN_LEFT_END);

  if (TURN_FILL_ANIMATION) {
    // Front turn
    neoPixelClearTo(colorTurnRGBW, PIXEL_TURN_LEFT_END - PIXEL_TURN_FILL_COUNT, PIXEL_TURN_LEFT_END);
    turnLeftStepsLeft = (PIXEL_COUNT_SIDE - PIXEL_TURN_FILL_COUNT) / PIXEL_TURN_STEP + TURN_FINISH_STEPS;
    // Tail turn
    neoPixelClearTo(colorTurnRGBW, PIXEL_TAIL_TURN_LEFT_START, PIXEL_TAIL_TURN_LEFT_START + PIXEL_TAIL_TURN_FILL_COUNT - 1);
    turnTailLeftStepsLeft = (PIXEL_COUNT_TAIL_TURN - PIXEL_TAIL_TURN_FILL_COUNT) / PIXEL_TURN_STEP + TAIL_TURN_FINISH_STEPS;
  } else {
    neoPixelClearTo(colorTurnRGBW, PIXEL_TURN_LEFT_END - PIXEL_TURN_COUNT, PIXEL_TURN_LEFT_END);
    turnLeftStepsLeft = (PIXEL_COUNT_SIDE - PIXEL_TURN_COUNT) / PIXEL_TURN_STEP - 2;
  }

  turnLeftTimeoutAt = millis() + TURN_TIMEOUT;
  animations.StartAnimation(1, PIXEL_TURN_DURATION_STEP, AnimUpdateTurnLeft);
  animations.StartAnimation(3, PIXEL_TAIL_TURN_DURATION_STEP, AnimUpdateTailTurnLeft);
}

void neoPixelTurnRight() {
  animations.StopAnimation(2);
  RgbwColor base(colorNeedRGBW.r, colorNeedRGBW.g, colorNeedRGBW.b, colorNeedRGBW.w);
  RgbwColor base_faded = RgbwColor::LinearBlend(
                           base,
                           black,
                           0.7f);
  colorNeedRGBWfaded = base_faded;
  neoPixelClearTo(colorNeedRGBWfaded, PIXEL_TURN_RIGHT_START, PIXEL_TURN_RIGHT_END);

  animations.StopAnimation(4);
  RgbwColor baseTail_faded = RgbwColor::LinearBlend(
                           colorTailRGBW,
                           black,
                           0.7f);
  colorTailRGBWfaded = baseTail_faded;
  neoPixelClearTo(colorTailRGBWfaded, PIXEL_TAIL_TURN_RIGHT_END, PIXEL_TAIL_TURN_RIGHT_START);

  if (TURN_FILL_ANIMATION) {
    // Front turn
    neoPixelClearTo(colorTurnRGBW, PIXEL_TURN_RIGHT_START, PIXEL_TURN_RIGHT_START + PIXEL_TURN_FILL_COUNT);
    turnRightStepsLeft = (PIXEL_COUNT_SIDE - PIXEL_TURN_FILL_COUNT) / PIXEL_TURN_STEP + TURN_FINISH_STEPS;
    // Tail turn
    neoPixelClearTo(colorTurnRGBW, PIXEL_TAIL_TURN_RIGHT_START - PIXEL_TAIL_TURN_FILL_COUNT + 1, PIXEL_TAIL_TURN_RIGHT_START);
    turnTailRightStepsLeft = (PIXEL_COUNT_TAIL_TURN - PIXEL_TAIL_TURN_FILL_COUNT) / PIXEL_TURN_STEP + TAIL_TURN_FINISH_STEPS;
  } else {
    neoPixelClearTo(colorTurnRGBW, PIXEL_TURN_RIGHT_START, PIXEL_TURN_RIGHT_START + PIXEL_TURN_COUNT);
    turnRightStepsLeft = (PIXEL_COUNT_SIDE - PIXEL_TURN_COUNT) / PIXEL_TURN_STEP - 2;
  }

  turnRightTimeoutAt = millis() + TURN_TIMEOUT;
  animations.StartAnimation(2, PIXEL_TURN_DURATION_STEP, AnimUpdateTurnRight);
  animations.StartAnimation(4, PIXEL_TAIL_TURN_DURATION_STEP, AnimUpdateTailTurnRight);
}

void neoPixelFadeTo(RGBW_Color targetColor) {
  RgbwColor target(targetColor.r, targetColor.g, targetColor.b, targetColor.w);
  animationState[0].StartingColor = colorOldRGBW;
  animationState[0].EndingColor = target;
  colorOldRGBW = target;

  animations.StartAnimation(0, FADE_TIME, AnimUpdateFade);
}

void neoPixelClearTo(RgbwColor target, uint16_t first, uint16_t last) {
  target = correctGamma(target);
  neoPixel.ClearTo(target, first, last);
  neoPixel.Show();
}

void neoPixelClearTo(RGBW_Color targetColor, uint16_t first, uint16_t last) {
  RgbwColor target(targetColor.r, targetColor.g, targetColor.b, targetColor.w);
  neoPixelClearTo(target, first, last);
}

void feedUpgradeTimeout() {
  upgradeTimeoutAt = millis() + UPGRADE_FIRMWARE_TIMEOUT;
}

void processClear() {
  BT_Serial.write(command_clear);
  init_colors();
  BT_Serial.write(command_success);
  BT_Serial.write(command_end);
}

void processUpgrade(unsigned long sz, String md5) {
  upgradeSize = sz;
  isUpgrading = true;
  upgradeMD5 = md5;
  /*if (md5.length()) {
    if (!Update.setMD5(md5.c_str())) {
      _lastError = HTTP_UE_SERVER_FAULTY_MD5;
      DEBUG_HTTP_UPDATE("[httpUpdate] Update.setMD5 failed! (%s)\n", md5.c_str());
      return false;
    }
    }*/
  blocksRecieved = 0;
  upgradeBufPos = 0;
  memset(upgradeBuf, 0, UPGRADE_BUFLEN);
  feedUpgradeTimeout();
  IPAddress myIP = updateServerUp();
  Serial.print("Upgrade server is up, IP: ");
  Serial.println(myIP);
  BT_Serial.write(command_upgrade);
  BT_Serial.write(command_success);
  BT_Serial.print(ssid);
  BT_Serial.print(F(" "));
  BT_Serial.print(password);
  BT_Serial.print(F(" "));
  BT_Serial.print(myIP);
  BT_Serial.print(F(" "));
  BT_Serial.print(update_username);
  BT_Serial.print(F(" "));
  BT_Serial.print(update_password);
  BT_Serial.write(command_end);
}

void processVersion() {
  BT_Serial.write(command_version);
  BT_Serial.write(command_success);
  BT_Serial.print(VERSION);
  BT_Serial.print(F(" "));
  BT_Serial.print(WiFi.macAddress());
  BT_Serial.write(command_end);
}

void processPing() {
  BT_Serial.write(command_ping);
  BT_Serial.write(command_success);
  BT_Serial.write(command_end);
}

void processCurrent() {
  BT_Serial.write(command_current);
  BT_Serial.write(command_success);
  BT_Serial.write((const uint8_t *)&colorCurrent, sizeof(colorCurrent));
  BT_Serial.write(command_end);
}

void processRender(unsigned int n) {
  BT_Serial.write(command_render);
  if (n < COLOR_COUNT) {
    colorCurrent = n;
    colorNeedRGBW = colors[colorCurrent];

    neoPixelFadeTo(colorNeedRGBW);
    BT_Serial.write(command_success);
  } else {
    BT_Serial.write(command_fail);
  }
  BT_Serial.write(command_end);
}

void processRender(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  BT_Serial.write(command_render);
  colorNeedRGBW.r = r;
  colorNeedRGBW.g = g;
  colorNeedRGBW.b = b;
  colorNeedRGBW.w = w;

  neoPixelFadeTo(colorNeedRGBW);
  BT_Serial.write(command_success);
  BT_Serial.write(command_end);
}

void processWriteColorCurrent() {
  BT_Serial.write(command_write);
  if (write_colorCurrent()) {
    BT_Serial.write(command_success);
  } else {
    BT_Serial.write(command_fail);
  }
  BT_Serial.write(command_end);
}

void processWriteAll() {
  BT_Serial.write(command_write);
  if (write_config()) {
    BT_Serial.write(command_success);
  } else {
    BT_Serial.write(command_fail);
  }
  BT_Serial.write(command_end);
}

void processSet(unsigned int n, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  BT_Serial.write(command_set);
  if (n < COLOR_COUNT) {
    colors[n].r = r;
    colors[n].g = g;
    colors[n].b = b;
    colors[n].w = w;
    BT_Serial.write(command_success);
  } else {
    BT_Serial.write(command_fail);
  }
  BT_Serial.write(command_end);
}

void processGet(unsigned int n) {
  BT_Serial.write(command_get);
  if (n < COLOR_COUNT) {
    BT_Serial.write(command_success);
    BT_Serial.write((uint8_t *)&colors[n], sizeof(colors[n]));
  } else {
    BT_Serial.write(command_fail);
  }
  BT_Serial.write(command_end);
}

void processList() {
  BT_Serial.write(command_list);
  BT_Serial.write(command_success);
  for (int i = 0; i < COLOR_COUNT; i++) {
    BT_Serial.write((const uint8_t *)&colors[i], sizeof(colors[i]));
  }
  BT_Serial.write(command_end);
}

IPAddress updateServerUp() {
  WiFi.forceSleepWake();
  delay(1);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  httpUpdater.setup(&httpServer, update_path);
  //httpUpdater.setup(&httpServer, update_path, update_username, update_password);
  httpServer.begin();

  IPAddress myIP = WiFi.softAPIP();

  return myIP;
}

void updateServerDown() {
  disable_wifi();
  httpServer.close();
  Serial.print("Update server ended");
}

RgbwColor correctGamma(RgbwColor color) {
  return colorGamma.Correct(color);
}


void init_button() {
  pinMode(PIN_BRAKE, INPUT_PULLUP);
  pinMode(PIN_TURN_LEFT, INPUT_PULLUP);
  pinMode(PIN_TURN_RIGHT, INPUT_PULLUP);
}

void init_colors() {
  for (int i = 0; i < COLOR_COUNT; i++) {
    colors[i].r = 0;
    colors[i].g = 0;
    colors[i].b = 0;
    colors[i].w = 0;
  }
}

bool write_colorCurrent() {
  File f = SPIFFS.open(CONFIG_CURRENT_PATH, "w");
  if (!f) {
    Serial.println("Failed to open current file for writing");
    return false;
  }
  f.write((const uint8_t *)&colorCurrent, sizeof(colorCurrent));
  f.close();

  Serial.println("Color current saved");
  return true;
}

bool write_config() {
  Serial.println("");

  File f = SPIFFS.open(CONFIG_PATH, "w");
  if (!f) {
    Serial.println("Failed to open file for writing");
    return false;
  }
  for (int i = 0; i < COLOR_COUNT; i++) {
    f.write((const uint8_t *)&colors[i], sizeof(colors[i]));
  }
  f.close();

  Serial.println("Colors config saved");

  if (!write_colorCurrent()) {
    return false;
  }

  return true;
}

bool read_config() {
  File f = SPIFFS.open(CONFIG_PATH, "r");
  if (!f) {
    Serial.println("No config file found");
    return false;
  }
  size_t size = f.size();
  if (size != COLOR_COUNT * sizeof(colors[0])) {
    Serial.println("Config error");
    return false;
  }
  for (int i = 0; i < COLOR_COUNT; i++) {
    f.read((uint8_t *)&colors[i], sizeof(colors[i]));
  }
  f.close();

  f = SPIFFS.open(CONFIG_CURRENT_PATH, "r");
  if (!f) {
    Serial.println("No config current file found");
    return false;
  }
  size = f.size();
  if (size != sizeof(colorCurrent)) {
    Serial.println("Config current error");
    return false;
  }
  f.read((uint8_t *)&colorCurrent, sizeof(colorCurrent));
  f.close();

  Serial.println("Config loaded");
  return true;
}

void disable_wifi( ) {
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);
}
