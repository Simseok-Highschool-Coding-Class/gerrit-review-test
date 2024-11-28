#include <Arduino.h>
#include <TensorFlowLite_ESP32.h>
#include <NewPing.h>
#include <SoftwareSerial.h>
#include "DFRobotDFPlayerMini.h"

// 초음파 센서 핀
#define TRIG_PIN 14
#define ECHO_PIN 13
#define MAX_DISTANCE 150

// DFPlayer Mini 핀
#define DF_RX 16
#define DF_TX 17

// 모델과 카메라 초기화
#include <esp_camera.h>
#include <WiFi.h> // WiFi 비활성화
#include <TensorFlowLite.h>

tflite::MicroInterpreter *interpreter;
TfLiteTensor *input;
TfLiteTensor *output;

// 초음파 센서 및 DFPlayer Mini 객체 생성
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
SoftwareSerial mp3Serial(DF_RX, DF_TX);
DFRobotDFPlayerMini myDFPlayer;

// 거리 변수
long duration;
int distance;

// 카메라 설정
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// 모델 입력 크기
const int inputWidth = 96;
const int inputHeight = 96;
const int inputChannels = 1;

void setup()
{
    Serial.begin(115200);
    mp3Serial.begin(9600);

    // DFPlayer Mini 초기화
    if (!myDFPlayer.begin(mp3Serial))
    {
        while (true)
            ;
    }
    myDFPlayer.volume(20);

    // 초음파 센서 초기화
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // 카메라 초기화
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAMESIZE_96X96;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    // TensorFlow Lite 모델 로드
    static tflite::MicroMutableOpResolver<10> resolver;
    resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D, tflite::ops::micro::Register_CONV_2D());

    // 모델 설정 및 버퍼 할당
    interpreter = new tflite::MicroInterpreter(model, resolver, tensorArena, kTensorArenaSize, error_reporter);
    interpreter->AllocateTensors();
    input = interpreter->input(0);
    output = interpreter->output(0);

    myDFPlayer.play(1); // "Device is ready"
}

void loop()
{
    captureImage();
    int result = analyzeBrailleBlock();

    if (result == 1)
    {
        myDFPlayer.play(2); // "Braille block in front"
    }
    else if (result == 2)
    {
        myDFPlayer.play(3); // "Braille block on the left"
    }
    else if (result == 3)
    {
        myDFPlayer.play(4); // "Braille block on the right"
    }

    checkObstacle();
    delay(1000);
}

void captureImage()
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb)
    {
        Serial.println("Camera capture failed");
        return;
    }
    memcpy(input->data.uint8, fb->buf, inputWidth * inputHeight * inputChannels);
    esp_camera_fb_return(fb);
}

int analyzeBrailleBlock()
{
    interpreter->Invoke();
    int prediction = output->data.uint8[0];
    return prediction; // 1: 정면, 2: 왼쪽, 3: 오른쪽
}

void checkObstacle()
{
    duration = sonar.ping();
    distance = sonar.convert_cm(duration);
    if (distance > 0 && distance <= 150)
    {
        myDFPlayer.play(5); // "Obstacle ahead"
        delay(2000);
    }
}