#include <Arduino.h>
/**
notice the ESP32 cam
GPIO 0 is for the camera MCLK generation, connected to camera, if used to other function the camera will not work
GPIO16 is used for CS# of the PSRAM, which is not suitable for other application once you use camera

only
*/
// =============================== SETUP ======================================

// 1. Board setup (Uncomment):
// #define BOARD_WROVER_KIT
#define BOARD_ESP32CAM_AITHINKER

/**
 * 2. Kconfig setup
 * 
 * If you have a Kconfig file, copy the content from
 *  https://github.com/espressif/esp32-camera/blob/master/Kconfig into it.
 * In case you haven't, copy and paste this Kconfig file inside the src directory.
 * This Kconfig file has definitions that allows more control over the camera and
 * how it will be initialized.
 */

/**
 * 3. Enable PSRAM on sdkconfig:
 * 
 * CONFIG_ESP32_SPIRAM_SUPPORT=y
 * 
 * More info on
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/kconfig.html#config-esp32-spiram-support
 */
    
// ================================ CODE ======================================


#include "SPI.h"
#include <TFT_eSPI.h>
#include <IRremoteESP8266.h>
//#include <NewRemoteReceiver.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <NewRemoteTransmitter.h>
#include "FastLED.h"
//#include "strip_index.h"

#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
#include <driver/rtc_io.h>
#include "esp_sleep.h"
#include "esp_camera.h"
#include "sensor.h"
#include "strip_index.h"
#define CONFIG_CAMERA_CORE1 1
//#define CONFIG_CAMERA_CORE0 1
TaskHandle_t Task1;
TaskHandle_t Task2;

//** declare led strip data pin 
#define LED_PIN 15 // no #CS(GPIO15) required when using ili9341

#define NUM_LEDS 259
#define STRIP_WIDTH 84
#define STRIP_HEIGHT 45

#define CHIPSET WS2812
#define COLOR_ORDER RGB
bool Process = true;
uint32_t lastimage = millis();
uint32_t temp_time;
uint32_t LED_T0;
CRGB leds[NUM_LEDS]; // arry for store the RGB data for strip
CRGB LED_GAMMA_TEMP;
// minimun light when start the camera 30,20,16
CRGB min_bright=CRGB(30,20,16);
// temp file for calculation
struct pixel{
    uint16_t R_TEMP;
    uint16_t G_TEMP;
    uint16_t B_TEMP;
    uint16_t pix_TEMP;
    //uint8_t count;
} pixel_temp={0,0,0,0};

// max brightness setting from FastLED
#define BRIGHTNESS 235

// capture = 1 start return frame buffer and enable recording, =0 stop return frame buffer
bool CAPTURE=1;

//* declare the tft backlight, but there has no pin, issue to solve
#define TFT_BKL 33

TFT_eSPI tft = TFT_eSPI();
#define QVGA
#ifdef QQVGA
    #define IMG_WIDTH 160
    #define IMG_HEIGHT 120
    
#elif defined(QVGA)
    #define IMG_WIDTH 320
    #define IMG_HEIGHT 120 // only halft of image stored , not enough ram for process
#endif
// only halft of image stored 
uint16_t Buffer[IMG_WIDTH*IMG_HEIGHT];


//************ declare RF and ir related info
//**********************************************
// Create a transmitter on address 5977494, using a digital pin to transmit, 
// with a period duration of 260ms (default), repeating the transmitted
// code 2^2=4 times.
NewRemoteTransmitter transmitter(5977494, 12, 260, 2); // GPIO12 for transmitt

// An IR detector/demodulator is connected to GPIO pin 4
// Note: GPIO 16 won't work on the ESP8266 as it does not have interrupts.
#define IR_USE
const uint16_t kRecvPin = 4; // GPIO4 for transmitt
#if defined IR_USE
IRrecv irrecv(kRecvPin);
bool NEO_FLAG=0;
uint8_t NEO_TYPE=0; //* change the lighting method, with
#else
bool NEO_FLAG=0;
uint8_t NEO_TYPE=1; //* change the lighting method, with
#endif

RTC_DATA_ATTR bool sleep_status=0;




//#define BOARD_WROVER_KIT 1

// WROVER-KIT PIN Map
#ifdef BOARD_WROVER_KIT

#define CAM_PIN_PWDN -1  //power down is not used
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 21
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 19
#define CAM_PIN_D2 18
#define CAM_PIN_D1 5
#define CAM_PIN_D0 4
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#endif

// ESP32Cam (AiThinker) PIN Map
#ifdef BOARD_ESP32CAM_AITHINKER

#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#endif

static const char *TAG = "example:take_picture";
// testing code before updating the strip controll script
/*void Task1code(void * pvParameters){ 
    delay(2000);
    while(1){
}
}*/

// LED strip controll code
/*void Task1code(void * pvParameters){
    delay(2000);
    //#if not defined IR_USE
    Serial.print("Task1 running on core ");
    Serial.println(xPortGetCoreID());
    //#endif
    uint16_t offset = 0;
    uint32_t last_t0;
    // led ambilight
    while(1){
        if((NEO_TYPE&0b00000011) ==1){
            // **********  the strip led data
            last_t0 = millis();
            //##########################################################################
            // grab right line of led and avg       
            for (int i = 0; i<STRIP_HEIGHT; i++){


                pixel_temp={0,0,0,0};

                //** along strip direction 2 samples
                for (int j =0; j<2; j++){
                    //** toward center 4 samples
                    for (int k=0;k<4;k++){
                        #if defined IR_USE
                        //pixel_temp.pix_TEMP = (NEO_FLAG)? Buffer[right_buf[i]-j*IMG_WIDTH+k*9] : 0 ;
                        pixel_temp.pix_TEMP = (NEO_FLAG)? Buffer[right_buf[i]-right_off[i]*k] : 0 ;
                        if (NEO_FLAG && i <= 20){
                            //** try to draw on TFT but failed, not yet clear on why failed
                            //tft.drawPixel((right_buf[i]-j*IMG_WIDTH+k*3)%TFT_WIDTH, (right_buf[i]-j*IMG_WIDTH+k*3)/TFT_WIDTH, TFT_DARKCYAN);
                        }
                        #else
                        pixel_temp.pix_TEMP = Buffer[right_buf[i]-j*IMG_WIDTH+k*3];
                        if (!NEO_FLAG && i == 1){
                            tft.drawPixel((right_buf[i]-j*IMG_WIDTH+k*3)%TFT_WIDTH, (right_buf[i]-j*IMG_WIDTH+k*3)/TFT_WIDTH, TFT_DARKCYAN);
                        }
                        #endif
                
                        if (k==2 || k==3){
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 10);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 5);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 1);
                        }
                        else if (k==1){
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 9);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 4);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 2);
                        }
                        else{
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 8);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 3);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 3);
                        }

                    }

                }

                //pixel_temp.R_TEMP = dim8_video((pixel_temp.R_TEMP>>2));
                //pixel_temp.G_TEMP = dim8_video((pixel_temp.G_TEMP>>2));
                //pixel_temp.B_TEMP = dim8_video((pixel_temp.B_TEMP>>2));

                LED_GAMMA_TEMP.r = (pixel_temp.R_TEMP>>2);
                LED_GAMMA_TEMP.g = (pixel_temp.G_TEMP>>2);
                LED_GAMMA_TEMP.b = (pixel_temp.B_TEMP>>2);

                LED_GAMMA_TEMP = applyGamma_video(LED_GAMMA_TEMP, 2, 2, 2);
                
                //** start from right i+0
                offset = i;
                leds[offset].setRGB( 
                    (LED_GAMMA_TEMP.r>>1) + (leds[offset].r>>1), 
                    (LED_GAMMA_TEMP.g>>1) + (leds[offset].g>>1), 
                    (LED_GAMMA_TEMP.b>>1) + (leds[offset].b>>1)
                    );
                //** set min backlight
                leds[offset] |= min_bright;
            }

            //##########################################################################
            //** grab top line of led and avg
            for (int i =0; i<STRIP_WIDTH;i++){
                pixel_temp={0,0,0,0};
                //** along strip direction 2 samples
                for (int j = 0; j<2; j++){
                    //** toward center 4 samples
                    for (int k=0; k<4 ; k++){
                        #if defined IR_USE
                        //pixel_temp.pix_TEMP = (NEO_FLAG)? Buffer[top_buf[i]+j-k*IMG_WIDTH*2] : 0 ; 
                        pixel_temp.pix_TEMP = (NEO_FLAG)? Buffer[top_buf[i]-top_off[i]*k] : 0 ; 
                        #else
                        pixel_temp.pix_TEMP = Buffer[top_buf[i]+j-k*IMG_WIDTH*2];   
                        #endif

                        //avg the colors near by
                        if (k==2 || k==3){
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 10);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 5);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 1);
                        }
                        else if (k==1){
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 9);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 4);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 2);
                        }
                        else{
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 8);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 3);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 3);
                        }
                    }
                    
                }

                //pixel_temp.R_TEMP = dim8_video((pixel_temp.R_TEMP>>2)+0);
                //pixel_temp.G_TEMP = dim8_video((pixel_temp.G_TEMP>>2)+0);
                //pixel_temp.B_TEMP = dim8_video((pixel_temp.B_TEMP>>2)-0);
                LED_GAMMA_TEMP.r = (pixel_temp.R_TEMP>>2);
                LED_GAMMA_TEMP.g = (pixel_temp.G_TEMP>>2);
                LED_GAMMA_TEMP.b = (pixel_temp.B_TEMP>>2);

                LED_GAMMA_TEMP = applyGamma_video(LED_GAMMA_TEMP, 2, 2, 2);

                //** top led offset = right +i
                offset = i+45;
                leds[offset].setRGB( 
                    (LED_GAMMA_TEMP.r>>1) + (leds[offset].r>>1),
                    (LED_GAMMA_TEMP.g>>1) + (leds[offset].g>>1), 
                    (LED_GAMMA_TEMP.b>>1) + (leds[offset].b>>1)
                    );
                //** set min backlight
                leds[offset] |= min_bright;
            }

            //##########################################################################
            // grab left line of led and avg
            for (int i = 0; i<STRIP_HEIGHT; i++){
                pixel_temp={0,0,0,0};
                // along strip direction 2 samples
                for (int j =0; j<2; j++){ 
                    // toward center 4 samples
                    for (int k=0;k<4;k++){
                        #if defined IR_USE
                        //pixel_temp.pix_TEMP = (NEO_FLAG)? Buffer[left_buf[i]-j*IMG_WIDTH-k*3] : 0 ;
                        pixel_temp.pix_TEMP = (NEO_FLAG)? Buffer[left_buf[i]-left_off[i]*k] : 0 ;
                        #else
                        pixel_temp.pix_TEMP = Buffer[left_buf[i]-j*IMG_WIDTH-k*2];
                        #endif

                        if (k==2 || k==3){
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 10);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 5);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 1);
                        }
                        else if (k==1){
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 9);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 4);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 2);
                        }
                        else{
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 8);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 3);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 3);
                        }
                    }

                }

                //pixel_temp.R_TEMP = dim8_video((pixel_temp.R_TEMP>>2)+0);
                //pixel_temp.G_TEMP = dim8_video((pixel_temp.G_TEMP>>2)+0);
                //pixel_temp.B_TEMP = dim8_video((pixel_temp.B_TEMP>>2)-0);
                LED_GAMMA_TEMP.r = (pixel_temp.R_TEMP>>2);
                LED_GAMMA_TEMP.g = (pixel_temp.G_TEMP>>2);
                LED_GAMMA_TEMP.b = (pixel_temp.B_TEMP>>2);

                LED_GAMMA_TEMP = applyGamma_video(LED_GAMMA_TEMP, 2, 2, 2);
                //** offset = right + top + i
                offset = i+129;
                leds[offset].setRGB( 
                    (LED_GAMMA_TEMP.r>>1) + (leds[offset].r>>1),
                    (LED_GAMMA_TEMP.g>>1) + (leds[offset].g>>1), 
                    (LED_GAMMA_TEMP.b>>1) + (leds[offset].b>>1)
                    );
                //** set min color
                leds[offset] |= min_bright;
            }

            //##########################################################################
            // grab bottom line of led and avg
            for (int i =0; i<STRIP_WIDTH;i++){
                pixel_temp={0,0,0,0};
                for (int j = 0; j<2; j++){
                    for (int k=0;k<4;k++){
                        //delayMicroseconds(10);
                        #if defined IR_USE
                        //pixel_temp.pix_TEMP = (NEO_FLAG)? Buffer[bottom_buf[i]+j+k*IMG_WIDTH*2] : 0 ;
                        pixel_temp.pix_TEMP = (NEO_FLAG)? Buffer[bottom_buf[i]+bottom_off[i]*k] : 0 ;
                        #else
                        pixel_temp.pix_TEMP = Buffer[bottom_buf[i]+j+k*IMG_WIDTH*2];
                        #endif
                        if (k==2 || k==3){
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 10);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 5);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 1);
                        }
                        else if (k==1){
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 9);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 4);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 2);
                        }
                        else{
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 8);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 3);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 3);
                        }
                    }
                    
                }

                //pixel_temp.R_TEMP = dim8_video((pixel_temp.R_TEMP>>2)+0);
                //pixel_temp.G_TEMP = dim8_video((pixel_temp.G_TEMP>>2)+0);
                //pixel_temp.B_TEMP = dim8_video((pixel_temp.B_TEMP>>2)-0);
                LED_GAMMA_TEMP.r = (pixel_temp.R_TEMP>>2);
                LED_GAMMA_TEMP.g = (pixel_temp.G_TEMP>>2);
                LED_GAMMA_TEMP.b = (pixel_temp.B_TEMP>>2);

                LED_GAMMA_TEMP = applyGamma_video(LED_GAMMA_TEMP,2, 2, 2);
                //** offset = right + top  + left +i
                offset = i+174;            
                leds[offset].setRGB( 
                    (LED_GAMMA_TEMP.r>>1) + (leds[offset].r>>1),
                    (LED_GAMMA_TEMP.g>>1) + (leds[offset].g>>1), 
                    (LED_GAMMA_TEMP.b>>1) + (leds[offset].b>>1)
                );
                //** set min color
                leds[offset] |= min_bright;
            }

            FastLED.show();
            Serial.println(String(millis()-LED_T0)+" ms processed, led process loop");
            LED_T0 = millis() - last_t0;
            //tft.drawString(String(millis()- LED_T0) + " ms processed, led process loop",10,160);
        }
        else if((NEO_TYPE&0b00000011) == 2 && Process){
            for (uint i =0; i < NUM_LEDS; i++){
                leds[i] = min_bright;
            }
            delay(500);
            Process = !Process;
            Serial.println("NEO type 2, led = (30,20,16)");

            FastLED.show();
        }
        else if((NEO_TYPE&0b00000011) == 3 && Process){
            for (uint i =0; i < NUM_LEDS; i++){
                leds[i] = (38,25,25);
            }
            delay(500);
            Process = !Process;
            Serial.println("NEO type 3, led = 38, 25, 25");

            FastLED.show();
        }
        else if((NEO_TYPE&0b00000111) == 4 && Process){
            for (uint i =0; i < NUM_LEDS; i++){
                leds[i] = (255,255,255);
            }
            delay(500);
            Process = !Process;
            Serial.println("NEO type 4, led = 25, 20, 30");

            FastLED.show();
        }
        else if(Process){
            for (uint i =0; i < NUM_LEDS; i++){
                leds[i] = (0,0,0);
            }
            delay(500);
            Process = !Process;
            Serial.println("NEO type NA, led = 0,0,0");

            FastLED.show();
        }
    }
}*/

void Task2code(void * pvParameters){
    camera_fb_t *pic = esp_camera_fb_get();
    while(1){
        if (CAPTURE){
            lastimage = millis();
            ESP_LOGI(TAG, "Taking picture...");
            if (pic){
                esp_camera_fb_return(pic);
            }
            pic = esp_camera_fb_get();
            ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes", pic->len);
            
            // use pic->buf to access the image, store image buffer for LED strip process and capture without 
            // not know if can process read/write with same memory by taking picture and LED  strip process
            #ifdef QQVGA
                for (uint i = 0; i<38400; i+=2){
                    Buffer[i/2] = pic->buf[i+1] | pic->buf[i]<<8;
                    
                }
            #elif defined(QVGA)
                for (uint i = 0; i<76800; i+=2){
                    Buffer[i/2] = pic->buf[i+1] | pic->buf[i]<<8;
                }
            #endif
            temp_time = millis()- lastimage;
            //esp_camera_fb_return(pic);
        }
    }
}

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,
    

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 10000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    
    //.ledc_timer = LEDC_TIMER_1,
    //.ledc_channel = LEDC_CHANNEL_1,

    .pixel_format = PIXFORMAT_RGB565, //YUV422,GRAYSCALE,RGB565,JPEG
    
    #ifdef QQVGA
        .frame_size = FRAMESIZE_QQVGA,    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.
    #elif defined(QVGA)
        .frame_size = FRAMESIZE_QVGA,
    #endif
    
    .jpeg_quality = 22, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 2,       //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    
};

static esp_err_t init_camera(){
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK){
        Serial.println("init fail");
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    };
    return ESP_OK;
}
void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }
    // begin camera
    if(ESP_OK != init_camera()) {
        //return;
        while (1){delay(1000);}
    }
    sensor_t * s = esp_camera_sensor_get();
    
    /*
    s->set_wb_mode(s, 1);
    s->set_saturation(s, 1);     // -2 to 2
    s->set_contrast(s, 1);       // -2 to 2
    s->set_brightness(s, -1);   // -2 to 2
    s->set_gainceiling(s, GAINCEILING_4X);
    s->set_aec_value(0, 500);
    s->set_ae_level(s, -1);
    */
    // wake up from ext0, After waking up from sleep, the IO pad used for wakeup will be configured as RTC IO. 
    // Therefore, before using this pad as digital GPIO, users need to reconfigure it using rtc_gpio_deinit() function.
    // change the state of sleep, make ESP32 know it wake up from sleep mode,
    if (sleep_status){
        rtc_gpio_deinit(GPIO_NUM_4);
        Serial.print("reboot from sleep mode");
        sleep_status = 0; // reset to 0 state for clear sleep state
    }
    
    


    // begin IR receiver
    #if defined IR_USE
    irrecv.enableIRIn();  // Start the receiver
    Serial.print("IRrecvDemo is now running and waiting for IR message on Pin ");
    Serial.println(kRecvPin);
    #endif

    // begin setup the TFT
    tft.begin();
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.fillScreen(TFT_BLACK);
    tft.setRotation(1);
    tft.setSwapBytes(true);
    
    
    pinMode(TFT_BKL, OUTPUT);
    digitalWrite(TFT_BKL, HIGH); // backlight led on
    //**********************begin setup of led strip
 
    // It's important to set the color correction for your LED strip here,
    // so that colors can be more accurately rendered through the 'temperature' profiles 
    //TypicalSMD5050  =0xFFB0F0 /* 255, 176, 240 */,
    //TypicalLEDStrip =0xFFB0F0 /* 255, 176, 240 */,
    //TypicalPixelString=0xFFE08C /* 255, 224, 140 */,
    FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness( BRIGHTNESS );
    FastLED.setMaxPowerInMilliWatts(12*5000);
    
    //********************** core 1 operation for LED
    /*xTaskCreatePinnedToCore(
        Task1code,  //* Function to implement the task
        "Task1",    // Name of the task
        10000,      //* stack size in words
        NULL,       //* Task input parameter
        0,      //* priority of the task
        &Task1, //* Task handle.
        1
    );*/
    //** camera operation on core 0 
    xTaskCreatePinnedToCore(
        Task2code,  //* Function to implement the task
        "Task2",    // Name of the task
        10000,      //* stack size in words
        NULL,       //* Task input parameter
        1,          //* priority of the task
        &Task2,     //* Task handle.
        0
    );
    
    ledcSetup(0, 10, 8);
    //ledcAttachPin(LED_BUILTIN, 0);
    ledcWrite(0, 245);
}


void loop()
{

    decode_results results;
    //uint32_t lastimage = millis();
    uint32_t calt_time=millis();
    uint16_t offset = 0;
    uint32_t last_t0;
    //camera_fb_t *pic = esp_camera_fb_get();
    float_t capture_fps;
    String print_on_lcd="  ";
    while (1)
    {

        // led ambilight
    
        if((NEO_TYPE&0b00000011) ==1 && Process){
            // **********  the strip led data
            last_t0 = millis();
            //##########################################################################
            // grab right line of led and avg       
            for (int i = 0; i<STRIP_HEIGHT; i++){


                pixel_temp={0,0,0,0};

                //** along strip direction 2 samples
                for (int j =0; j<2; j++){
                    //** toward center 4 samples
                    for (int k=0;k<4;k++){
                        #if defined IR_USE
                        //pixel_temp.pix_TEMP = (NEO_FLAG)? Buffer[right_buf[i]-j*IMG_WIDTH+k*9] : 0 ;
                        pixel_temp.pix_TEMP = (NEO_FLAG)? Buffer[right_buf[i]-right_off[i]*k] : 0 ;
                        if (NEO_FLAG && i <= 20){
                            //** try to draw on TFT but failed, not yet clear on why failed
                            //tft.drawPixel((right_buf[i]-j*IMG_WIDTH+k*3)%TFT_WIDTH, (right_buf[i]-j*IMG_WIDTH+k*3)/TFT_WIDTH, TFT_DARKCYAN);
                        }
                        #else
                        pixel_temp.pix_TEMP = Buffer[right_buf[i]-j*IMG_WIDTH+k*3];
                        if (!NEO_FLAG && i == 1){
                            tft.drawPixel((right_buf[i]-j*IMG_WIDTH+k*3)%TFT_WIDTH, (right_buf[i]-j*IMG_WIDTH+k*3)/TFT_WIDTH, TFT_DARKCYAN);
                        }
                        #endif
                
                        if (k==2 || k==3){
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 10);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 5);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 1);
                        }
                        else if (k==1){
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 9);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 4);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 2);
                        }
                        else{
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 8);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 3);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 3);
                        }

                    }

                }

                //pixel_temp.R_TEMP = dim8_video((pixel_temp.R_TEMP>>2));
                //pixel_temp.G_TEMP = dim8_video((pixel_temp.G_TEMP>>2));
                //pixel_temp.B_TEMP = dim8_video((pixel_temp.B_TEMP>>2));

                LED_GAMMA_TEMP.r = (pixel_temp.R_TEMP>>2);
                LED_GAMMA_TEMP.g = (pixel_temp.G_TEMP>>2);
                LED_GAMMA_TEMP.b = (pixel_temp.B_TEMP>>2);

                LED_GAMMA_TEMP = applyGamma_video(LED_GAMMA_TEMP, 2, 2, 2);
                
                //** start from right i+0
                offset = i;
                leds[offset].setRGB( 
                    (LED_GAMMA_TEMP.r>>1) + (leds[offset].r>>1), 
                    (LED_GAMMA_TEMP.g>>1) + (leds[offset].g>>1), 
                    (LED_GAMMA_TEMP.b>>1) + (leds[offset].b>>1)
                    );
                //** set min backlight
                leds[offset] |= min_bright;
            }

            //##########################################################################
            //** grab top line of led and avg
            for (int i =0; i<STRIP_WIDTH;i++){
                pixel_temp={0,0,0,0};
                //** along strip direction 2 samples
                for (int j = 0; j<2; j++){
                    //** toward center 4 samples
                    for (int k=0; k<4 ; k++){
                        #if defined IR_USE
                        //pixel_temp.pix_TEMP = (NEO_FLAG)? Buffer[top_buf[i]+j-k*IMG_WIDTH*2] : 0 ; 
                        pixel_temp.pix_TEMP = (NEO_FLAG)? Buffer[top_buf[i]-top_off[i]*k] : 0 ; 
                        #else
                        pixel_temp.pix_TEMP = Buffer[top_buf[i]+j-k*IMG_WIDTH*2];   
                        #endif

                        //avg the colors near by
                        if (k==2 || k==3){
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 10);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 5);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 1);
                        }
                        else if (k==1){
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 9);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 4);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 2);
                        }
                        else{
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 8);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 3);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 3);
                        }
                    }
                    
                }

                //pixel_temp.R_TEMP = dim8_video((pixel_temp.R_TEMP>>2)+0);
                //pixel_temp.G_TEMP = dim8_video((pixel_temp.G_TEMP>>2)+0);
                //pixel_temp.B_TEMP = dim8_video((pixel_temp.B_TEMP>>2)-0);
                LED_GAMMA_TEMP.r = (pixel_temp.R_TEMP>>2);
                LED_GAMMA_TEMP.g = (pixel_temp.G_TEMP>>2);
                LED_GAMMA_TEMP.b = (pixel_temp.B_TEMP>>2);

                LED_GAMMA_TEMP = applyGamma_video(LED_GAMMA_TEMP, 2, 2, 2);

                //** top led offset = right +i
                offset = i+45;
                leds[offset].setRGB( 
                    (LED_GAMMA_TEMP.r>>1) + (leds[offset].r>>1),
                    (LED_GAMMA_TEMP.g>>1) + (leds[offset].g>>1), 
                    (LED_GAMMA_TEMP.b>>1) + (leds[offset].b>>1)
                    );
                //** set min backlight
                leds[offset] |= min_bright;
            }

            //##########################################################################
            // grab left line of led and avg
            for (int i = 0; i<STRIP_HEIGHT; i++){
                pixel_temp={0,0,0,0};
                // along strip direction 2 samples
                for (int j =0; j<2; j++){ 
                    // toward center 4 samples
                    for (int k=0;k<4;k++){
                        #if defined IR_USE
                        //pixel_temp.pix_TEMP = (NEO_FLAG)? Buffer[left_buf[i]-j*IMG_WIDTH-k*3] : 0 ;
                        pixel_temp.pix_TEMP = (NEO_FLAG)? Buffer[left_buf[i]-left_off[i]*k] : 0 ;
                        #else
                        pixel_temp.pix_TEMP = Buffer[left_buf[i]-j*IMG_WIDTH-k*2];
                        #endif

                        if (k==2 || k==3){
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 10);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 5);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 1);
                        }
                        else if (k==1){
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 9);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 4);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 2);
                        }
                        else{
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 8);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 3);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 3);
                        }
                    }

                }

                //pixel_temp.R_TEMP = dim8_video((pixel_temp.R_TEMP>>2)+0);
                //pixel_temp.G_TEMP = dim8_video((pixel_temp.G_TEMP>>2)+0);
                //pixel_temp.B_TEMP = dim8_video((pixel_temp.B_TEMP>>2)-0);
                LED_GAMMA_TEMP.r = (pixel_temp.R_TEMP>>2);
                LED_GAMMA_TEMP.g = (pixel_temp.G_TEMP>>2);
                LED_GAMMA_TEMP.b = (pixel_temp.B_TEMP>>2);

                LED_GAMMA_TEMP = applyGamma_video(LED_GAMMA_TEMP, 2, 2, 2);
                //** offset = right + top + i
                offset = i+129;
                leds[offset].setRGB( 
                    (LED_GAMMA_TEMP.r>>1) + (leds[offset].r>>1),
                    (LED_GAMMA_TEMP.g>>1) + (leds[offset].g>>1), 
                    (LED_GAMMA_TEMP.b>>1) + (leds[offset].b>>1)
                    );
                //** set min color
                leds[offset] |= min_bright;
            }

            //##########################################################################
            // grab bottom line of led and avg
            for (int i =0; i<STRIP_WIDTH;i++){
                pixel_temp={0,0,0,0};
                for (int j = 0; j<2; j++){
                    for (int k=0;k<4;k++){
                        //delayMicroseconds(10);
                        #if defined IR_USE
                        //pixel_temp.pix_TEMP = (NEO_FLAG)? Buffer[bottom_buf[i]+j+k*IMG_WIDTH*2] : 0 ;
                        pixel_temp.pix_TEMP = (NEO_FLAG)? Buffer[bottom_buf[i]+bottom_off[i]*k] : 0 ;
                        #else
                        pixel_temp.pix_TEMP = Buffer[bottom_buf[i]+j+k*IMG_WIDTH*2];
                        #endif
                        if (k==2 || k==3){
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 10);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 5);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 1);
                        }
                        else if (k==1){
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 9);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 4);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 2);
                        }
                        else{
                            pixel_temp.R_TEMP += ((pixel_temp.pix_TEMP & 0b1111100000000000) >> 8);
                            pixel_temp.G_TEMP += ((pixel_temp.pix_TEMP & 0b0000011111100000) >> 3);
                            pixel_temp.B_TEMP += ((pixel_temp.pix_TEMP & 0b0000000000011111) << 3);
                        }
                    }
                    
                }

                //pixel_temp.R_TEMP = dim8_video((pixel_temp.R_TEMP>>2)+0);
                //pixel_temp.G_TEMP = dim8_video((pixel_temp.G_TEMP>>2)+0);
                //pixel_temp.B_TEMP = dim8_video((pixel_temp.B_TEMP>>2)-0);
                LED_GAMMA_TEMP.r = (pixel_temp.R_TEMP>>2);
                LED_GAMMA_TEMP.g = (pixel_temp.G_TEMP>>2);
                LED_GAMMA_TEMP.b = (pixel_temp.B_TEMP>>2);

                LED_GAMMA_TEMP = applyGamma_video(LED_GAMMA_TEMP,2, 2, 2);
                //** offset = right + top  + left +i
                offset = i+174;            
                leds[offset].setRGB( 
                    (LED_GAMMA_TEMP.r>>1) + (leds[offset].r>>1),
                    (LED_GAMMA_TEMP.g>>1) + (leds[offset].g>>1), 
                    (LED_GAMMA_TEMP.b>>1) + (leds[offset].b>>1)
                );
                //** set min color
                leds[offset] |= min_bright;
            }

            FastLED.show();
            //Serial.println(String(millis()-last_t0)+" ms processed, led process loop");
            LED_T0 = millis() - last_t0;
            //tft.drawString(String(millis()- LED_T0) + " ms processed, led process loop",10,160);
        }
        else if((NEO_TYPE&0b00000011) == 2 && Process){
            for (uint i =0; i < NUM_LEDS; i++){
                leds[i] = min_bright;
            }
            delay(500);
            Process = !Process;
            Serial.println("NEO type 2, led = (30,20,16)");

            FastLED.show();
        }
        else if((NEO_TYPE&0b00000011) == 3 && Process){
            for (uint i =0; i < NUM_LEDS; i++){
                leds[i] = (38,28,20);
            }
            delay(500);
            Process = !Process;
            Serial.println("NEO type 3, led = 38, 25, 25");

            FastLED.show();
        }
        else if((NEO_TYPE&0b00000111) == 4 && Process){
            for (uint i =0; i < NUM_LEDS; i++){
                leds[i] = TypicalLEDStrip;//(255,255,255);
            }
            delay(500);
            Process = !Process;
            Serial.println("NEO type 4, led = 25, 20, 30");

            FastLED.show();
        }
        else if(Process){
            for (uint i =0; i < NUM_LEDS; i++){
                leds[i] = (0,0,0);
            }
            delay(500);
            Process = !Process;
            Serial.println("NEO type NA, led = 0,0,0");

            FastLED.show();
        }
    
        //lastimage = millis();
        /*if (CAPTURE){
            ESP_LOGI(TAG, "Taking picture...");
            if (pic){
                esp_camera_fb_return(pic);
            }
            pic = esp_camera_fb_get();
            ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes", pic->len);
            
            // use pic->buf to access the image, store image buffer for LED strip process and capture without 
            // not know if can process read/write with same memory by taking picture and LED  strip process
            #ifdef QQVGA
                for (uint i = 0; i<38400; i+=2){
                    Buffer[i/2] = pic->buf[i+1] | pic->buf[i]<<8;
                    
                }
            #elif defined(QVGA)
                for (uint i = 0; i<76800; i+=2){
                    Buffer[i/2] = pic->buf[i+1] | pic->buf[i]<<8;
                }
            #endif

            //esp_camera_fb_return(pic);
        }*/
       
        if(!NEO_FLAG){
            // cast image to TFT
            #ifdef QQVGA
                tft.pushImage(0,0,160, 120, Buffer);
            #elif defined(QVGA)
                tft.pushImage(0,0,320, 120, Buffer);
            #endif

            //Serial.println(Buffer[39000], BIN);
            for (uint i=0;i<NUM_LEDS-5;i++){
                uint offset_tft;
                if (i<STRIP_HEIGHT){
                    offset_tft = i;
                    tft.drawPixel(right_buf[offset_tft]%320, right_buf[offset_tft]/320,TFT_MAGENTA);
                }
                else if (i<STRIP_HEIGHT+STRIP_WIDTH){
                    offset_tft = i-(STRIP_HEIGHT);
                    tft.drawPixel(top_buf[offset_tft]%320, top_buf[offset_tft]/320,TFT_YELLOW);
                }
                else if (i<STRIP_HEIGHT+STRIP_WIDTH+STRIP_HEIGHT){
                    //left
                    offset_tft = i - (STRIP_HEIGHT+STRIP_WIDTH);
                    tft.drawPixel(left_buf[offset_tft]%320, left_buf[offset_tft]/320,TFT_CYAN);
                }
                else{
                    //bottom
                    offset_tft = i - (STRIP_HEIGHT+STRIP_WIDTH+STRIP_HEIGHT);
                    tft.drawPixel(bottom_buf[offset_tft]%320, bottom_buf[offset_tft]/320,TFT_YELLOW);
                }
            }

   
            
        }
        else{
           // tft.fillScreen(TFT_BLACK);
        }


        //*** IR decode and process if matches
        #if defined IR_USE
        
        if (irrecv.decode(&results)) {
            
            // print() & println() can't handle printing long longs. (uint64_t)
            //serialPrintUint64(results.value, HEX);
            //Serial.println("");
            // off buttom 
            if (results.value == 0x1CF20CE4 ){
                transmitter.sendUnit(0, false); //* turn off lamp
                transmitter.sendUnit(2, true); //* trun on tv desk strip
                // off power of LCD
                digitalWrite(TFT_BKL, LOW);
                NEO_TYPE ++;
                NEO_FLAG = 1;
                CAPTURE = 1;
                Process = true;
                Serial.printf("Neotype %d\n", NEO_TYPE);
                Serial.println(F("off lamp, on tv set"));
            }

            else if (results.value == 0x247BF3DA ){
                //transmitter.sendUnit(0, false); //* turn off lamp
                //transmitter.sendUnit(2, false); //* trun on tv desk strip
                
                transmitter.sendGroup(false);
                NEO_TYPE =0;
                
                Serial.println(F("NEOtype0, OFF 433Mhz devices; OFF LCD backligh"));
                
                // try to off strip power
                NEO_FLAG = 0;

                // off power of LCD
                digitalWrite(TFT_BKL, HIGH);
                tft.fillScreen(TFT_BLACK);

                // slow down esp32?
                CAPTURE = 0;

                // change the state of sleep, make ESP32 know it wake up from sleep mode,
                sleep_status=1;
                /*
                First we configure the wake up source
                We set our ESP32 to wake up for an external trigger.
                There are two types for ESP32, ext0 and ext1 .
                ext0 uses RTC_IO to wakeup thus requires RTC peripherals
                to be on while ext1 uses RTC Controller so doesnt need
                peripherals to be powered on.
                Note that using internal pullups/pulldowns also requires
                RTC peripherals to be turned on.
                */
            
                esp_sleep_enable_ext0_wakeup(GPIO_NUM_4,0); //GPIO_NUM_4 , 1 = High, 0 = Low
                Serial.println("Going to sleep now");
                delay(1000);
                esp_deep_sleep_start();
                
            }

            else if (results.value == 0xA594C007 || results.value== 0x678A85BC){
                transmitter.sendUnit(0, false); //* turn off lamp
                transmitter.sendUnit(2, true); //* trun on tv desk strip
                NEO_TYPE = 0;
                CAPTURE = 1;
                NEO_FLAG = 0;
                Serial.println(F("off all"));
            }
            
            irrecv.resume();  // Receive the next value
        }
        //delay(1000);
        #endif
        
        if (millis()-calt_time >200){
            capture_fps = (temp_time)? 1000.0/temp_time: 0.0;
            print_on_lcd = "core1 FPS: "+String(capture_fps,2);
            Serial.println(String(temp_time) + " ms processed, 1 loop");
            tft.drawString(String(temp_time) + " ms processed, 1 loop",10,180);  
            tft.drawString("NEO enable: " + String(NEO_FLAG)+", NEO type: "+String(NEO_TYPE), 10, 140);  
            tft.drawString("LED strip time:    ", 10, 160);
            tft.drawString(String(LED_T0), 106, 160);
            tft.drawString(print_on_lcd,10,120);
            calt_time=millis();        
        }
        //vTaskDelay(1000 / portTICK_RATE_MS);

    }
}