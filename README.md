

## this is only personal project, not really an expert in ESP32 and coding, the code is really messy and may have only little support from me.

# ESP32-cam with OV7725, ili9341 320x240 TFT and WS2815 for ambiligh

in order to make the ambiligt with non app installed, currently Arduino base, no considering low power, as mentioned by @pierre-muth/OV7725_ESP32cam The OV7725 is a more sensitive sensor compare to the one usually found with the ESP32cam board (often the OV2640)  
well, in my case I don't need to covnet RGB565 to jpeg for streaming and no need of wifi web server, that saves alot of computing power to achieve 30FPS for single core doing only taking picture by ov7725.

## Key functions of the projects

- FPS around 30FPS
    - with RGB565, QVGA setting, when there turned off the image push to TFT, only 15fps while push picture to TFT.
    - main loop of taking photo on ESP32 core 1
    - image process and pushing WS2815 data by ESP32 core 0
- receive my IR remote of ceilling light for activate the movie mode
- RF433 remote control for controlling my RF controlled AC socket connecting to IKEA led strip powering
  - the RC-remote arduino library is not fit my AC socket controller, there need a variable light contorl available library, is the @1technophile/NewRemoteSwitch
- AEC, AWB is manual setting for consist color during difference scenes, but the HDR video still has chances got too blueish or less color


currently depend on my TV set, the image location can only be defined by self calculating on excel, my TV set is 65inch, and 15 cm height and 20cm away from the camera.  
I turned off the AEC and AWB for the low light environment of my movie, gaming senario, however there still has color shift in the result, but can modify by the gamma calculation of the esp32 built in image process function  
with my 65inch TV, need 45 LEDs on right/left, and 84 LEDs on top/down, total 248 LEDs by 50 LEDs/meter LED strip, 12V power require ~40W [WLED calculator](https://wled-calculator.github.io/)


## Required hardware

    ESP32-cam clone (using the modified with RST pin, compatible with ESP32-cam MB downloader board)
    ov7725 with M12 ~170degree lens ultrawide camera 
    TFT LCD 320x240 (with ili9341 controlller)
    WS2815 248 LEDs for my 65inch TV (depend on the FastLED support)
    IR receiver module
    RF 433Mhz transmitter module
    12V Power supply in my case 

## Required arduino package

	fastled/FastLED@^3.5.0
	crankyoldgit/IRremoteESP8266@^2.8.5
    NewRemoteSwitch-master
    bodmer/TFT_eSPI@^2.5.23

## Wiring of the project

![image](https://github.com/WASmars/the_project/assets/54877239/a41b12f6-0b1a-4ae0-b5e5-bd952424c0dc)

|ESP32CAM pin| wiring to/from |
|-----------|-------------|
| 5V        |from POWER 5V|
| GND       |from POWER GND|
| GPIO12    |to 433MHZ RF TX DATA |
| GPIO13    |to TFT MOSI|
| GPIO15    |to level shift for WS2815 LED DATA |
| GPIO14    |to TFT SCK|
| GPIO2     |to TFT DC|
| GPIO4     |to IR REMOTE RX INPUT|
| GPIO16    | *PSRAM #CS pin used*|
| GPIO0	    | *CAMERA used*|


- GPIO 0 is for the camera MCLK generation, connected to camera, ***which is not suitable for other application once you use camera***
- GPIO16 is used for CS# of the PSRAM, ***which is not suitable for other application once you use camera***
- GPIO 1, 3 used for UART communication with PC
- original GND near the flash is ***modified as RST pin in the clone ESP32-cam board*** wihcih is useful for pin release
- ****remove the R13(in reference schematic) 1k resistor or flash to remove the light****
![IMG_20230615_220835346_HDR](https://github.com/WASmars/the_project/assets/54877239/cffb8529-0237-45a7-a857-8d0ed28df1b9)



# REFERENCE DOCUMENT OF ESP32 AND RELATED HARDWARE
## ESP32-CAM
notice on the R13 resistor and GND/R pin  
![image](https://github.com/WASmars/the_project/assets/54877239/08f9af69-4bb7-4ae5-954d-5228e4295feb)





### ESP32-cam image and pinout

![image](https://github.com/WASmars/the_project/assets/54877239/f2d47d68-0da5-4785-87eb-bbe4525919ff)
[detail pin info can refer](https://github.com/raphaelbs/esp32-cam-ai-thinker/blob/master/docs/esp32cam-pin-notes.md)

### ESP32-cam schematic

![image](https://github.com/WASmars/the_project/assets/54877239/6bd5947a-d09d-4417-a245-3cfcf2be35cf)  

You can download a PDF file with better resolution on [this GitHub repository](https://github.com/SeeedDocument/forum_doc/blob/master/reg/ESP32_CAM_V1.6.pdf)

### OV7725 ULTRA WIDE CAMERA WITH M12 LENS

![image](https://github.com/WASmars/the_project/assets/54877239/6b4b152f-4d8b-471a-8fad-e8294fd2a499)  
 OV7725攝像頭模組模塊 30萬像素 M12鏡頭適用於STM32 OPENMV K210 [shop link](https://world.taobao.com/item/wap/668570605351.htm?spm=a21wu.23452756-tw.taglist-content.30.6cca2b64RCVOqW#)


### TFT LCD 2.2inch, 320x240 pixels ili9341 driver

![image](https://github.com/WASmars/the_project/assets/54877239/244867a5-7cef-4f7b-bdfd-f4b7b1e8acc2)
| TFT pin| def | wiring to/from|
|-------|--------|-------------|
| PIN1	| VCC    | TO ESP32 3.3V| 
| PIN2	| GND    | TO ESP32 GND| 
| PIN3	| CS    | TO ESP32 GND| 
| PIN4	| RESET  | TO ESP32 RST| 
| PIN5	| DC/RS  | TO ESP32 GPIO2| 
| PIN6	| MOSI   | TO ESP32 GPIO13| 
| PIN7	| SCK    | TO ESP32 GPIO14| 
| PIN8	| BLK    | TO ESP32 3.3V| 
| PIN9	| MISO   |  LEAVE OPEN| 

can download a related documentation on [this site](http://www.lcdwiki.com/2.2inch_SPI_Module_ILI9341_SKU:MSP2202)  

### 433MHZ transmitter module

![image](https://github.com/WASmars/the_project/assets/54877239/94d889eb-c816-4ac8-a0d0-f30657eded81)  
Regular cheap 433Mhz TX module, with antenna connected for 17.3cm long solid wire, don't use the stranded cable it would not help in the range (google it for detail)

### IR receiver module

notice, traditional IR receiver module accompany with arduino may not fit with 3.3Vin, find one fit 3.3V operation  
**notice from the IRremoteESP8266 author**[link](https://github.com/crankyoldgit/IRremoteESP8266/wiki/Frequently-Asked-Questions#user-content-Help_Im_getting_very_inconsistent_results_when_capturing_an_IR_message_using_a_VS1838b_IR_demodulator)  
*Please have a read of [this article](https://www.analysir.com/blog/2014/12/08/infrared-receiver-showdown-tsop34438-vs-vs1838b-winner-revealed/) and Issue#1173. It appears that the VS1838B is very cheap, and you get exactly what you pay for sometimes. In ideal conditions, you'll get good results, add sunlight/heat etc and it apparently offers poor perforce. We have had similar reports from our users. In short, buy a better hardware IR demodulation device, and your results should improve. If you can't afford the few extra cents, try some of the suggestions offered in Issue#1173.* 

### level shift
any type of level shift is okay, typically single direction by transistor is enough no 
