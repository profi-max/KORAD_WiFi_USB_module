# KORAD_WiFi_USB_module
 Turn your KA3005D into programmable KA3005P.
 The project  is based on the ESP8266 (ESP-12) module for communicate over Wifi or USB with KORAD 3005 power supply.

## How it works
Although KORAD3005D is not intended for remote control, nevertheless, some of the devices can work like KORAD3005P. Under the cover on the printed circuit board there is a connector for communication via UART. Let's call such devices that can be controlled via UART KORAD3005DP.
To see the difference between KORAD3005D and KORAD3005DP, just listen to it. Watch this short video and you will hear the difference.
https://youtu.be/TDWt2rKNsIM
[![Watch the video](https://img.youtube.com/vi/TDWt2rKNsIM/hqdefault.jpg)](https://www.youtube.com/embed/TDWt2rKNsIM)

[<img src="https://img.youtube.com/vi/TDWt2rKNsIM/hqdefault.jpg" width="600" height="300"
/>](https://www.youtube.com/embed/TDWt2rKNsIM)

If you are lucky and have a KORAD3005DP, then this project is for you. If not, check out my next project, it will be even more interesting.

## Equipment
You can use any ESP8266 (ESP-12) development board with USB socket. Also you need an optional USB connector and a button.
<details>
<summary>I use this one: Click to view the  board</summary>
Aliexpress page: https://aliexpress.ru/item/4000550036826.html
<image src="/Pictures/NodeMCU.jpg" alt="NodeMCU">
</details>

## How to flash ESP8266 (3 ways)
1. Use finished firmware files from the folder **Binary**. Upload Flash Download Tools  from official page: https://www.espressif.com/en/support/download/other-tools
Connect USB cable to your development board, run the application, select COM port and bin-file.
2. Use Arduino IDE. Open the sketch from **Arduino** folder. Select NodeMCU 1.0 board. Install WiFiManager by tzapu library. Compile and upload the sketch to your dev board.
3. Use Visual Studio Code + Platformio. Open project from **VSCode** folder. Build and upload the project.
<details>
<summary>Click to view Flash Download Tools</summary>
<image src="/Pictures/flasher.jpg" alt="Flasher">
</details>

## DPSmaster - Windows PC application for managing KORAD3005
1. WiFi: In the DPSmaster application select "TCP" and enter correct ESP8266's local IP address.
2. USB:  In the DPSmaster application select correct COM-port. Baud rate 115200, slave address 1.
DPSmaster officail page: https://profimaxblog.ru/dpsmaster/
![DPSmaster](/Pictures/DPSmaster.jpg)

## Schematics
Avoid simultaneous 5 volt power supply from different devices. Power up the development board either from USB or from KORAD. 
In the final device, remove diode D1 so that the dev board is powered only from KORAD.
In the NodeMcu dev board "FLASH" button connected to GPIO0, so you can use "FLASH" button as "WiFi Reset".
![Korad_NodeMcu](/Pictures/korad_NodeMcu.jpg)
<details>
<summary>Click to view NodeMcu schematic</summary>
<image src="/Pictures/NodeMCU_schematic.jpg" alt="NodeMCU">
</details>

## Video & Photo Gallery
<details>
<summary>Click to view the Gallery</summary>
https://youtube.com/shorts/995Rk9Xic3o

https://youtube.com/shorts/P--5Z4uEhjc

![DPSview](/Pictures/IMG_4732.JPG)
![DPSview](/Pictures/IMG_4735.JPG)
![DPSview](/Pictures/IMG_4733.JPG)
![DPSview](/Pictures/IMG_4736.JPG)
</details>

## How to establish a connection with Wi-Fi Router.
Push "WiFi Reset" button for 3 seconds. The ESP8266 module switch to access point mode. Open WiFi settings on your phone. Choose "KORAD bridge", no password needed. Then click "Configure WiFi", choose your router, enter password and click "Save". Now the ESP8266 module is in the station mode and connected to your wifi router.
<image src="/Pictures/WifiManager.jpg" alt="WiFiManager">

## How to find out the local Wi-Fi IP address of ESP8266.
Open the router's web page in a browser. View the list of clients.
For DPS alternative firmware: Go to "Parameters" menu. Press the "SET" button. The IP address will be displayed for 3 seconds.

## How to debug
The hardware serial over UART1 is used at 115200 speed. GPIO02 is UART1 TX pin.
```
// uncomment the line below if you need debug via UART1
#define MB_DEBUG
//#define KORAD_DEBUG_TX
//#define KORAD_DEBUG_RX

```
## KORAD Internal UART protocol
https://sigrok.org/wiki/Atten_PPS3000_Series

##  DPSmaster  Modbus protocol
https://profimaxblog.ru/modbus-communication-protocol-2/

## Known bugs
Unfortunately when output is on and at the moment of changing Uset or Iset,  the KORAD response is Iout = 0. 
I don't know what to do about it.