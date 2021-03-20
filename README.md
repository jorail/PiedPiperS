# PiedPiperS
Smartphone HTML control for model trains, using ESP32 microcontroller, WiFi access point, asynchronous webserver, WebXocket and JSON, outdoor compatible with on-board USB power supply 

The program purpose is to control a model train motor with independent power supply e.g. from USB power bank for outdoors. 
Following options exist:
a) Control fron a smartphone: ESP32 Wifi WebServer for commands from a client browser via WebSocket connection:
   https://github.com/jorail/PiedPiperS
b) Control by a whistle: Original PiedPiper whistle Morse code signals. Works with microphone input and FFT tone 
   analysis in a Teensy 4.0 microcontroller (in this version tone sampling is deactivated for ESP32 by commenting out):
   https://github.com/jorail/PiedPiper
c) Switch or Touch-Pin Morse code input
d) SerialMonitor input commands by typing letters, connection via USB

The project is inspired by the project 'free your model train' (FYMT) proposed by Frei Softwarefreunde
at https://freie-software.org/free-your-model-train/

The code and layout are developed for ESP32 (or Teensy 4.0) but can easily be modified for other micro controllers.
Asynchronous WebServer libraries are used in combination with WebSocket connection and JSON messages.
Sound evaluation and tone signal identification is performed by FFT analysis in the microprocessor.
Identified client commands or Morse code commands from a switch, touch senosor or tone signals are processed to 
commands for change of the speed level. New motor settings are transfered via pulse width modulation (PWM) output 
to a H-bridge motor control IC.

The idea for the FFT analysis for tone identification and part of the functions are based on 'Audio Tone Input' 
(toneinput.ino) by Tony DiCola, which is published with MIT License (see below) as part of the ardafruit 
learning guide and examples at http://learn.adafruit.com/fft-fun-with-fourier-transforms/. The corresponding 
function headings are marked by 'MIT License'.
