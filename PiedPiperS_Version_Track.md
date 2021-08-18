PiedPiperS Version Track

PiedPiperS
  This code is published with GNU General Public License GPL v3, 
  Copyright (c) J. Ruppert, 2021-08, jorail [at] gmx.de
  
  The program purpose is to control a model train motor with independent power supply
  e.g. from USB power bank for outdoors. Following options exist:
  a) Control fron a smartphone: ESP32 Wifi WebServer for commands from a client browser via WebSocket connection:
     https://github.com/jorail/PiedPiperS
     including options for online power data reading and speed data sampling by IR sensor
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

  Many thanks and acknowledgments to D.Meschede for support during the debugging of the PiedPiperS code.
  The idea for the FFT analysis for tone identification in PiedPiper and part of the functions are based on 
  'Audio Tone Input' (toneinput.ino) by Tony DiCola, which is published with MIT License (see below) 
  as part of the ardafruit learning guide and examples at http://learn.adafruit.com/fft-fun-with-fourier-transforms/. 
  The corresponding function headings are marked by 'MIT License'.
    
  GNU General Public License Version 3, GPLv3
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.


  The MIT License (MIT) for marked sections with copyright (c) 2013 Tony DiCola
    
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
    
    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.
    
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.


  Version track:
  056 2021-02    PiedPiper_056 stable version before Server module integration

                 Development of ESP32 Server as ESP32_AP_AWebServer_WS_JSON_020.ino
                 including whistle speed control for model trains, using microcontroller, FFT tone analysis, outdoor compatible with on-board USB power supply 
  061 2021-03-10 PiedPiperS_061 first draft of server integration (create server, before startup)   
  062 2021-03-10 Integrate Server Module startup routines
  063 2021-03-10 Integrate Server Module main loop
  064 2021-03-10 First interated PiedPiperS
  065 2021-03-10 pepare for "set value" command
  066 2021-03-10 CLIENT_COMMAND consolidated
  067 2021-03-11 adjust notifyClients
  068 2021-03-11 Debugging
  069 2021-03-11 consolidated for complete compilation
  070 2021-03-11 Debugging
  071 2021-03-11 Optimize and Debug Wifi WS command interpretation
  074 2021-03-11 Debugging Client Command 
  075 2021-03-11 First stable client command
  076 2021-03-12 Reintegrate Morse code commands, solve timing issue for Morse Signal termination/recognition
  077 2021-03-12 PiedPiperS consolidated and integrated version 
  078 2021-03-12 Optimize server program code
  079 2021-03-12 add server Info page
  080 2021-03-12 add info website
  081 2021-03-13 clean startup and info website
  082 2021-03-13 clean unused server variables, add monitor website  
  083 2021-03-13 consolidated version, 2 WLAN clients allowed
  084 2021-03-13 Webdesign, Error MotorIC management, E-Mail, License
  085 2021-03-13 consolidatedd running version of PiedPiperS
  086 2021-03-14 integrate additional html websites for license, programm code download, images
  087 2021-03-15 further web-work, static server from SPIFFS
  088 2021-03-17 debugging, serving images from SPIFFS, optimize website layout
  089 2021-03-18 fully running version, rewrite captive portal with full ip address, define minimum PWM display for speed level =1
  090 2021-03-18 html optimisation, image gallery test
  091 2021-03-19 optimize html and calls to portal, image-viewer
  092 2021-03-19 test captive redirect for https:// => not working
  093 2021-03-20 clean up, additional code comments and consolidation
  094 2021-03-20 prepare for github, consolidated and running well
  095 2021-03-21 optimize ESP32 layout and wiring, fix voltage indication, add controller status link to /monitor, enhance /parts
  096 2021-03-22 fix image gallery viewer.html, optimise /info PWM description
  097 2021-03-22 definition of #ifdef ToneSampling instead of commenting out FFT tone signal analysis as of PiedPiper version 056, reorganise #include libraries after #define
  098 2021-03-22 optimise high level links and PWM description on /info /parts /viewer    
  099 2021-03-22 optimise numbers, units in /parts reformat /viewer, PiedPiper-Project link via github.html and QR
  100 2021-03-22 consolidated version, start for github
  101 2021-03-26 preventing false positve SignalActive readings, which resulted from spikes in the ESP32 TouchRead, by averaging valarray of multiple TouchRead
  102 2021-03-26 add device width regulation to html websites in the code, add ESP modification detail img003 to parts.html
  103 2021-03-27 consolidated version, prepare for lok.ini branch and further development
  
  lok.ini branch in github: https://github.com/jorail/PiedPiperS/tree/lok.ini
  104 2021-03-28 .ini reading for variable loco name in index.html, ssid and password
  105 2021-03-29 .ini evaluation for speed_adjustment setting and fine tuning speed settings for 32 speed levels
  106 2021-03-30 .ini editing via html textarea and saving new lok.ini in SPIFFS with backup of previous version 
  107 2021-03-31 .ini reading for textbox
  108 2021-03-31 modify websocket
  113 2021-04-01 solve websocket gateway connection by soft.WifiAPIP
  114 2021-04-02 interpretation of websocket JSON message for writing lok.ini files
  115 2021-04-02 adding file management to /iniedit
  118 2021-04-02 save spiffs
  119 2021-04-02 test spiff writing
  124 2021-04-04 adressing save inifile problems
  127 2021-04-15 use Sting to solve json_message interpretation problems
  131 2021-04-16 iphone access to ini.html solved by avoiding specification "java-script", inipath with "/" instead of "\" works
  132 2021-04-17 optimise ini.html
  134 2021-04-17 move variable declaration of iniSetup in dedicated function, which can be called for reaload
  135 2021-04-17 ini.html problem fixed
  136 2021-04-17 re-introduce some alert messages, working very well
  137 2021-04-17 optimize ini.html switches
  138 2021-04-17 clean up ini.html
  139 2021-04-17 consolidate
  140 2021-04-17 add LED indication for successful writing of .ini file
  141 2021-04-17 add LED indication for loading of .ini file
  142 2021-04-17 on /dir command output fileindex as plain text, not working
  143 2021-04-17 on GET /dir command output fileindex as plain text   
  145 2021-04-17 fileindex display optimised, small fixes necessary
  146 2021-04-18 consolidated, working well
  147 2021-04-18 add location.reload() to ini.html, add LED and debugging infor to ini.html
  148 2021-04-18 cleanup, consolidation, fine tuning, amend code comments
  149 2021-04-18 consolidated code version prepared for github branch lok.ini
  150 2021-04-18 lok.ini branch upload failed
  151 2021-04-18 additional explanation on ini.html, explain .ini save problems due to limited size of ESP32 flash memory
  152 2021-04-19 lok.ini branch successful upload: https://github.com/jorail/PiedPiperS/tree/lok.ini
  153 2021-04-19 editorial change for additional idea: GPIO pin 35 instead of 26 (26 is not working when wifi in use!)
  154 2021-04-19 amend description for motor error ic input on GPIO pin 25 (25 can stay for digital input)
  155 2021-04-20 add analog input for motor current observation at GPIO pin 35 = ADC1_CH7
  156 2221-04-21 motor current sampling and monitoring in notify_client()
  157 2021-04-24 motor power data transfered as JSON response to get request from smartphone
  158 2021-04-25 add motor voltage reading and transmission in JSON
  159 2021-04-25 add implement motor voltage reading 
  160 2021-04-25 bakcup
  161 2021-04-25 test motor voltage reading
  162 2021-04-25 parametrisation of voltage analog input readings, including 1/11 kOhm voltage split, to mV 
  163 2021-04-25 debugging mV output, MotorVoltageArray.sum() integer overflow, solved by devision by 10
  164 2021-04-26 complete version including mV output to /powerdata
  165 2021-04-26 complete, running
  166 2021-04-26 add PWM data to /powerdata JSON message
  167 2021-04-26 optimise PWM data, 0 at speed_level 0
  168 2021-04-26 optimise powerdata display
  169 2021-04-27 optimise /info display
  170 2021-04-27 otimise /info and /monitor display, add buttons to power.html, working and consolidated version
  171 2021-04-27 enhance descriptions, sketch info, link to license, parts.html, add WLAN station number display, allow 3 stations
  172 2021-04-28 html text editing in /info
  173 2021-04-28 html text editing in /info /license.html /project.html
  174 2021-04-29 html text editing in captive portal and /parts.html
  175 2021-04-30 debugging /ini.html JS function
  176 2021-05-07 html text /info amended for LED indication in case of motor IC error
  178 2021-05-09 consolidated code
   
  speedo branch in github: https://github.com/jorail/PiedPiperS/tree/speedo, develop speed-o-meter with reflective ir detector on railway sleepers
  180 2021-05-09 use D15 as input
  181 2021-05-10 apply ESP32 pulse counter, driver/pcnt.h library included, test counting, successful, but glitches on logical high level
  182 2021-05-10 apply pulse counter filter https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html#filtering-pulses
  183 2021-05-11 change to Analog Input for pulse counting with deadband for edge hysteresis
  186 2021-05-12 various tests of IR measure and counting of railway sleepers via analog input and monitoring from main loop, deadband 0.5 V to 1.0 V
  187 2021-05-12 /speeddata and speed.html, speedmeasurement optimised for up to 75 km/h at model scale, test if counting is complete or lacking counts?
  189 2021-05-12 speed.html integration, zerocounter button, LED indicator toggle button
  190 2021-05-13 speed.html add IR mV signal display, decrease IR high level to 800 mV 
  191 2021-05-13 add SpeedAtScale to TextStatus in notifyClients()
  192 2021-05-14 amendment of readme, consolidate /data folger: parts.html, /image/004.jpg, add irlow and irhigh to lok.ini
  193 2021-05-15 add all relevant parameters for SpeedSampling to lok.ini in iniSetup(), add display of speedled status, irlow and irhigh values in speed.html
  194 2021-05-15 cleanup code, reducing file size, amending .ini files, delete code for digital pulse count of version 182
  195 2021-05-16 consolidated version
  196 2021-05-16 optimise IR monitoring speed for complete counting after testing with result of 60% count at fast speed
  197 2021-05-16 optimise timing in main loop, reduce calls for lengthy and slow functions, in order to increase speed sampling frequency, reorganise serial pring of global variables
  198 2021-05-17 address MonitorFrequency = 0 problem, when reading "samplerate" from lok.ini, adjust main loop frequency 
  199 2021-05-17 reorganise calculations and power sampling in main loop, main loop frequency to about 1 second, monitoring frequency set relative to result in 20 Hz, speed_wait_loops relative to monitoring frequency
  200 2021-05-17 adjust IR sample averaging procedure by lok.ini, reduce averaging default to 2, repair voltage and current sampling
  201 2021-05-18 adjust main loop timing
  202 2021-05-19
  205 2021-05-22 adjusting ini setup for main loop timing
  206 2021-05-22 debugging main loop timing variable definition, solve devision by zero problem
  207 2021-05-25 debugging main loop timing, optimise speed sampling and power sampling, MainLoopFrequencySeconds evaluated in MainLoop, not in SpeedSampling
  208 2021-05-26 loop timing without speedsampling subdevision (commented out), prevention of int devision by zero problem in ini setup for loop variables
  209 2021-05-29 monitor speed sampling
  210 2021-05-30 json serialize with data nestedjsonarray
  211 2021-05-31 json recorder only one array, toggle average sample, layout of irsamplerecord.html
  212 2021-06-01 irsamplerecord.html internal switch for 'irrecordreading', add explanation, html formatting
  213 2021-06-01 html formatting
  214 2021-06-01 round count reference value in irsamplerecord.html, move IRSampleRecording commands to void SpeedCalculation, adjust threshold for RoundMarkerDetected flag
  215 2021-06-02 debug RoundMarkerDetection by introducint valarray SpeedCountDifference, debug samplerecorderonoff toggle by absolute websocket command 'r1'=on, 'r0'=off    
  216 2021-06-02 consolidated version for speedo branch
  
  speedo branch in github: https://github.com/jorail/PiedPiperS/tree/speedo, develop speed-o-meter with reflective ir detector on railway sleepers, debugging incomplete sleeper counting
  217 2021-06-03 adjust transmission of irsamplerecord array, toggle ir sample averaging for railway sleeper detection
  218 2021-06-03 add adjustment for IRlow and IRhigh from irsamplerecord.html via websocket message
  219 2021-06-03 modify effect of flag IRSampleRecording: Extension to individual data record and json2 record submission
  220 2021-06-04 change time recording back to individual records instead of 10-fold record averaging
  221 2021-06-05 introduce switch for SpeedSampling in .ino and irsamplerecord.html

  222 2021-06-06 place SpeedSampling in interrupt function IRAverageSample
  223 2021-06-08 debug interrupt function
  224 2021-06-12 define IRArray without valarry but instead with global variable IRArrayPointer 
  225 2021-06-13 manage conflict of analogReading between TimerInterrupt and MotorCurrent Sampling by locking PortMultiplexer
                 according to https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
  228 2021-08-10 testing attaching ISR timer interrupt to core 0
  229 2021-08-11 debugging void SpeedSampling. For debugging analogRead in ISR timer interrupt refer to https://www.toptal.com/embedded/esp32-audio-sampling
  230 2021-08-16
  231 2021-08-17 using I2S AnalogReading DMA mode and ESP32 I2S example on HighFreq_ADC
  232 2021-08-17 core0 solution with normal analogRead (no ISR timer interrupt, no IRAM attribute required)
  235 2021-08-18 SpeedSamplingIRSensor running as parallel task on core0, IRsamplecounter
  236 2021-08-18 optimse task and related IR recorder outputs
  237 2021-08-18 optimse code and comments, delete ISR timer interrupt remains, ca. 6 kHz IRsensor sample frequency achieved, with main loop ca. 0.75 s and 1000 power samples/main 