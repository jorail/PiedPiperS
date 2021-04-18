/* PiedPiperS
  This code is published with GNU General Public License GPL v3, 
  Copyright (c) J. Ruppert, 2021-03, jorail [at] gmx.de
  
  The program purpose is to control a model train motor with independent power supply
  e.g. from USB power bank for outdoors. Following options exist:
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
  104 2021-03-28 .ini reading for variable loco name in index.html, ssid and password
  105 2021-03-29 .ini evaluation for speed_adjustment setting and fine tuning speed settings for 32 speed levels
  106 2021-03-30 .ini editing via html textarea and saving new lok.ini in SPIFFS with backup of previous version 
  107 2021-03-31 .ini reading for textbox
*/

////////////////////////////////////////////////////////////////////////////////
// CONIFIGURATION of the microprocessor setup and PWM control for the motor IC 
////////////////////////////////////////////////////////////////////////////////

String SKETCH_INFO = "PiedPiperS.ino, Version 107, GNU General Public License Version 3, GPLv3, J. Ruppert, 2021-03-30";

#define ESP32          //option to adjust code for interaction with different type of microProcessor 
                       //(default or comment out or empty, i.e. the else option in the if statement = Teensy4.0)
#define TLE5206        //option to adjust code for interaction with different type of MotorIC        
                       //(default or comment out or empty, i.e. the else option in the if statement = L293D)
//#define ToneSampling //###### comment out tone sampling, will be needed for FFT tone signal analysis in PiedPiper
                       //option to adjust code for tone sampling, for use together with microphone on analogue input, 
                       //sound sampling, FFT analysis and tone signal identification 
                       //this optional code originates unchanged from PiedPiper project, version 056:
                       //https://github.com/jorail/PiedPiper
                       //it has not been thoroughly tested in combination with PiedPiperS, version 60 through 100 and later
                       //https://github.com/jorail/PiedPiperS

////////////////////////////////////////////////////////////////////////////////
// Select libraries
////////////////////////////////////////////////////////////////////////////////                       

#ifdef ESP32
  #include <valarray>
  
  // Import libraries for WebServerControl module
  #include <WiFi.h> // already includes libraries #include <WiFiClient.h> and #include <WiFiAP.h>
  #include <AsyncTCP.h>
  #include <ESPAsyncWebServer.h>
  #include <DNSServer.h>
  #include <ArduinoJson.h>

  //libraries for ini file handling https://github.com/yurilopes/SPIFFSIniFile

  #include "FS.h"  //################### double chekc if really needed
  #include <SPIFFS.h>
  #include <SPIFFSIniFile.h>  


  
#endif

#ifdef ToneSampling
  // ###### comment out libraries for tone sampling, will be needed for FFT tone signal analysis in PiedPiper
  // Import libraries for TrainMotorDriver module
  #define ARM_MATH_CM4
  #include <valarray>
  #include <arduinoFFT.h>
  
  // Libraries for SoundControl module
  //#include <arm_math.h>
  //#include <dsps_fft4r.h> 
  //#include <wavelib.h>
#endif

////////////////////////////////////////////////////////////////////////////////
// Microprocessor and Development Board Pin-out definition
////////////////////////////////////////////////////////////////////////////////

#ifdef ESP32 
  // input output defintions

  int input_switch  = 13;          // D13 (touch switch input: HIGH=steady state; LOW < 60 = activated, Pin 13 = Touch4 = T4 => if((touchRead(input_switch) < 60)
  int input_switch_level = 60 ;    // Set high sensitivity at 60 with very short wire at Touch-Input pin, LOW value < 60 = activated (default with some longer cable at Touch-Input pin: LOW < 50 = activated)
  int POWER_LED_PIN = 4;           // optional output pin for power LED (use ESP32 GPIO pin D15 next to D02 onboard LED pin), alternatively, if no seperate power LED is attached, redLED pin can be used with only minor interference
  int greenLED      = 2;           // PIN of on-board blue LED for simple digitalWrite, D02 pin
  int redLED        = 4;           // PIN of on-board red LED for simple digitalWrite, after hardware modification by connection via 1kOhm resistero to D04 pin
  int motor2        = 32;          // D32 digital output to motor H-bridge ic input 2 setting motor direction4
  int motor1        = 33;          // D33 digital output to motor H-bridge ic input 1 setting motor direction
  int motor_enable  = 25;          // D32 PWM output to motor H-bridge ic enable setting motor speed
  int error_motoric = 25;          // motor H-bridge ic error flag, for TLE5206 on pin 25 must be defined as input, alternatively: D12 

  #define LEDC_TIMER_BITS   8            // PWM properties, use 8 bit precission for LEDC timer with 256 levels
  #define LEDC_BASE_FREQ    5000         // PWM properties, use 5000 Hz as a LEDC base frequency
  const int pwmChannel1   = 0;           // required for ESP32 in combination with either L293D or TLE5206 motor ic
  const int pwmChannel2   = 1;           // only required in combination of ESP32 and TLE5206 motor ic
  int dutyCycle           = 200;

  int AUDIO_INPUT_PIN = 14;        // Input D14 = ADC16 pin for audio signal.
  int LIGHT_INPUT_PIN = 27;        // Input D27 = ADC17 pin for light signal.
  int ANALOG_READ_RESOLUTION = 10; // Bits of resolution for the ADC.
  int ANALOG_READ_AVERAGING = 8;   // Number of samples to average with each ADC reading. Recommanded for ESP32 =8. For Teensy 4.0 successfully tested with =16


#else  //uP not definied, defualt for Teensy 4.0
  // input output defintions
  int input_switch  =  5;          // manual switch input: HIGH=steady state, LOW=activated
  int POWER_LED_PIN = 13;          // Output pin for power LED (pin 13 to use Teensy 4.0's onboard LED).
  int greenLED      = 11;          // PIN of LED for simple digitalWrite
  int redLED        = 12;          // PIN of LED for simple digitalWrite
  int motor1        =  9;          // PWM output to motor H-bridge ic input 1
  int motor2        = 10;          // PWM output to motor H-bridge ic input 2
  int error_motoric =  4;          // motor H-bridge ic error flag

  int AUDIO_INPUT_PIN = 21;        // Input ADC pin for audio signal.
  int LIGHT_INPUT_PIN = 20;        // Input ADC pin for light signal.
  int ANALOG_READ_RESOLUTION = 10; // Bits of resolution for the ADC.
  int ANALOG_READ_AVERAGING = 16 ; // Number of samples to average with each ADC reading. Recommanded for ESP32 =8. For Teensy 4.0 successfully tested with =16

#endif

////////////////////////////////////////////////////////////////////////////////
// Main loop timing, monitoring, speed adjustments and Morse code processing
////////////////////////////////////////////////////////////////////////////////

// mainloop and monitoring
const int MONITOR_FREQ =   10;                      // Frequency of monitoring switch Morse signal input (ca. 10 Hz or 0.1 second, defined by division of main LOOP COUNT)
const int FLASH_FREQ   =   2*5059;                  // Frequency for power LED flashes while void loop running (ca. 1 to 2 seconds, defined by value for main LOOP_COUNT), default 18815 on ESP32 without WebServer, 5059 on ESP with WebServer v106
const int FLASH_FREQ_DARK = int(FLASH_FREQ * 0.99); // flash frequency period with unlit power LED, use number close to 1.00, when redLED is used as alternative powerLEDdefault 0.99

//MorseCodeDecoder
const int dotMinLength   =  200; //dots are more than 100ms long and shorter than dashMinLength
const int dashMinLength  =  800; //dashes are more than 300ms long
const int TerminalLength = 1500; //wait after last dash/dot for termination of command
const int   morsecode_size = 12;
const char *morsecode[morsecode_size] = {".",  "..",       "...",                              "....", ".....", ".-", ".--",       ".---",             "-", "--", "---", "?"};
const char *commands[morsecode_size]  = {"--", "--------", "--------------------------------", "0",    "0<",   "++",  "++++++++", "++++++++++++++++", "0", "00", "?",   "?"};
const char *meaning[morsecode_size]   = {"decrease speed", "slow down", "break to halt", "fast brake and stop", "fast brake, stop, reverse direction", "increase speed", "speed up", "go fast", "fast brake and stop", "fast brake and stop", "info?", "info?"};
//acceleration is limited to half full speed, i.e. 8x '+' character, while full break is from max and any speed to halt, i.e. 16x '-' character
//use fast brake to immediatly stop the motor by command charcter '0',
//'<'   stands for reverse of direction, which can only be done after a safe full halt, thus multiple leading '0' characters are introduced in front for train inertia
const int MAX_CHARS      = 65;   // Max size of the input command buffer

#ifdef ToneSampling
////////////////////////////////////////////////////////////////////////////////
// CONIFIGURATION tone signal analysis
// ###### comment out tone sampling, will be needed for FFT tone signal analysis in PiedPiper
// <=######## These values can be changed to alter the behavior of the FFT tone analysis.
// Some values relate to the spectrum display. 
// ////////////////////////////////////////////////////////////////////////////////
  
  //toneLoop, parameters for tone signal detection
  int SAMPLE_RATE_HZ = 5120;             // Sample rate of the audio in hertz. Default= 9000 Hz => resolution 37 Hz; increased resolution = 5120 => resolution = 20 Hz => highest detectable frequency = 2500 Hz
  const int FFT_SIZE = 256;              // Size of the FFT.  Realistically can only be at most 256 = default: Frequency bin siez (Hz) = SAMPLE_RATE_HZ/FFT_SIZE
  float TONE_THRESHOLD1_DB = 11.0;       // <=########## Sensitivity threshold (in decibels) for energy in tone window, which must be above energy in other frequencies for successful detection, default 10.0. in quite room, default 12.0 with speach in the environment
  std::valarray<int> tone_array(8);      // <=########## setting room for gliding average of tones detected in tone loop, default tone_array size = 10, lower values for faster response time
  float TONE_THRESHOLD2_DB = 9.0;        // <=########## Sensitivity threshold (in decibels) for the gliding average of tones, which must be above other frequencies for successful, default 15.0 in quite room, default 14.0 in noisy environment
  int  LIGHT_THRESHOLD = 20;             // <=######## if below threshold then the light signal is active, choose analog input threshold value between 1 and 1023
  int TONE_FREQ_OFFSET = int(200/(SAMPLE_RATE_HZ/FFT_SIZE)); // Default 200 Hz converted into number of low frequency bins to be skipped for 'windowMean' evaluation and difference of energy in window vs. in other frequencies
  const int TONE_LOWS[] = {              // <=########## Lower bound (in hz) of each tone in the input tone window (or sequence). Default single tone = h'' = 988 Hz. Default tone sequence: 1723, 1934, 1512, 738, 1125. 
    988
  };
  const int TONE_HIGHS[] = {             // <=########## Upper bound (in hz) of each tone in the input tone window (or sequence). Default single tone = h'' = 988 Hz. Default tone sequence: 1758, 1969, 1546, 773, 1160. 
    988
  };
  // Margin for calculation of the low and high frequency bin limites for the currently expected tone.
  int TONE_ERROR_MARGIN_HZ = 50;         // <=########## Allowed fudge factor above and below the bounds for each tone input. Default = 50 Hz
  
  // Tone sequence detection
  int tonePosition = 0;                  // start with first TONE_LOWS[] and TONE_HIGH[], multible values in the array allow for tone sequence code detection
  int TONE_WINDOW_MS = 4000;             // Maximum amount of milliseconds allowed to enter the full tone sequence detection, default 4000. without running out of memory for buffers and other state

/* Tone frequency table, keep this information for adjusting FFT analysis in the code
Tone Hertz      Tone Hertz      Tone Hertz        Tone    Hertz       Tone   Hertz
''A  27,50      C   65,41       c'    261,63      c'''    1.046,50    c''''' 4.186,01
''B  29,14      Des 69,30       des'  277,18      des'''  1.108,73           
''H  30,87      D   73,42       d'    293,66      d'''    1.174,66           
'C   32,70      Es  77,78       es'   311,13      es'''   1.244,51           
'Des 34,65      E   82,41       e'    329,63      e'''    1.318,51           
'D   36,71      F   87,31       f'    349,23      f'''    1.396,91           
'Es  38,89      Ges 92,50       ges'  369,99      ges'''  1.479,98           
'E   41,20      G   98,00       g'    392,00      g'''    1.567,98           
'F   43,65      As  103,83      as'   415,30      as'''   1.661,22           
'Ges 46,25      A   110,00      a'    440,00      a'''    1.760,00           
'G   49,00      B   116,54      b'    466,16      b'''    1.864,66           
'As  51,91      H   123,47      h'    493,88      h'''    1.975,53           
'A   55,00      c   130,81      c''   523,25      c''''   2.093,00           
'B   58,27      des 138,59      des'' 554,37      des'''' 2.217,46           
'H   61,74      d   146,83      d''   587,33      d''''   2.349,32           
                es  155,56      es''  622,25      es''''  2.489,02           
                e   164,81      e''   659,26      e''''   2.637,02           
                f   174,61      f''   698,46      f''''   2.793,83           
                ges 185,00      ges'' 739,99      ges'''' 2.959,96           
                g   196,00      g''   783,99      g''''   3.135,96           
                as  207,65      as''  830,61      as''''  3.322,44           
                a   220,00      a''   880,00      a''''   3.520,00           
                b   233,08      b''   932,33      b''''   3.729,31           
                h   246,94      h''   987,77      h''''   3.951,07           
*/
#endif

////////////////////////////////////////////////////////////////////////////////
// INTERNAL STATE and variable initialisation
// These shouldn't be modified unless you know what you're doing.
////////////////////////////////////////////////////////////////////////////////

#ifdef ToneSampling
  //###### comment out tone sampling, will be needed for FFT tone signal analysis in PiedPiper
  //toneLoop
  IntervalTimer samplingTimer;  //###TODO correct for use of code with ESP32
  float samples[FFT_SIZE * 2];
  float magnitudes[FFT_SIZE];
  
  int sampleCounter = 0;
  unsigned long toneStart = 0;
#endif

char commandBuffer[MAX_CHARS];

// mainloop and monitoring
int LOOP_COUNT = 0;            // Monitor the void loop running by flashing power LED
int FLASH_COUNT = 0;

int TONE_LOOP_COUNT =0; 
int TONE_ACTIVE_COUNT =0;  
String speed_command = "";     // variable for parsing the execution of a morse command
int speed_wait_countdown = 0;  // loop count for delay of next step of speed adjustment, set to 0 for immediate execution of new speed_command
int speed_level = 0;           // speed_level for PWD output to motor
int max_speed_level = 32;      // maximum speed level, default =32 in odrder to get smooth speed increase/decrease, adjust speed_wait_loops and number of *commands speed adjust symbols in appropriately 
                               // (max_speed_level=16 as simplified method up to version 028 with half the number of command symbols)
bool speed_direction = true;   // true=forward, false=backward

//speed adjustment, defined as variables for later adjustment of following default values by reading values from .ini in setup()
float motor_voltage_supply   = 9.5;                 // indicate motor voltage supply level in Vdc, as adjusted at step-up converter output
float motor_voltage_start = 6.0;                    // equivalent voltage for first speed level PWM setting
float speedoffset = 1- ((motor_voltage_supply - motor_voltage_start)/(max_speed_level - 1) * max_speed_level / motor_voltage_supply) ; 
// offset of pulse width modulation (PWM) for speed_level "0" as ratio of maximum pulse width, default 0.60 @ 9.5 V dc, 0.50 @ 10 Vdc, 0.30 @ 12Vdc, default 0.25 @ 16Vdc
int speed_frequency = 16;                            // frequency of speed adjustment per FLASH_FREQ in main loop
int speed_wait_loops = FLASH_FREQ / speed_frequency; // Number of loops for delay of next step of speed adjustment, default = ca. 0.5 seconds

// variables from whistle004.ino, partially obsolete?
int Intensity = 0;
int IntensityStep = 8;
int switch_ok = HIGH;          // (not used in version 030)
int motor_ok = HIGH;           // motor ic error flag (not used in version 030 to 45)
int sound = 0;

// variables from MorseCodeDecoder

#ifdef ESP32
  // ESP touch sensor multiple reading for robust results, preventing false positive switch_signals
  std::valarray<int> touch_array(5);      // for assessment of gliding average of touch readings in Morse signal detection loop, default touch_array size = 10, lower values for faster response time
#endif

bool switch_signal        = HIGH;
bool tone_signal          = false;
bool light_signal         = false;
// int  light_level       = 0;    //disabled light level reading
unsigned long SignalTimer = 0;    //for recording the time how long the button was pressed
int SignalDuration        = 0;
bool SignalActive         = false;
bool shortSignalActive    = false;
bool longSignalActive     = false;
bool verylongSignalActive = false; //2xlongSignal as additional signal for emergency brake
bool SignalTerminated     = true;
String currentSignal      = ""; //string to hold what is currently being signalled, complete Morse command after termination

////////////////////////////////////////////////////////////////////////////////
// .ini file handling, according to https://github.com/yurilopes/SPIFFSIniFile, GNU GPL v3
////////////////////////////////////////////////////////////////////////////////
#ifdef ESP32 

//for debugging .ini file reading errors
void printErrorMessage(uint8_t e, bool eol = true)
{
  switch (e) {
  case SPIFFSIniFile::errorNoError:
    Serial.print("no error");
    break;
  case SPIFFSIniFile::errorFileNotFound:
    Serial.print("file not found");
    break;
  case SPIFFSIniFile::errorFileNotOpen:
    Serial.print("file not open");
    break;
  case SPIFFSIniFile::errorBufferTooSmall:
    Serial.print("buffer too small => ");
    Serial.print("limit .ini line length to bufferLen (default 40 characters!)");
    break;
  case SPIFFSIniFile::errorSeekError:
    Serial.print("seek error");
    break;
  case SPIFFSIniFile::errorSectionNotFound:
    Serial.print("section not found");
    break;
  case SPIFFSIniFile::errorKeyNotFound:
    Serial.print("key not found");
    break;
  case SPIFFSIniFile::errorEndOfFile:
    Serial.print("end of file");
    break;
  case SPIFFSIniFile::errorUnknownError:
    Serial.print("unknown error");
    break;
  default:
    Serial.print("unknown error value");
    break;
  }
  if (eol)
    Serial.println();
}

#endif

////////////////////////////////////////////////////////////////////////////////
// Create Server Module for ESP32
////////////////////////////////////////////////////////////////////////////////
#ifdef ESP32 

//Create DNS Server on port 53
const byte DNS_PORT = 53;
DNSServer dnsServer;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Create CaptivePortal, redirect used for any specific device requests for captive portal handling
void captivePortalTarget(AsyncWebServerRequest *request) {
      AsyncResponseStream *response = request->beginResponseStream("text/html");
      response->print("<!DOCTYPE html><html><head><title>Infos zur Lokomotivsteuerung</title>");
      response->print("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
      response->print("<style>body{max-width: 1024px; font-size: 1.0rem; font-family: Helvetica; display: inline-block; padding:2%; margin:2% auto; text-align: left;}");
      response->print("h1 {font-size: 1.6rem;} h2 {font-size: 1.2rem;}</style></head><body>");
      response->printf("<h1>Startseite zur Loksteuerung</h1>");
      response->printf("<p>Dies ist die automatische Startseite des ESP32 WLAN-Servers (CaptivePortalTarget). ");
      response->printf("Du hast versucht folgende Seite zu erreichen und bist zur Startseite geleitet worden:</p>");
      response->printf("<p>http://%s%s</p>" , request->host().c_str(), request->url().c_str());
      response->printf("<p>Der ESP32 hat keinen Internet&shy;zugang. Du kannst aber &uuml;ber die Verbindung von deinen Browser zum ESP32 WLAN-Server eine Lokomotive steuern. ");
      response->printf("Es k&ouml;nnen immer nur zwei Nutzer (Clients) gleichzeitig mit dem WLAN-Server Verbindung aufnehmen. </p>"); 
      response->printf("<h2><a href='http://%s/'>Loksteuerung</a> &nbsp;&nbsp; <a href='http://%s/info'>weitere Informationen</a></h2>", WiFi.softAPIP().toString().c_str(), WiFi.softAPIP().toString().c_str());
      response->printf("<div><a href='http://%s/'><img src=\"/lok.png\" alt='Loksteuerung' style='width:100%%'></a></div>", WiFi.softAPIP().toString().c_str());      
      response->printf("<p>Viel Spa&szlig; beim Ausprobieren, Jo</p>");      
      response->print("</body></html>");
      request->send(response);
};

class CaptiveRequestHandler : public AsyncWebHandler {
  public:
    CaptiveRequestHandler() {}
    virtual ~CaptiveRequestHandler() {}
    bool canHandle(AsyncWebServerRequest *request) {
      request->addInterestingHeader("ANY");
      return true;
    }
    void handleRequest (AsyncWebServerRequest *request) {
       //request->redirect("/portal");
      AsyncResponseStream *response = request->beginResponseStream("text/html");
      response->print("<!DOCTYPE html><html><head><title>Infos zur Lokomotivsteuerung</title>");
      response->print("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
      response->print("<style>max-width: 1024px; body{font-size: 1.0rem; font-family: Helvetica; display: inline-block; padding:2%; margin:2% auto; text-align: left;}");
      response->print("h1 {font-size: 1.6rem;} h2 {font-size: 1.2rem;}</style></head><body>");
      response->printf("<h1>Startseite zur Loksteuerung</h1>");
      response->printf("<p>Dies ist die automatische Startseite des ESP32 WLAN-Servers (CaptiveRequestHandlter). ");
      response->printf("Du hast versucht folgende Seite zu erreichen und bist zur Startseite geleitet worden:</p>");
      response->printf("<p>http://%s%s</p>" , request->host().c_str(), request->url().c_str());
      response->printf("<p>Der ESP32 hat keinen Internetzugang. Du kannst aber &uuml;ber die Verbindung von deinen Browser zum ESP32 WLAN-Server eine Lokomotive steuern. ");
      response->printf("Es k&ouml;nnen immer nur zwei Nutzer (Clients) gleichzeitig mit dem WLAN-Server Verbindung aufnehmen. </p>"); 
      response->printf("<h2><a href='http://%s/'>Loksteuerung</a> &nbsp;&nbsp; <a href='http://%s/info'>weitere Informationen</a></h2>", WiFi.softAPIP().toString().c_str(), WiFi.softAPIP().toString().c_str());
      response->printf("<div><a href='http://%s/'><img src=\"/lok.png\" alt='Loksteuerung' style='width:100%%'></a></div>", WiFi.softAPIP().toString().c_str());      
      response->printf("<p>Viel Spa&szlig; beim Ausprobieren, Jo</p>");      
      response->print("</body></html>");
      request->send(response);
    }
};
#endif

////////////////////////////////////////////////////////////////////////////////
// WebSocket Module for ESP32
////////////////////////////////////////////////////////////////////////////////
#ifdef ESP32 

//Create WebSocket
void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
};

//Websocket Event
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      Serial.println("");
      Serial.println("WebSocket event data received, handle WS message");
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

// Websocket input processing
String client_message ="Bereit..."; //intial command message on index.html 

//Evaluate Websocket Message
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {

        Serial.print("WebSocket message received: ");
        data[len] = 0;
        client_message = (char*)data;
        Serial.println(client_message); //echo received message for test and monitoring
        if (client_message[1]='i') {
          writeIni (client_message);
        }
        else {
          //ws.textAll(client_message); //echo received message for test only
          CLIENT_COMMAND(client_message.c_str()); //compilation type problem solved by .c_str()   
        }  
    }
}

void writeIni (String newinitxt)  {
   Serial.println("write Ini : "); 
   Serial.println(newinitxt); //echo received message for test and monitoring
}


// Send WebSocket Message as JSON Document
void notifyClients() {
  Serial.print ("Notify clients! ");
  // Compute the capacity for the JSON document,
  // which is used to exchange status information 
  // and commands via WebSocket
  // adjust according to members in JSON document, 
  // see https://arduinojson.org/v6/assistant/
  const uint8_t size = 192; //JSON_OBJECT_SIZE(3);
  StaticJsonDocument<size> json;
  String TextStatus ="";
  if (speed_direction) {
    TextStatus="Vorwärts ";
  }
  else {
    TextStatus="Rückwärts ";
  }
  TextStatus = TextStatus + String(speed_level*5)+" km/h";
  json["status"] = TextStatus;
  Serial.print(" speed_level: ");
  Serial.print(speed_level);
  String TextSpeed ="";
  for (int i = 0;  i< speed_level/2; ++i) {
    if (speed_direction) {
      TextSpeed =  TextSpeed + " >";
    }
    else {
      TextSpeed =  TextSpeed + " <";
    }  
  }
  Serial.print(" | TextSpeed: ");
  Serial.print(TextSpeed);
  json["speed"] = TextSpeed;
  json["command"] = client_message; 
  char data[size];
  size_t len = serializeJson(json, data);
  ws.textAll(data, len);
  Serial.print(" | WebSocket text to all:");
  data[len] = 0;
  Serial.println((char*)data);
}

#ifdef ESP32
  //define buffer length for .ini file readings
  const char *filename = "/lok.ini";
  const size_t bufferLen = 41;  //maximum .ini file line length of 40 characters plus '\0' character
  char buffer[bufferLen];

  char loconame[bufferLen] = {0};
  char ssid[bufferLen]     = {0};
  char password[bufferLen] = {0};
 
#endif 

// Adjust index.html variables at start or reload
String processor(const String& var){
    Serial.print("processor working on: ");
    Serial.println(var);

    if(var == "LOCONAME"){
      return String(loconame); // send loconame for replacement of the header variable in index.html
    }
   
    //other variables will not by replaced here by processor but by notifyClients() and  
    //sending a JSON message after a short delay
    return "Starte...";
  }
#endif

////////////////////////////////////////////////////////////////////////////////
// SETUP 
////////////////////////////////////////////////////////////////////////////////
void setup() {

#ifdef ESP32 
  setCpuFrequencyMhz(80); //reduce CPU frequency for power saving
  Serial.begin(115200);   // Set up serial port.

////////////////////////////////////////////////////////////////////////////////
// Open .ini file, according to https://github.com/yurilopes/SPIFFSIniFile, GNU GPL v3
////////////////////////////////////////////////////////////////////////////////

  // Initialize SPIFFS
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // open .ini file
  SPIFFSIniFile ini(filename);
  if (!ini.open()) {
    Serial.print("Ini file ");
    Serial.print(filename);
    Serial.println(" does not exist");
  }
  Serial.println("Ini file exists");

  // Check if the file is valid. This can be used to warn if any lines are longer than the buffer.
  if (!ini.validate(buffer, bufferLen)) {
    Serial.print("ini file ");
    Serial.print(ini.getFilename());
    Serial.print(" not valid: ");
    printErrorMessage(ini.getError());
  }

/////////////////////////////////////////////////////////////////////////////////////////////
// Read values from the .ini file > section > key, when the key is present
/////////////////////////////////////////////////////////////////////////////////////////////

  // Define defaultnetwork credentials and name
  const char* loconame_default = "Loksteuerung";
  const char* ssid_default = "Lok_wlan";
  const char* password_default = "freifahrt";

  //loco name from .ini
  if (ini.getValue("loco", "name", buffer, bufferLen)) {strncpy(loconame,buffer,bufferLen);}
  else {
    Serial.print(".ini no loconame! ");
    printErrorMessage(ini.getError());
    strncpy (loconame,loconame_default,bufferLen);
  }

  //wifi ssid from .ini
  if (ini.getValue("wifi", "ssid", buffer, bufferLen)) {strncpy(ssid,buffer,bufferLen);}
  else {
    Serial.print(".ini no wifi ssid! ");
    printErrorMessage(ini.getError());
    strncpy (ssid,ssid_default,bufferLen);
    //Serial.print("ssid default = ");
    //Serial.println(ssid);
  }

  //wifi password from .ini
  if (ini.getValue("wifi", "password", buffer, bufferLen)) {strncpy(password,buffer,bufferLen);}
  else {
    Serial.print(".ini no wifi password! ");
    printErrorMessage(ini.getError());
    strncpy (password,password_default,bufferLen);
  }

  Serial.print("loco name: "); Serial.println(loconame);
  Serial.print("ssid: "); Serial.println(ssid);
  Serial.print("password: "); Serial.println(password);

  //voltage settings, adjustment of default values by reading values from .ini
  if (ini.getValue("voltage", "supply", buffer, bufferLen)) {motor_voltage_supply = atof(buffer);}
  else {Serial.println(".ino default voltage supply");}
  if (ini.getValue("voltage", "start", buffer, bufferLen)) {motor_voltage_start = atof(buffer);}
  else {Serial.println(".ino default voltage start");}
  speedoffset = 1- ((motor_voltage_supply - motor_voltage_start)/(max_speed_level - 1) * max_speed_level / motor_voltage_supply) ; 
  // offset of pulse width modulation (PWM) for speed_level "0" as ratio of maximum pulse width, default 0.60 @ 9.5 V dc, 0.50 @ 10 Vdc, 0.30 @ 12Vdc, default 0.25 @ 16Vdc

  //speed adjustment, adjustment of default values by reading values from .ini
  if (ini.getValue("speed", "adjustment_frequency", buffer, bufferLen)) {
    speed_frequency = atoi(buffer);
    Serial.print(".ini speed_frequency = ");
    Serial.println(speed_frequency);
  }
  else {Serial.println(".ino default speed adjustment frequency");}
  speed_wait_loops = FLASH_FREQ / speed_frequency; // Number of loops for delay of next step of speed adjustment, default = ca. 0.5 seconds

  // add reading pinout from .ini here ###########

#endif

////////////////////////////////////////////////////////////////////////////////
// SETUP Main Functions, check for brownout
////////////////////////////////////////////////////////////////////////////////
  
  pinMode(input_switch, INPUT);  
 
#ifdef ESP32 

  // check for brownout or other startup reason
  esp_reset_reason_t reset_reason = esp_reset_reason();
  Serial.println();
  Serial.print("Startup reset_reason: ");
  Serial.print(reset_reason);
  Serial.print(" = ");
  switch (reset_reason)
    {
    case ESP_RST_UNKNOWN:    Serial.println("Reset reason can not be determined"); break;
    case ESP_RST_POWERON:    Serial.println("Reset due to power-on event"); break;
    case ESP_RST_EXT:        Serial.println("Reset by external pin (not applicable for ESP32)"); break;
    case ESP_RST_SW:         Serial.println("Software reset via esp_restart"); break;
    case ESP_RST_PANIC:      Serial.println("Software reset due to exception/panic"); break;
    case ESP_RST_INT_WDT:    Serial.println("Reset (software or hardware) due to interrupt watchdog"); break;
    case ESP_RST_TASK_WDT:   Serial.println("Reset due to task watchdog"); break;
    case ESP_RST_WDT:        Serial.println("Reset due to other watchdogs"); break;
    case ESP_RST_DEEPSLEEP:  Serial.println("Reset after exiting deep sleep mode"); break;
    case ESP_RST_BROWNOUT:   Serial.println("Brownout reset (software or hardware)"); break;
    case ESP_RST_SDIO:       Serial.println("Reset over SDIO"); break;
    default :                Serial.println("Reset reason not recognized"); break;
    }
 
  if (reset_reason == ESP_RST_BROWNOUT) {
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_deep_sleep_start();
    delay(180000); // 3 min delay for cool down and power recovery before continuation in deep_sleep_mode
    pinMode(greenLED, OUTPUT);
    for (int i = 0; i < 150; i++) { //indicate brown out error by fast flashing of green LED for 30 seconds
      digitalWrite(greenLED, HIGH);
      delay(100);
      digitalWrite(greenLED, LOW);
      delay(100);
    }
  }

  delay(2000); //wait 2 s at startup in order to alliviate start-up brown-out cycle
  // Initialise LEDs.
  pinMode(POWER_LED_PIN, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  
  digitalWrite(POWER_LED_PIN, LOW);
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, HIGH);
  for (int i = 0; i < reset_reason;i++) {
    digitalWrite(greenLED, LOW);
    delay(480);
    digitalWrite(greenLED, HIGH);
    delay(20);
  }
    
  while (!(touchRead(input_switch) < input_switch_level)) { //wait for touch signal at startup in order to alliviate start-up brown-out cycle
    Serial.print("Waiting for startup signal at GPIO pin ");
    Serial.print(input_switch);
    Serial.print(" TOUCH read= ");
    Serial.println(touchRead(input_switch)); 
    
    delay(950); 
    digitalWrite(greenLED, LOW);
    delay(50);
    digitalWrite(greenLED, HIGH);
  }

  // Set up ADC and audio input.
  pinMode(AUDIO_INPUT_PIN, INPUT);
  pinMode(LIGHT_INPUT_PIN, INPUT);
  analogReadResolution(ANALOG_READ_RESOLUTION); 
  //analogReadAveraging(ANALOG_READ_AVERAGING);  //#####TODO test if adjustment to ESP analog input reading commands is functioning
  //analogSetCycles(ANALOG_READ_AVERAGING);      //#####TODO test if adjustment to ESP analog input reading commands is functioning

#else  //mP not definied, defualt for Teensy 4.0
  // Set up serial port.
  Serial.begin(38400);

  // Initialise LEDs.
  pinMode(POWER_LED_PIN, OUTPUT);
  digitalWrite(POWER_LED_PIN, LOW);
  pinMode(redLED, OUTPUT);
  digitalWrite(redLED, HIGH);
  pinMode(greenLED, OUTPUT);
  digitalWrite(greenLED, HIGH);

  // Set up ADC and audio input.
  pinMode(AUDIO_INPUT_PIN, INPUT);
  pinMode(LIGHT_INPUT_PIN, INPUT);
  analogReadResolution(ANALOG_READ_RESOLUTION);
  analogReadAveraging(ANALOG_READ_AVERAGING);
  
#endif

#if defined(ESP32) && defined(TLE5206) //motor IC, TLE5206 is using only 2 input signals but offers an error flag output
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(error_motoric, INPUT_PULLUP); //use internal 45 kOhm pullup resistor
  // configure ESP32 LEDC PWM functionalitites for TLE5206
  ledcSetup(pwmChannel1, LEDC_BASE_FREQ, LEDC_TIMER_BITS);
  ledcAttachPin(motor1, pwmChannel1); // attach the channel to the GPIO to be controlled
  ledcSetup(pwmChannel2, LEDC_BASE_FREQ, LEDC_TIMER_BITS);
  ledcAttachPin(motor2, pwmChannel2); // attach the channel to the GPIO to be controlled

#elif defined(ESP32)&& !defined(TLE5206)  //motorIC not defined, default L293D, using motor_enable for PWM signal
  //pinMode(error_motoric, INPUT);
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor_enable, OUTPUT);
  // configure ESP32 LEDC PWM functionalitites
  ledcSetup(pwmChannel1, LEDC_BASE_FREQ, LEDC_TIMER_BITS);
  ledcAttachPin(motor_enable, pwmChannel1); // attach the channel to the GPIO to be controlled

#else   //assume setting for Teensy 4.0 with TLE5206 motor IC
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(error_motoric, INPUT);
  
#endif

  // Clear the input command buffer
  memset(commandBuffer, 0, sizeof(commandBuffer));
  speed_command = "00"; //initial brake speed_command in order to check routines and LED indication

#ifdef ToneSampling
  // ###### comment out tone sampling, will be needed for FFT tone signal analysis in PiedPiper
  // Begin sampling audio
  samplingBegin();
#endif

////////////////////////////////////////////////////////////////////////////////
// Setup WiFi WebServer and CaptivePortal redirects
////////////////////////////////////////////////////////////////////////////////
#ifdef ESP32

  // Open WiFi Access Point with SSID and password
  Serial.println("Setting WiFi Access Point…");
  Serial.print("SSID = ");
  Serial.println(ssid);
  Serial.print("password = ");
  Serial.println(password);
  // Remove the password parameter, if you want the AP (Access Point) to open unprotected
  WiFi.softAP(ssid, password,1,0,2); //chanel=1, broadcastID=0, max_connections=2 //changed from 1 to two clients in version 083
  IPAddress IP = WiFi.softAPIP();
  Serial.print("WiFi Access Point IP address: ");
  Serial.println(IP);

  // Setup DNS Server and WebSocket communication
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  initWebSocket();

  // Route for root web page = /index.html
  // When the server receives a request on the root “/” URL, it will send the index.html file to the client.
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    //request->send_P(200, "text/html", index_html, processor);
    request->send(SPIFFS, "/index.html", String(), false, processor); //processor replaces variables in index.html
    Serial.println("Webserver root called. Send index.html");
    speed_command = '?'; //for updating notification strings on index.html via adjustSpeed(), notifyClients() and JASON message (alternative to update via processor)
  });

  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // Route to load locomotive image file
  server.on("/lok", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/lok.png", "image/png");
  });

  // Route to CaptivePortal
  server.on("/portal", captivePortalTarget);  

  // Route to load status info and instruction
  server.on("/info", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncResponseStream *response = request->beginResponseStream("text/html");
      response->print("<!DOCTYPE html><html><head><title>Infos zur Lokomotivsteuerung</title>");
      response->print("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
      response->print("<style>body{max-width: 1024px; font-size: 1.0rem; font-family: Helvetica; display: inline-block; padding:2%; margin:2% auto; text-align: left;}");
      response->print("table, tr {vertical-align: top; padding: 1%; border-collapse: collapse; border-bottom: 2pt solid #ddd;} tr:last-child { border-bottom: none; }"); 
      response->print("th, td {text-align: left; padding: 1%; } h1 {font-size: 1.8rem;} h2 {font-size: 1.2rem;}</style></head><body>");
      response->printf("<h2><a href='/project.html'>PiedPiperS-Projekt</a></h2>");
      response->printf("<h2><a href='/access'>WLAN-Zugang</a></h2>");
      response->printf("<h1>Infos zur Loksteuerung</h1>");
      response->printf("<p>Der Datenaustausch zwischen der Webseite zur Loksteuerung im Browser und dem ESP32 WLAN-Server ");  
      response->printf("auf dem ersten Wagen hinter der Lok erfolgt &uuml;ber das WebSocket-Protokoll. Der ESP32 steuert den angeschlossenen Motor. </p>"); 
      response->printf("<p>Ein Fahrbefehl wird als Zeichenkette an den WLAN-Server &uuml;bermittelt. Wenn der ESP32 Mikrocontroller den Fahrbefehl erkennt, setzt er ihn "); 
      response->printf("schrittweise Zeichen f&uuml;r Zeichen zur Steuerung des Motors um. Dies wird durch zwei Leuchtdioden (LEDs) auf dem Mikrocontroller angezeigt. "); 
      response->printf("Bei jedem Schritt meldet er &uuml;ber WebSocket Informationen an alle WLAN-Nutzer (Clients) zur&uuml;ck. "); 
      response->printf("Damit werden die Fahrtrichtung, die Geschwindigkeit und der erhaltenen Fahrbefehl in der Anzeige der Webseite aktualisiert.</p>"); 
      response->printf("<p> </p>"); 
      response->printf("<p><b>MOTORSTEUERUNG</b></p>");
      response->printf("<p>Die Stromversorgung der Lok erfolgt nicht durch die Schiene sondern aus einer mitgef&uuml;hrten Batterie oder USB-Powerbank. ");
      response->printf("Aus 5 Volt Gleichspannung wird in einem Step-up-Konverter eine Gleichspannung von etwa %.1f Volt erzeugt.</p>", motor_voltage_supply);      
      response->printf("<p>Eine Puls&shy;weiten&shy;modulierung (PWM) im ESP32-Mikro&shy;controller mit %i Geschwindig&shy;keits&shy;stufen dient zur Steuerung der Lok. ",max_speed_level); 
      response->printf("Dabei wird das PWM-Signal in einem integrierten Schaltkreis (IC) mit einer H-Br&uuml;cke verst&auml;rkt, um den Gleich&shy;strom&shy;motor per PWM zu regeln. ");
      response->printf("Die erste Geschwindig&shy;keits&shy;stufe entspricht einer PWM von %.0f %%. </p>", ((static_cast<float>(1)/max_speed_level) * (1 - speedoffset) + speedoffset)*100); 
      
      if(speed_level>0){
        response->printf("<p>PWM derzeit <b>%.0f %% bei Geschwindig&shy;keits&shy;stufe %i</b>. ", ((static_cast<float>(speed_level)/max_speed_level) * (1 - speedoffset) + speedoffset)*100, speed_level); 
        response->printf("Das entspricht einem elektrischen Strom durch den Lokmotor, der bei etwa %.1f Volt flie&szlig;en w&uuml;rde.</p>",((static_cast<float>(speed_level)/max_speed_level) * (1 - speedoffset) + speedoffset)* motor_voltage_supply);  
      }
      else {
        response->printf("<p>PWM derzeit <b>0 %% bei Geschwindigkeitsstufe 0</b>. "); 
        response->printf("Es flie&szlig;t kein Strom durch den Lokmotor.</p>");  
      }
      response->printf("<p>&nbsp;</p>");   
      response->printf("<p> </p>");   
      response->printf("<table style=\"width:100%%\">");
      response->printf("<tr><th>STEU&shy;ER&shy;UNG</th><th>BE&shy;SCHLEU&shy;NIGUNG</th><th>BEDEUTUNG</th></tr>");  
      response->printf("<tr><td>Schiebe&shy;regler</td>  <td> +/- x Stufen</td>  <td>auf neue Geschwindigkeit einstellen</td></tr>");        
      response->printf("<tr><td>----</td>     <td>- 8 Stufen</td>            <td>bremsen </td></tr>");    
      response->printf("<tr><td> - </td>      <td>- 2 Stufen</td>            <td>etwas langsamer </td></tr>");    
      response->printf("<tr><td> + </td>      <td>+ 2 Stufen</td>            <td>etwas schneller </td></tr>");    
      response->printf("<tr><td>++++ </td>    <td>+ 8 Stufen</td>            <td>beschleunigen </td></tr>"); 
      response->printf("<tr><td> <&nbsp;> </td>    <td>0&nbsp;<&nbsp;></td>  <td>Stop und Richtung wechseln </td></tr>");    
      response->printf("<tr><td>Halten</td>   <td>- 32 Stufen</td>           <td>abbremsen bis zum Halt </td></tr>");    
      response->printf("<tr><td>Stopp!</td>   <td>00</td>                    <td>Schnellbremsung zum Notstopp</td></tr>"); 
      response->printf("<tr><td>Info?</td>    <td>?</td>                     <td>Status der Programmvariablen</td></tr>"); 
      response->printf("</table>");     
      response->printf("<p> </p>"); 
      response->printf("<p> </p>");   
      response->printf("<table style=\"width:100%%\">");
      response->printf("<tr><th>LED ANZEIGEN</th></tr>"); 
      response->printf("<tr><td>Programm&shy;schleife l&auml;uft</td>  <td>Oranger Blitz etwa alle 2 Skunden nach</td></tr>"); 
      response->printf("<tr><td> </td>  <td>%i Hauptprogrammschleifen</td></tr>", FLASH_FREQ);  
      response->printf("<tr><td> </td>  <td>%i mal Steuerbefehle beobachten</td></tr>", FLASH_FREQ/MONITOR_FREQ);
      response->printf("<tr><td> </td>  <td>bis zu %i mal Geschwindigkeit anpassen</td></tr>", FLASH_FREQ/speed_wait_loops);       
      response->printf("<tr><td>Info zur Richtung</td>  <td>blaue LED = vorw&auml;rts = 1</td></tr>"); 
      response->printf("<tr><td> </td>  <td>rote LED = r&uuml;ckw&auml;rts = 0</td></tr>"); 
      response->printf("<tr><td>Info zur Geschwin&shy;digkeit</td>  <td>0 bis 16 kurze LED-Blitze</td></tr>");
      response->printf("<tr><td>&Auml;nderung der Geschwin&shy;digkeit</td>  <td>blaue LED blinkt = Geschwindigkeit + 1</td></tr>");
      response->printf("<tr><td> </td>  <td>rote LED blinkt = Geschwindigkeit - 1</td></tr>"); 
      response->printf("<tr><td> </td>  <td> blaue & rote LEDs langes Blinken = Abbremsen, Geschwindigkeit = 0</td></tr>");
      response->printf("<tr><td>&Auml;nderung der Richtung</td>  <td>blau/rot/blau + langes blaues LED-Blinken = vorw&auml;rts = 1</td></tr>");
      response->printf("<tr><td> </td>  <td>rot/blau/rot + langes rotes LED-Blinken = r&uuml;ckw&auml;rts = 0</td></tr>");
      response->printf("<tr><td>Tonesignal&shy;erkennung (nur wenn zus&auml;tzlich mit Mikrofon ausger&uuml;stet)</td>  <td>rote LED, "); 
      response->printf("wenn Signal&shy;str&auml;rke in der einfachen FFT im Frequenz&shy;fenster > THRESHOLD1</td></tr>");
      response->printf("<tr><td> </td>  <td>blaue LED = aktives Tonsignal, wenn Signal&shy;str&auml;rke der gemitttelten FFTs im Frequenz&shy;fenster > THRESHOLD2</td></tr>");    
      response->printf("</table>");     
      response->printf("<p> </p>");      
      response->printf("<h2><a href='/monitor'>Info zu Programmvariablen</a></h2>");
      response->printf("<p> </p>");
      //response->printf("<h2><a href='http://%s'>Loksteuerung</a></h2>", WiFi.softAPIP().toString().c_str());
      response->printf("<div><a href='/'><img src=\"/lok.png\" alt='Loksteuerung' style='width:95%%'></a></div>");
      response->print("</body></html>");
      request->send(response);
      //request->send(SPIFFS, "/lok.png", "image/png");
  });

  // Route to load status info and instruction
  server.on("/access", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncResponseStream *response = request->beginResponseStream("text/html");
      response->print("<!DOCTYPE html><html><head><title>WLAN-Zugang</title>");
      response->print("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
      response->print("<style>p, td {font-size: 1.2em;} body{max-width: 1024px; font-size: 1.0rem; font-family: Helvetica; display: inline-block; padding:2%; margin:2% auto; text-align: left;}");
      response->print("table, tr {vertical-align: top; padding: 1%; border-collapse: collapse; border-bottom: 2pt solid #ddd;} tr:last-child { border-bottom: none; }"); 
      response->print("th, td {text-align: left; padding: 1%; } h1 {font-size: 1.4rem;} h2 {font-size: 1.2rem;}</style></head><body>");
      response->printf("<h1>WLAN-Zugang zur Loksteuerung</h1>");
      response->printf("<table style=\"width:100%%\">");
      response->printf("<tr><td>WLAN-Name (SSID) = </td>  <td>%s</td></tr>",ssid);        
      response->printf("<tr><td>Passwort = </td>          <td>%s</td></tr>",password);    
      response->printf("</table>"); 
      response->printf("<p>Link zur Loksteuerung: <a href='http://%s'>http://%s</a></p>",WiFi.softAPIP().toString().c_str(),WiFi.softAPIP().toString().c_str());
      response->printf("<a href='/'><img src=\"/qr_code.png\" alt='Startseite/Portal' style='width:100%%'></a>");
      response->printf("</body></html>");
      request->send(response);
  });

  // Route to load status info and instruction
  server.on("/monitor", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncResponseStream *response = request->beginResponseStream("text/html");
      Serial.println("Monitor called. Send status via speed_command '?' to adjustSpeed()");
      speed_command = '?'; //Send info on status from controller via serial monitor and indicate by flashing LEDs, executed via adjustSpeed(), additionally notifyClients() and JASON message (alternative to update via processor)
      response->print("<!DOCTYPE html><html><head><title>Programmvariablen</title>");
      response->print("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
      response->print("<style>body{max-width: 1024px; font-size: 1.0rem; font-family: Helvetica; display: inline-block; padding:2%; margin:2% auto; text-align: left;}");
      response->print("table, tr {vertical-align: top; padding: 1%; border-collapse: collapse; border-bottom: 2pt solid #ddd;} tr:last-child { border-bottom: none; }");
      response->print("th, td {font-size: 0.8rem; text-align: left; padding: 1%; } h1 {font-size: 1.8rem;} h2 {font-size: 1.2rem;}</style></head><body>");
      response->printf("<h1>Programmvariablen</h1>");
      response->printf("<p>Laufender arduino/c++ <b>SKETCH</b> ist: </p>");
      response->printf("<p>%s</p>", SKETCH_INFO.c_str());
      //response->printf("<p>WLAN-Nutzer (Stations connected to Access Point):%s</p>", WiFi.softAPgetStationNum()); //causes some problems with ESP32 power management
      response->printf("<p> </p>"); 
      response->printf("<table style=\"width:100%%\">");    
      response->printf("<tr><th>Hauptprogramm</th></tr>"); 
      response->printf("<tr><td>Programmschleife</td>  <td>%i von %i</td></tr>", LOOP_COUNT, FLASH_FREQ);  
      response->printf("<tr><td>Geschwindigkeit</td>  <td>Sturfe %i, Richtung %i, %s</td></tr>", speed_level, speed_direction, speed_direction ? "vorw&auml;rts" : "r&uuml;ckw&auml;rts");   
      response->printf("<tr><td>Geschwindigkeit</td>  <td>"); 
        for (int i = 1; i <= max_speed_level; ++i) {
          if (i <= speed_level) {
            if (speed_direction) {
              response->printf(">");
            }
            else {
              response->printf("<");
            }
          }
          else {
            response->printf("~");
          }
        }    
      response->printf("</td></tr>"); 
      response->printf("<tr><td>Fahrbefehl</td>  <td>%s</td></tr>",speed_command.c_str()); 
      #ifdef ESP32
        response->printf("<tr><td>ESP32 PIN f&uuml;r den Ber&uuml;hr&shy;ungs&shy;sensor</td>    <td>%i</td></tr>",input_switch);  
        response->printf("<tr><td>Ber&uuml;hrungssensor Eingabewert</td>    <td>%i</td></tr>",touchRead(input_switch));
        response->printf("<tr><td>Ber&uuml;hrungssensor Sensitivit&auml;t</td>    <td>&lt; %i</td></tr>",input_switch_level); 
        switch_ok = !(touchRead(input_switch) < input_switch_level);
        response->printf("<tr><td>Eingabe</td>  <td>%s</td></tr>",switch_ok ? "nicht aktiv" : "aktiv");    
      #else
        switch_ok = digitalRead(input_switch);
        response->printf("<tr><td>Eingabeschalter</td>  <td>%s</td></tr>",switch_ok ? "nicht aktiv" : "aktiv");  
      #endif
      
      #ifdef TLE5206
       response->printf("<tr><td>Motorstatus</td>  <td>%s</td></tr>",motor_ok ? "ok" : "!!! ST&Ouml;RUNG am MotorIC !!! Elektrische oder mechanische St&ouml;rung beseitigen und danach erneut freigeben?");
      #endif   
      response->printf("<tr><td>Tonsignal</td>  <td>%s</td></tr>",tone_signal ? "aktiv" : "nicht aktiv");
      response->printf("<tr><td>Ton-Programm-z&auml;hler </td>  <td>%i</td></tr>",TONE_LOOP_COUNT); 
      TONE_LOOP_COUNT =0; 
      response->printf("<tr><td>Ton-Aktivit&auml;ts-z&auml;hler </td>  <td>%i</td></tr>",TONE_ACTIVE_COUNT); 
      TONE_ACTIVE_COUNT =0;  
      response->printf("</table>");     
      response->printf("<p> </p>");      
      response->printf("<h2><a href='/monitor'>Aktualisieren: Sende '?'</a></h2>");
      response->printf("<h2><a href='/lok.ini'>Einstellung des Programms: lok.ini</a></h2>");
      response->printf("<div><a href='/'><img src=\"/lok.png\" alt='Loksteuerung' style='width:100%%'></a></div>");
      response->printf("<p> </p>");      
      response->printf("<h2 style=\"color: grey;\"><a href='ini'>Programmeinstellungen in der lok.ini Datei ver&auml;ndern</a></h2>");
      response->printf("<p> </p>");
      response->print("</body></html>");
      request->send(response);
      request->send(SPIFFS, "/lok.png", "image/png");
  });

  // Route to load status info and instruction
  server.on("/ini", HTTP_GET, [](AsyncWebServerRequest *request){
    String iniorig;
    Serial.println("+ .ini editor called. Opening lok.ini from SPIFFS. Saved new version will only be used at next microprocessor stratup");

    // open SPIFFS
    File file = SPIFFS.open("/lok.ini", "r");
    delay (10);
    if(!file){
      // File not found
      Serial.println("- Failed to open file lok.ini for reading!");
      return;
    } 
    else {
      Serial.println("+ lok.ini opened for reading.");
      while(file.available()){
        iniorig += String((char)file.read());
//        Serial.println( (char*) file.read());
      }
      Serial.println("iniorig = ");
      Serial.println(iniorig);
      file.close();
    }
    Serial.println("+ lok.ini closed after reading");
    AsyncResponseStream *response = request->beginResponseStream("text/html");

      // cange to textarea
      response->print("<!DOCTYPE html><html><head><title>Programmvariablen</title>");
      response->print("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
      response->print("<style>body{max-width: 1024px; font-size: 1.0rem; font-family: Helvetica; display: inline-block; padding:2%; margin:2% auto; text-align: left;}");
      response->print("table, tr {vertical-align: top; padding: 1%; border-collapse: collapse; border-bottom: 2pt solid #ddd;} tr:last-child { border-bottom: none; }");
      response->print("th, td {font-size: 0.8rem; text-align: left; padding: 1%; } h1 {font-size: 1.8rem;} h2 {font-size: 1.2rem;}</style></head><body>");
      response->printf("<h1>Programmeinstellungen</h1>");
//      response->printf("<form action=\"/get\">");
//      response->printf("input1: <input type=\"text\" name=\"input1\">");
//      response->printf("<input type=\"submit\" value=\"Submit\">");
//      response->printf("</form>"); 
      response->printf("  <form onSubmit=\"event.preventDefault(); formToJson(this);\">  "); 
//      response->printf("    <input type=\"text\" name=\"iniedit2\"/>  "); 
      response->printf("    <label>Programmeinstellungen in lok.ini anpassen:</label>  "); 
      response->printf("    <textarea style=\"font-size: 1.0rem; width:96%%;\" rows = \"20\" cols = \"40\" name = \"iniedit\"> %s </textarea><br>  ", iniorig.c_str());
      response->printf("    <input type=\"submit\" value=\"Submit\">  ");
      response->printf("  </form>  ");
      response->printf("<p> </p>");      
      response->printf("<h2 ><a href='/lok.ini'>Einstellung des Programms: lok.ini</a></h2>");
      response->printf("<div><a href='/'><img src=\"/lok.png\" alt='Loksteuerung' style='width:100%%'></a></div>");
      response->printf("<p> </p>");
      response->printf("</body></html>");
      response->printf("  <script language=\"JavaScript\">  ");
      response->printf("    function formToJson(form){  ");
      response->printf("      var iniedit3=form.iniedit.value;  ");
      response->printf("      var jsonIniEdit=JSON.stringify({i:iniedit3});  ");
      response->printf("      window.alert(jsonIniEdit);  ");
      response->printf("      websocket.send(jsonIniEdit);  ");
      response->printf("      console.log('sent iniedit to Server via WebSocket');  ");
//      response->printf("      var xhr  = new XMLHttpRequest()  ");
//      response->printf("      xmlHttp.open('PUT', '/put', jsonIniEdit);  ");  
//      response->printf("      xmlHttp.setRequestHeader('Content-Type', \"text/plain\");  ");  //(alternatively \"application/json\")
//      response->printf("      xmlHttp.send(null);  "); 
      response->printf("    }  ");
      response->printf("  </script>  ");     
      request->send(response);
      request->send(SPIFFS, "/lok.png", "image/png");
  });
/*
  // evaluate textarea input and write lok.ini to SPIFFS, with backup of former version
  // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
  server.on("/put", HTTP_PUT, [] (AsyncWebServerRequest *request) {},
    NULL,
    [](AsyncWebServerRequest * request, uint8_t *ininew, size_t len, size_t index, size_t total) {
    String iniwrite; 
    char * iniwrite2;
    Serial.println(); 
    Serial.println("Data received: ");
    iniwrite2 = (char*)ininew;
    Serial.println(iniwrite2); //echo received message for test and monitoring
    
    String iniwrite; 
    for(int i=0; i < len; i++) {
        Serial.print((char) ininew[i]);
        Serial.print(ininew[i]);
        iniwrite += (char) ininew[i]; 
    }
    Serial.println();
    Serial.println();
    request->send(200);

    File file = SPIFFS.open("/data/lok.ini", "w");
    if(!file){
      // File not found
      Serial.println("Failed to open file lok.ini for writing!");
      return;
    } 
    else {
      if(file.print(iniwrite)){
        Serial.println("- lok.ini file written");
      } 
      else {
        Serial.println("- lok.ini write failed!");
      }
      file.close();
    }

    
  });
*/  

  // Serve files in directory "/data/" when request url starts with "/"
  // Request to the root or none existing files will try to server the defualt
  // file name "index.htm" if exists
  // (!put just after specific request handlers and before CaptivePortal and NotFoundHandler)
  server.serveStatic("/", SPIFFS, "/");

  //Android captive portal. Maybe not needed. Might be handled by notFound handler.
  server.on ( "/generate_204", captivePortalTarget );

  //Microsoft captive portal. Maybe not needed. Might be handled by notFound handler.
  server.on ( "/fwlink", captivePortalTarget );
  
  // CaptivePortal redirection
  server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);//only when requested from AP

  //NotFound handler: Serve URI from SPIFFS or redirect to index.html
  server.onNotFound ( captivePortalTarget );

  // Start server
  server.begin();

#endif

  //finish startup
  digitalWrite(greenLED, LOW);
  digitalWrite(redLED, LOW);
  delay(500);
  Serial.print("Startup ");
  Serial.println(SKETCH_INFO);
  Serial.println();
  for (int i = 0; i < reset_reason;i++) {
    digitalWrite(greenLED, HIGH);
    delay(20);
    digitalWrite(greenLED, LOW);
    delay(480);
  }
}

////////////////////////////////////////////////////////////////////////////////
// MAIN LOOP
////////////////////////////////////////////////////////////////////////////////

void loop() {

#ifdef ESP32  
  // DNS Server and WebSocket management
  dnsServer.processNextRequest();
  ws.cleanupClients();
#endif
  
  // Indicate void loop i.e. program is running by frequent flashes
  ++LOOP_COUNT;
  if (LOOP_COUNT == FLASH_FREQ_DARK) {
    digitalWrite(POWER_LED_PIN, HIGH);
  }
  if (LOOP_COUNT >= FLASH_FREQ) {
    digitalWrite(POWER_LED_PIN, LOW);
    monitor_global_variables();
    LOOP_COUNT = 0;
  }

#ifdef ToneSampling
  //###### comment out tone sampling, will be needed for FFT tone signal analysis in PiedPiper
  // Calculate FFT if a full sample is available.
  if (samplingIsDone()) {
    // Run FFT on sample data.
    
    arm_cfft_radix4_instance_f32 fft_inst;
    arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);
    arm_cfft_radix4_f32(&fft_inst, samples);
    // Calculate magnitude of complex numbers output by the FFT.
    arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);
    
    
    // Detect tone sequence.
    toneLoop();

    // Restart audio sampling.
    samplingBegin();
  }
#endif

  // Parse any pending commands.
  parserLoop();

  if (LOOP_COUNT % MONITOR_FREQ == 0) {

    #if defined(ESP32) && defined(TLE5206) //motor IC TLE5206 offers a comprehensive error flag output
      if (!digitalRead(error_motoric)) { //TLE5206 error indicated if error_motoric = LOW
        motor_set(0);                   // emergency break
        speed_level = 0;
        motor_ok =0;
        IndicateMotorError();
      }
    #endif
    
    // evaluate manual switch digital reading (or any alternative digital HIGH/LOW signal) for MorseCode
    MorseCodeDecoder();
  }

  // use speed_command information for motor 1. speed adjustment and 2. motor PWM setting
  if (!speed_command.equals("")) {
    adjustSpeed();
  }

  // end of main loop
}

#ifdef ToneSampling
////////////////////////////////////////////////////////////////////////////////
// SPECTRUM DISPLAY FUNCTIONS (J Ruppert 2020, built on principles by Tony DiCola, Copyright 2013, MIT License)
//###### comment out tone sampling, will be needed for FFT tone signal analysis in PiedPiper
///////////////////////////////////////////////////////////////////////////////
  void toneLoop() {
    ++TONE_LOOP_COUNT;
    tone_array = tone_array.shift(-1);
    int lowBin = frequencyToBin(TONE_LOWS[tonePosition] - TONE_ERROR_MARGIN_HZ);
    int highBin = frequencyToBin(TONE_HIGHS[tonePosition] + TONE_ERROR_MARGIN_HZ);
    // Get the average intensity of frequencies inside and outside the tone window.
    float window, other;
    windowMean(magnitudes, lowBin, highBin, &window, &other);
    window = intensityDb(window);
    other = intensityDb(other);
    // Check if tone intensity is above the threshold to detect a step in the sequence.
    if ((window - other) >= TONE_THRESHOLD1_DB) {
      // Start timing the window if this is the first in the sequence.
      digitalWrite(redLED, HIGH);
      //toneDetected(); //disable for morse signal detection
      ++TONE_ACTIVE_COUNT;  //just for monitoring
      tone_array[0]= int(window-other);
        
    }
    else {
      digitalWrite(redLED, LOW);
      //tone_array[0] = int((window - other)/2);  // record value if below TONE_THRESHOLD1_DB; alternatively use half of TONE_THRESHOLD1_DB (or 0) for adjusting the sensitivity of the gliding average assessment
      tone_array[0] = int(TONE_THRESHOLD1_DB/2);  // record value if below TONE_THRESHOLD1_DB; alternatively use half of TONE_THRESHOLD1_DB (or 0) for adjusting the sensitivity of the gliding average assessment
    }
    //is the gliding average over time of multiple events, which are above TONE_THRESHOLD1_DB, also above TONE_THRESHOLD2_DB?
    if (tone_array.sum()/tone_array.size()>= TONE_THRESHOLD2_DB) { 
      tone_signal=true;
      digitalWrite(greenLED, HIGH);
  
      //monitor clarity of active tone signal, comment out for faster processing
      if (TONE_LOOP_COUNT % 8 == 0) {
         Serial.print  ("Average tone_array= ");
         Serial.print  (tone_array.sum()/tone_array.size());
         Serial.print  (" Tone_array: ");
         for (int j = 0; j < 8;++j) {
           Serial.print(tone_array[j]);
           Serial.print(", ");
         }     
      Serial.println(" ");
      }
     
    }
    else {
      tone_signal=false;
      digitalWrite(greenLED, LOW);
    }
  }
  
  // monitor successful toneSequenceDetection after sequence of tones (not used in version 030)
  void toneSequenceDetected() { //do not call, if uninterrupted main loop is required for morse signal detection
    // Flash the green LED multiple times, when the tone was detected.
    digitalWrite(redLED, HIGH);
    int pause = 50;
    for (int i = 0; i < 10; ++i) {
      digitalWrite(greenLED, HIGH);
      delay(pause);
      digitalWrite(greenLED, LOW);
      delay(pause);
    }
    digitalWrite(redLED, LOW);
  }
  
  
  ////////////////////////////////////////////////////////////////////////////////
  // SAMPLING FUNCTIONS (by Tony DiCola, Copyright 2013, MIT License)
  ////////////////////////////////////////////////////////////////////////////////
  // ###### comment out tone sampling, will be needed for FFT tone signal analysis in PiedPiper
  void samplingCallback() {
    // Read from the ADC and store the sample data
    samples[sampleCounter] = (float32_t)analogRead(AUDIO_INPUT_PIN);
    
    // disabled light level reading
    // light_level = analogRead(LIGHT_INPUT_PIN);
    
    // Complex FFT functions require a coefficient for the imaginary part of the input.
    // Since we only have real data, set this coefficient to zero.
    samples[sampleCounter + 1] = 0.0;
    // Update sample buffer position and stop after the buffer is filled
    sampleCounter += 2;
    if (sampleCounter >= FFT_SIZE * 2) {
      samplingTimer.end();
    }
  }
  
  void samplingBegin() {
    // Reset sample buffer position and start callback at necessary rate.
    sampleCounter = 0;
    samplingTimer.begin(samplingCallback, 1000000 / SAMPLE_RATE_HZ);
  }
  
  boolean samplingIsDone() {
    return sampleCounter >= FFT_SIZE * 2;
  }
#endif

////////////////////////////////////////////////////////////////////////////////
// Recording morse code commands active input signals (switch_signal, tone_signal)
////////////////////////////////////////////////////////////////////////////////

void MorseCodeDecoder() {
  //input signal active here by input_switch=LOW, i.e. switch_signal=!switch_switch
  //use alternative inputs (light detection, tone detection together with the switch_signal by logical OR combination '||'

#ifdef ESP32 
    // Touch input multiple sampling and averaging, necessary due to spikes in the touchRead, for preventing false positive switch_signals
    touch_array = touch_array.shift(-1);
    touch_array[0] = touchRead(input_switch); 
    switch_signal = (touch_array.sum()/touch_array.size() < input_switch_level);      //for robust ESP32 touch sensor reading
  
#else  // uP = Teensy and normally closed microswitch connected to input_switch
    switch_signal = !digitalRead(input_switch);         //for use with connected normally closed micro switch connected to HIGH and pull-down 10 kOhm resistor

#endif
  
  //light sampling analysis
  //light_signal =  (light_level < LIGHT_THRESHOLD);    // for disabling the assessment of light level readings
  light_signal = false;                                 // for disabling the assessment of light level readings
  
  if (switch_signal || tone_signal || light_signal) {  //Morse code can be transmitted via switch or tone or light
    if (SignalActive == false) {
      delay(5);// delay(button.debounce);
      // additional update status in case of bounce
      // button.currentState = digitalRead(button.pin);
      //reference to millis() function usage:
      //https://www.baldengineer.com/millis-tutorial.html
      //https://www.baldengineer.com/detect-short-long-button-press.html
      SignalTimer = millis();
      SignalActive = true;
      SignalTerminated = false;
    }
    SignalDuration = int (millis() - SignalTimer);
    if ((SignalDuration > dotMinLength) && (shortSignalActive == false)) {
      shortSignalActive = true;
    }
    if ((SignalDuration > dashMinLength) && (longSignalActive == false)) {
      longSignalActive = true;
    }
    if ((SignalDuration > 2*dashMinLength) && (longSignalActive == true)) {
      //2xlongSignal as additional signal for emergency brake, immediate activity
      SignalActive = false;
      longSignalActive = false;
      shortSignalActive = false;
      SignalTerminated = true;
      currentSignal="--";
      Serial.println("Morse signal: emergency brake by very long signal " + currentSignal);
      MORSE_CODE(currentSignal.c_str());
      currentSignal = "";
    }
  }
  //signal not active
  else {
    //evaluate length of last signal
    if (SignalActive) {
      //Serial.print("##### Monitor false positive SignalActive readings ##### millis() - SignalTimer = ");
      //Serial.println(millis() - SignalTimer);
      SignalTimer = millis();
      SignalActive = false;
      if (longSignalActive) {
        currentSignal += ("-");
        Serial.println("Recognized: " + currentSignal);
        longSignalActive = false;
        shortSignalActive = false;
      }
      if (shortSignalActive) {
        currentSignal += (".");
        Serial.println("Recognized: " + currentSignal);
        //longSignalActive = false;
        shortSignalActive = false;
      }
    }
    else {
      if (SignalTerminated == false) {
        SignalDuration = int (millis() - SignalTimer);
        if ((SignalDuration > TerminalLength)) {
          SignalTerminated = true; //command completed
          Serial.println("Morse signal: " + currentSignal);
          // translate Morse code in currentSignal to speed_command; input to MORSE_CODE as: const char *
          MORSE_CODE(currentSignal.c_str());
          currentSignal = "";
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Parse MORSE_CODE and translation to speed_command, sizeof(*morsecode)
////////////////////////////////////////////////////////////////////////////////

void MORSE_CODE (const char* command) {
  for (unsigned int i = 0 ; i < morsecode_size ; ++i) {
    if (strcmp(command, morsecode[i]) == 0) {
      Serial.println();
      Serial.print("GOT MORSE CODE: ");
      Serial.print(morsecode[i]);
      speed_command = commands[i]; // do not use 'String' in front of speed_command when passing command in subroutine and in if loop, as this will define a new and !non global vairable, which becomes unavailable outside this if loop!
      Serial.print(" | SPEED COMMAND: ");
      Serial.print(speed_command);
      Serial.print(" | ACTIVITY: ");
      Serial.println(meaning[i]);
      speed_wait_countdown = 0;    // don't wait but immedeatly (countdown value = 0) start adjusting the motor speed
      client_message = speed_command;
      #ifdef ESP32
        notifyClients();
      #endif  
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Parse CLIENT COMMAND and translation to speed_command, sizeof(*morsecode)
////////////////////////////////////////////////////////////////////////////////
#ifdef ESP32 
void CLIENT_COMMAND (const char* command) {  //const char * commandstrpointer = command.c_str();
      const char * str = "=";

      if (strncmp(command, str, 1)==0) { // compare first character
        int new_speed_level = atoi(&command[1]); // convert string to integer after first character
        int difference = new_speed_level - speed_level;
        speed_command = "";   
        if (difference <0) {
          for (int i = 0; i > difference; --i) {
            speed_command = speed_command + "-";
          }
        }
        if (difference >0) {
          for (int i = 0; i < difference; ++i) {
            speed_command = speed_command + "+";
          }
        }
        Serial.print("GOT WIFI WEB CLIENT COMMAND");
        Serial.print(" | SPEED COMMAND: ");
        Serial.print(speed_command);
        Serial.print(" | ACTIVITY: ");
        Serial.print("adjust speed by ");
        Serial.println(difference);
        speed_wait_countdown = 0;    // don't wait but immedeatly (countdown value = 0) start adjusting the motor speed
      }
      else {
        speed_command = command;
        //Serial.println();
        //Serial.println("=== MONITOR POINT 4 ===");
        Serial.print("GOT WIFI WEB CLIENT COMMAND");
        Serial.print(" | SPEED COMMAND: ");
        Serial.print(speed_command);
        speed_wait_countdown = 0;    // don't wait but immedeatly (countdown value = 0) start adjusting the motor speed
      }   
  notifyClients();
}
#endif

////////////////////////////////////////////////////////////////////////////////
// MOTOR SPEED ADJUSTMENT
////////////////////////////////////////////////////////////////////////////////

void adjustSpeed() {
  // Serial.print("Speed_command_received. Speed_wait_countdown = "); //### for monitoring ###
  // Serial.println(speed_wait_countdown);                            //### for monitoring ###
  if (speed_wait_countdown <= 0) {
    Serial.print("Adjust speed: ");
    Serial.print(speed_command.charAt(0)); //evaluate first speed_command character
    Serial.print(" | ");

    //evaluate first speed_command character here: speed_command.charAt(0)
    switch (char(speed_command.charAt(0))) {
      case '0':        //fast brake, speed_level =0
        speed_level = 0;
        digitalWrite(redLED, HIGH);
        digitalWrite(greenLED, HIGH);
        Serial.print("fast brake, speed level set to 0 | ");
        delay(1000);
        break;
      case '-':        //decrease speed
        --speed_level;
        if (speed_level < 0) {
          speed_level = 0;
          break;
        }
        digitalWrite(redLED, HIGH);
        Serial.print("speed level decreased - | ");
        delay(200);
        break;
      case '+':        //increase speed
        ++speed_level;
        if (speed_level > max_speed_level) {
          speed_level = max_speed_level;
          break;
        }
        digitalWrite(greenLED, HIGH);
        Serial.print("speed level increased + | ");
        delay(200);
        break;
      case '<':        //reverse direction
        speed_direction = !speed_direction;
        if (speed_direction) {
          digitalWrite(greenLED, HIGH);
          delay (200);
          digitalWrite(greenLED, LOW);
          digitalWrite(redLED, HIGH);
          delay (200);
          digitalWrite(redLED, LOW);
          digitalWrite(greenLED, HIGH);
          delay (200);
          digitalWrite(greenLED, LOW);
          delay (200);
          digitalWrite(greenLED, HIGH);
          delay (600);
          Serial.print("direction reversed forward >>> | ");
        }
        else {
          digitalWrite(redLED, HIGH);
          delay (200);
          digitalWrite(redLED, LOW);
          digitalWrite(greenLED, HIGH);
          delay (200);
          digitalWrite(greenLED, LOW);
          digitalWrite(redLED, HIGH);
          delay (200);
          digitalWrite(redLED, LOW);
          delay (200);
          digitalWrite(redLED, HIGH);
          delay (600);
          Serial.println("direction reversed backward <<< | ");
        }
        break;
      case '?':        //question
        #ifdef ESP32
          notifyClients();  // Send current speed_level information via Wifi WebSocket to clients, additional early call for updating index.html
        #endif
        InfoText(); //print info text to serial monitor
                
        //indicate motor direction by green and red LED long flash
        if (speed_direction) {
          digitalWrite(greenLED, HIGH);
          delay(1500);
          digitalWrite(greenLED, LOW);
        }
        else {
          digitalWrite(redLED, HIGH);
          delay(1500);
          digitalWrite(redLED, LOW);
        }
        delay(100);

        //indicate motor speed by green or red LED short flashes
        for (int i = 0; i < speed_level/2; ++i) {
          delay(480);
          if (speed_direction) {
            digitalWrite(greenLED, HIGH);
            delay(20);
            digitalWrite(greenLED, LOW);
          }
          if (!speed_direction) {
            digitalWrite(redLED, HIGH);
            delay(20);
            digitalWrite(redLED, LOW);
          }
        }    
        delay (1000);
        Serial.println();
        break;
      default:
        // if nothing else matches, do the default
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, HIGH);
        delay (200);
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, LOW);
        delay (200);
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, HIGH);
        delay (200);
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, LOW);
        delay (200);
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, HIGH);
        delay (200);
        Serial.println("unknown command ??? no change of speed");
        break;
    } // end of switch case

    //switch_ok = digitalRead(input_switch);  // (no condition and value not used after version 029)
    //motor_ok = digitalRead(error_motoric);  // (no condition and value not used after version 029)
    
    // forward new speed_level to motor set
    motor_set(speed_level);

    #ifdef ESP32
      notifyClients();  // Send current speed_level information via Wifi WebSocket to clients
    #endif
    
    // prepare for next round of speed_command
    speed_wait_countdown = speed_wait_loops;
    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, LOW);
    speed_command.remove(0, 1); //ersase first charcter of speed_command, i.e. the command, which was just executed

  } //end of speed wait countdown loop
  --speed_wait_countdown;
}

////////////////////////////////////////////////////////////////////////////////
// Info Text Layout
////////////////////////////////////////////////////////////////////////////////

void InfoText(){
  Serial.println();
  Serial.println();
  Serial.print("Running arduino/c++ sketch is: ");
  Serial.println(SKETCH_INFO); //print sketch name, license, author
  Serial.println();
  
  //Serial.print morse code information
  Serial.println("Morse code  | speed adjustment                   | meaning ");
  for (int i = 0; i < morsecode_size ; ++i) {
    Serial.print(morsecode[i]);
    int l=(10 - strlen(morsecode[i]));
    for (int i=0; i < l; i++) {
      Serial.print(' ');
    }
    Serial.print("  |  ");
    Serial.print(commands[i]);
    l=(max_speed_level - strlen(commands[i]));
    for (int i=0; i < l; i++) {
      Serial.print(' ');
    }
    Serial.print("  |  ");
    Serial.println(meaning[i]);
  }
  Serial.println();

  //indicate motor speed level by green and red LED very short flashes
  Serial.println("LED INDICATORS"); 
  Serial.println("Information on motor direction:   forward = 1 = green LED; backward = 0 = red LED"); 
  Serial.println("Information on motor speed_level: 0 to 16 short LED flashes");
  Serial.println("Adjustment of motor speed:        green LED flash = speed level + 1; red LED flash = speed level - 1; green & red LEDs long flash = brake, speed level = 0");
  Serial.println("Adjustment of motor direction:    green/red/green + green LED flash = forward = 1;  red/green/red + red LED flash = backward = 0;");
  Serial.println("Tone signal detected:             Single FFT with frequency window > THRESHOLD1: red LED; averaged FFTs frequency windows > THRESHOLD2: green LED = tone signal active");  
  Serial.print  ("Program loop frequencies:         orange flash after ");     
  Serial.print  (FLASH_FREQ);
  Serial.print  (" main loop cycles, "); 
  Serial.print  (FLASH_FREQ/MONITOR_FREQ);
  Serial.print  (" times monitoring for input activity, "); 
  Serial.print  (FLASH_FREQ/speed_wait_loops);
  Serial.println(" steps for speed adjustment");
  Serial.print  ("Motor speed setting:              ");     
  Serial.print  (max_speed_level);
  Serial.print  (" speed levels, PWM at lowest speed level "); 
  Serial.print  (speedoffset);
  Serial.print  (" , current PWM: "); 
  if (speed_level > 0){
  Serial.print  ((static_cast<float>(speed_level)/max_speed_level) * (1 - speedoffset) + speedoffset);
  }
  Serial.print  (" @ "); 
  Serial.print  (speed_level); 
  Serial.println(" speed levels");
}

////////////////////////////////////////////////////////////////////////////////
// MOTOR SET: set motor speed level through PWM output
////////////////////////////////////////////////////////////////////////////////

void motor_set (int v) {
  #if defined(ESP32) && defined(TLE5206)
      if (v > 0 && v <= max_speed_level) {  // speed level 1 to max_speed_level (default=32), not stopped, motor running
                                            //for TLE5206 and start with exact speedoffset at speed_level 1 use: ledcWrite(pwmChannel2, (1 - static_cast<float>(v)/max_speed_level) * (1-speedoffset) * 256);
        if (speed_direction) { //forward direction
          ledcWrite(pwmChannel1, 255);  // for setting motor1 = HIGH      
          ledcWrite(pwmChannel2, (1 - static_cast<float>(v)/max_speed_level) * (1-speedoffset) * 256);   // for setting motor 2 to PWM settinig for TLE5206 H-Bridge IC motor driver
          Serial.print("motor set: speed_level ");
          Serial.print(v);
          Serial.print(" forward | PWM analog write motor1 = 255, motor2 = ");
          Serial.println( (1 - static_cast<float>(v)/max_speed_level) * (1-speedoffset) * 256);     
        }
        else { //backward direction
          ledcWrite(pwmChannel1, (1 - static_cast<float>(v)/max_speed_level) * (1-speedoffset) * 256);   // for setting motor 2 to PWM settinig for TLE5206 H-Bridge IC motor driver
          ledcWrite(pwmChannel2, 255);  // for setting motor2 = HIGH      
          Serial.print("motor set: speed level ");
          Serial.print(v);
          Serial.print(" backward | PWM analog write motor2 = 255, motor1 = ");
          Serial.println( (1 - static_cast<float>(v)/max_speed_level) * (1-speedoffset) * 256);          
        }
      }
      else { // speed level 0 or out of range 1 to max_speed_level: stop motor
        ledcWrite(pwmChannel1, 0);  // for setting motor 1 = LOW
        ledcWrite(pwmChannel2, 0);  // for setting motor 2 = LOW
        Serial.print("motor set: ============== STOP ============== | ");
        Serial.print(v);
        Serial.print(" | PWM analog write motor1&2 = LOW, PWM = ");
        Serial.println(0);    
    }

  
  #elif defined(ESP32) && !defined(TLE5206) // for ESP32 in combination with L293D Motor IC
      if (v > 0 && v <= max_speed_level) {  // speed level 1 to max_speed_level (default=32), not stopped, motor running
                                            //for L293D and start with exact speedoffset at speed_level 1 use: ledcWrite(pwmChannel1, ((static_cast<float>(v)/max_speed_level) * (1 - speedoffset) + speedoffset)* 256);
        if (speed_direction) { //forward direction
          digitalWrite(motor1, HIGH);
          digitalWrite(motor2, LOW);          
          ledcWrite(pwmChannel1, ((static_cast<float>(v)/max_speed_level) * (1 - speedoffset) + speedoffset)* 256);   // PWM settinig for L293D H-Bridge IC motor driver
          Serial.print("motor set: speed_level ");
          Serial.print(v);
          Serial.print(" forward | PWM analog write motor1 = HIGH, motor2 = LOW, PWM = ");
          Serial.println(((static_cast<float>(v)/max_speed_level) * (1 - speedoffset) + speedoffset)* 256);     
        }
        else { //backward direction
          digitalWrite(motor1, LOW);
          digitalWrite(motor2, HIGH);   
          ledcWrite(pwmChannel1, ((static_cast<float>(v)/max_speed_level) * (1 - speedoffset) + speedoffset)* 256);   // PWM settinig for L293D H-Bridge IC motor driver
          Serial.print("motor set: speed level ");
          Serial.print(v);
          Serial.print(" backward | PWM analog write motor1 = LOW, motor2 = HIGH, PWM = ");
          Serial.println(((static_cast<float>(v)/max_speed_level) * (1 - speedoffset) + speedoffset)* 256);          
        }
      }
      else { // speed level 0 or out of range 1 to max_speed_level: stop motor
        digitalWrite(motor1, LOW);
        digitalWrite(motor2, LOW);
        ledcWrite(pwmChannel1, 0);  
        Serial.print("motor set: ============== STOP ============== | ");
        Serial.print(v);
        Serial.print(" | PWM analog write motor1&2 = LOW, PWM = ");
        Serial.println(0);    
    }
    
  #else  // mP= default e.g. Teensy 4.0 or Arduino assumed to be in combination with TLE5206 motor IC
    if (v > 0 && v <= max_speed_level) {  // speed level 1 to max_speed_level (default=32), not stopped, motor running
                                          //for start with exact speedoffset at speed_level 1 use: analogWrite(motor2, (max_speed_level - v) * (1 - speedoffset) * 256/max_speed_level);
                                          //approximate and for maximum of 16 speed levels use:   analogWrite(motor2,(16-v)*(1-speedoffset)*16); 
      if (speed_direction) { //forward direction
        analogWrite(motor1, 255);
        analogWrite(motor2, (max_speed_level - v) * (1 - speedoffset) * 256/max_speed_level);   // Teensy 4.0 PWM settinig for TLE5206S H-Bridge IC motor driver
        Serial.print("motor set: speed_level ");
        Serial.print(v);
        Serial.print(" forward | PWM analog write motor2 = ");
        Serial.println((max_speed_level - v) * (1 - speedoffset) * 256/max_speed_level);
      }
      else { //backward direction
        analogWrite(motor1, (max_speed_level - v) * (1 - speedoffset) * 256/max_speed_level);   // Teensy 4.0 PWM settinig for TLE5206S H-Bridge IC motor driver
        analogWrite(motor2, 255);
        Serial.print("motor set: speed level ");
        Serial.print(v);
        Serial.print(" backward | PWM analog write motor1 = ");
        Serial.println(((max_speed_level - v) * (1 - speedoffset) * 256/max_speed_level));
      }
    }
    else { // speed level 0 or out of range 1 to max_speed_level: stop motor
      analogWrite(motor1, 0);
      analogWrite(motor2, 0);
      Serial.print("motor set: ============== STOP ============== | ");
      Serial.print(v);
      Serial.print(" | PWM analog write motor1&2 = ");
      Serial.println(0);
    }
  #endif
}

////////////////////////////////////////////////////////////////////////////////
// Indicate Motor Error in main loop
////////////////////////////////////////////////////////////////////////////////

void IndicateMotorError() {
  Serial.println();
  Serial.println("============================================");
  Serial.println("=== TLE5206 motor IC error indicated !!! ===");
  Serial.println("============================================");
  Serial.println();
  #ifdef ESP32
    client_message = "!!! Störung am MotorIC !!!";
    notifyClients();
  #endif
  for (int i = 0; i < 50; i++) { //indicate motor error by fast flashing of red and green LED for 10 seconds
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, HIGH);
    delay(100);
    digitalWrite(redLED, LOW);
    digitalWrite(greenLED, LOW);
    delay(100);
  }
  while (!(touchRead(input_switch) < input_switch_level)) { //wait for touch signal at startup in order to alliviate start-up brown-out cycle
    Serial.print("Waiting for restart signal at GPIO pin ");
    Serial.print(input_switch);
    Serial.print(" TOUCH read= ");
    Serial.println(touchRead(input_switch)); 
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, HIGH);
    delay(950);
    digitalWrite(redLED, LOW);
    digitalWrite(greenLED, LOW);
    delay(50);
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, HIGH);
    delay(950);
    digitalWrite(redLED, LOW);
    digitalWrite(greenLED, LOW);
    delay(50);
    #ifdef ESP32
      client_message = "!!! Störung behoben?"; // Wenn ja, Freigabe über den TouchRead pin geben // if yes, release via TouchRead pin
      notifyClients();
    #endif  
  } 
  motor_ok=1; //manual re-setting, but will come back by !error_motoric in main loop, if electrical error is not resolved         
}

////////////////////////////////////////////////////////////////////////////////
// Monitoring global variables in main loop
////////////////////////////////////////////////////////////////////////////////

void monitor_global_variables() {
  Serial.print("Loop count: ");
  Serial.print(LOOP_COUNT);
  Serial.print("! Restart at 0.");
  Serial.print(" Speed_level: ");
  Serial.print(speed_level);
  Serial.print(" Direction: ");
  Serial.print(speed_direction);
  Serial.print(" | ");
  for (int i = 1; i <= max_speed_level; ++i) {
    if (i <= speed_level) {
      if (speed_direction) {
        Serial.print(">");
      }
      else {
        Serial.print("<");
      }
    }
    else {
      Serial.print("~");
    }
  }
  Serial.print(" | ");
  Serial.print("Speed_command: " + speed_command);
  //Serial.println("commandBuffer="+commandBuffer+"| ");
  
  #ifdef ESP32
    switch_ok = !(touchRead(input_switch) < input_switch_level);
  #else
    switch_ok = digitalRead(input_switch);
  #endif

  #ifdef TLE5206
   motor_ok = digitalRead(error_motoric);
  #endif
  
  Serial.print(" | switch_ok= ");
  Serial.print(switch_ok);
  Serial.print(" | motor_ok= ");
  Serial.print(motor_ok);
  
  //disabled light_level reading
  //Serial.print(analogRead(LIGHT_INPUT_PIN));  
  //int light_level = analogRead(LIGHT_INPUT_PIN);  
  //Serial.print(" | light_level= ");
  //Serial.print(light_level);  
  //Serial.print(" | light_signal= ");
  //Serial.print(light_signal);  

  #ifdef ESP32
    Serial.print(" | GPIO pin ");
    Serial.print(input_switch);
    Serial.print(" TOUCH read= ");
    Serial.print(touchRead(input_switch));
  #endif

  #ifdef ToneSampling
    Serial.print(" | tone_signal= ");
    Serial.print(tone_signal);
    Serial.print(" | Tone_loop_count= ");
    Serial.print(TONE_LOOP_COUNT);
    TONE_LOOP_COUNT =0; 
    Serial.print(" | Tone_signal_activ= ");
    Serial.print(TONE_ACTIVE_COUNT);
    TONE_ACTIVE_COUNT =0;
  #endif

  Serial.print(" | SignalActive= ");
  Serial.print(SignalActive);     
  Serial.println( " | send '?' for info");

  /*
  Serial.print("##### Monitor Point Signal Active 1 #####");
  Serial.print(" SignalActive: ");
  Serial.print(SignalActive);
  Serial.print(" , SignalTerminated: ");
  Serial.print(SignalTerminated);
  Serial.print(" , SignalDuration: ");
  Serial.print(SignalDuration);
  Serial.print(" , currentSignal: ");
  Serial.print(currentSignal);
  Serial.print(" , TerminationLengt: ");
  Serial.print(TerminalLength);
  Serial.println();
  */
}

#ifdef ToneSampling
////////////////////////////////////////////////////////////////////////////////
// UTILITY FUNCTIONS (by Tony DiCola, Copyright 2013, MIT License)
//###### comment out tone sampling, will be needed for FFT tone signal analysis in PiedPiper
////////////////////////////////////////////////////////////////////////////////
  
  // Compute the average magnitude of a target frequency window vs. all other frequencies.
  void windowMean(float* magnitudes, int lowBin, int highBin, float* windowMean, float* otherMean) {
    *windowMean = 0;
    *otherMean = 0;
    // Notice the first magnitude bin is skipped because it represents the
    // average power of the signal.
    for (int i = 1+TONE_FREQ_OFFSET; i < FFT_SIZE / 2; ++i) { // TONE_FREQ_OFFSET in order to avoid high loads of low frequency noise in peak detection of window vs. other 
      if (i >= lowBin && i <= highBin) {
        *windowMean += magnitudes[i];
      }
      else {
        *otherMean += magnitudes[i];
      }
    }
    *windowMean /= (highBin - lowBin) + 1;
    *otherMean /= (FFT_SIZE / 2 - (highBin - lowBin) - TONE_FREQ_OFFSET);
  }
  
  // Convert a frequency to the appropriate FFT bin it will fall within.
  int frequencyToBin(float frequency) {
    float binFrequency = float(SAMPLE_RATE_HZ) / float(FFT_SIZE);
    return int(frequency / binFrequency);
  }
  
  // Convert intensity to decibels
  float intensityDb(float intensity) {
    return 20.0 * log10(intensity);
  }
#endif

////////////////////////////////////////////////////////////////////////////////
// COMMAND PARSING FUNCTIONS (by Tony DiCola, Copyright 2013, MIT License, amended J. Ruppert 2020)
// These functions allow parsing simple commands input on the serial port.
// Commands allow reading and writing variables that control the device.
// The code also allows interaction with a Python program for Spectrogram display by Tony DiCola
// See instruction and download at http://learn.adafruit.com/fft-fun-with-fourier-transforms/
//
// The code interpretes commands defined in morsecode[] and evaluated in the function MORSE_CODE to a speed_command
// Example morse code commands are:
// ...
// .---
// ---
//
// Direct commands must end with a semicolon character.
//
// Example commands are:
// GET SAMPLE_RATE_HZ;
// - Get the sample rate of the device.
// SET SAMPLE_RATE_HZ 400;
// - Set the sample rate of the device to 400 hertz.
//
// Written Morse Code commands just end with a return after the last dash or dot
// Send '?' character for more information.
//
////////////////////////////////////////////////////////////////////////////////

void parserLoop() {
  // Process any incoming characters from the serial port
  while (Serial.available() > 0) {
    char c = Serial.read();
    // Add any characters that aren't the end of a command (semicolon) to the input buffer.
    if (c != ';') {
      c = toupper(c);
      strncat(commandBuffer, &c, 1);
    }
    else if (c == ';') {  //criterion changed in order to allow additional MORSE_CODE parsing
      parseCommand(commandBuffer); // Parse the command because an end of command token was encountered.
      memset(commandBuffer, 0, sizeof(commandBuffer)); // Clear the input buffer
    }
  }
  // Parse MORSE_CODE commands, which are not terminated with ';' character
  // detect availability of commandBuffer characters by first element in command buffer not equal to 0
  if (commandBuffer[0] != 0) {
    MORSE_CODE(commandBuffer);
    // Clear the input buffer
    memset(commandBuffer, 0, sizeof(commandBuffer));
  }
}

#ifdef ToneSampling
  //###### comment out tone sampling, will be needed for FFT tone signal analysis in PiedPiper
  // Macro used in parseCommand function to simplify parsing get and set commands for a variable
  #define GET_AND_SET(variableName) \
    else if (strcmp(command, "GET " #variableName) == 0) { \
      Serial.println(variableName); \
    } \
    else if (strstr(command, "SET " #variableName " ") != NULL) { \
      variableName = (typeof(variableName)) atof(command+(sizeof("SET " #variableName " ")-1)); \
    }
#endif

void parseCommand(char* command) {

#ifdef ToneSampling
  //###### comment out tone sampling, will be needed for FFT tone signal analysis in PiedPiper
  if (strcmp(command, "GET MAGNITUDES") == 0) {
    for (int i = 0; i < FFT_SIZE; ++i) {
      Serial.println(magnitudes[i]);
    }
  }
  else if (strcmp(command, "GET SAMPLES") == 0) {
    for (int i = 0; i < FFT_SIZE * 2; i += 2) {
      Serial.println(samples[i]);
    }
  }
  else if (strcmp(command, "GET FFT_SIZE") == 0) {
    Serial.println(FFT_SIZE);
  }
  
  GET_AND_SET(SAMPLE_RATE_HZ)
  GET_AND_SET(TONE_ERROR_MARGIN_HZ)
  GET_AND_SET(TONE_WINDOW_MS)
  GET_AND_SET(TONE_THRESHOLD1_DB)
  GET_AND_SET(TONE_THRESHOLD2_DB)
#endif

}
