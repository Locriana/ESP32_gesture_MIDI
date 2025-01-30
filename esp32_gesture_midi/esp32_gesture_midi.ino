#include <freertos/FreeRTOS.h>

#include <Adafruit_GFX.h>
#include "FreeSansBold6pt7b.h"
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/Picopixel.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

#include <Wire.h>
#include "RevEng_PAJ7620.h"


#define MIDI_STATUS_NOTE_OFF        0x80
#define MIDI_STATUS_NOTE_ON         0x90
#define MIDI_STATUS_POLY_AFTERTOUCH 0xA0
#define MIDI_STATUS_CONTROL_CHANGE  0xB0
#define MIDI_STATUS_PROGRAM_CHANGE  0xC0
#define MIDI_STATUS_CH_AFTERTOUCH   0xD0
#define MIDI_STATUS_PITCH_WHEEL     0xE0



//these are not the standard/optimal pins
//and they may not work @ full speed
//however... they work okay for MIDI data rate of 31250 bit/s
#define MIDI_RX_PIN     32
#define MIDI_TX_PIN     33

//is E pin available in the matrix?
#define HUB75E          0

#define ANALOG_IN       36
#define SW_NUMBER       3
const uint8_t sw_pin[3]     {35,34,39};

uint8_t globalClipSel = 0;
uint8_t globalScene = 0;
const uint8_t GLOBAL_CLIP_SEL_MAX = 4;

enum GlobalMode_e {
  MODE_NOTE = 0,
  MODE_CLIP = 1,
  MODE_SCENE = 2
};

GlobalMode_e GlobalMode = MODE_NOTE;

//here we define the display interface pins
//please remember about the E pin for larger HUB75 displays
//(A-E are for selecting scan areas)
enum HubPins {
  R1_PIN = 25,
  G1_PIN = 26,
  B1_PIN = 27,
  R2_PIN = 14,
  G2_PIN = 12,
  B2_PIN = 13,
  A_PIN = 23,   //21 when using the ad hoc prototype
  B_PIN = 19,
  C_PIN = 5,
  D_PIN = 17,
  #if HUB75E
    E_PIN = 18,
  #else
    E_PIN = -1,
  #endif
  LAT_PIN = 4,
  OE_PIN = 15,
  CLK_PIN = 16
};

#if HUB75E
  #define DISP_RES_X 128    // width in pixels of each panel module. 
  #define DISP_RES_Y 64     // height in pixels of each panel module.
#else
  #define DISP_RES_X 64    // width in pixels of each panel module. 
  #define DISP_RES_Y 32     // height in pixels of each panel module.
#endif

#define DISP_CHAIN 1      // number of HUB75 panels chained one to another

#define MC_CONTROL_CHANNEL  16


MatrixPanel_I2S_DMA *dma_display = nullptr;
HardwareSerial MidiSerial(2);

RevEng_PAJ7620 gestureSensor = RevEng_PAJ7620();


//Convert RGB888 to RGB565 which is used by the library
//It gets converted anyway inside the libs,
//but this is just to keep thing compatible
inline uint16_t disp_color(uint8_t r, uint8_t g, uint8_t b)
{
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

bool dispUpdFlag = true;
String dispLineGesture = "G ";
String dispLineFilter = "F ";

void updateDisplay(){
  if(dispUpdFlag==true){
    dma_display->clearScreen();
    dma_display->setFont(&Picopixel);
    dma_display->setCursor(4,5);
    dma_display->setTextColor(disp_color(0x80, 0x80, 0x80));
    switch(GlobalMode){
      case MODE_NOTE:
        dma_display->print("MODE: NOTE");
        break;
      case MODE_CLIP:
        dma_display->print("MODE: CLIP");
        //dma_display->setFont(&FreeSansBold6pt7b);
        dma_display->setFont(&FreeSans9pt7b);
        dma_display->setCursor(32,22);
        dma_display->setTextColor(disp_color(0x80, 0x80, 0x00));
        dma_display->print("C"+String(globalClipSel+1));
        break;
      case MODE_SCENE:
        dma_display->print("MODE: SCENE");
        //dma_display->setFont(&FreeSansBold6pt7b);
        dma_display->setFont(&FreeSans9pt7b);
        dma_display->setCursor(32,22);  //14
        dma_display->setTextColor(disp_color(0x00, 0x40, 0xA0));
        dma_display->print("S"+String(globalScene+1));
        break;
    }
    dma_display->setFont(&FreeSansBold6pt7b);
    dma_display->setCursor(1,14);
    dma_display->setTextColor(disp_color(0x80, 0x00, 0x80));
    dma_display->print(dispLineGesture);
    dma_display->setCursor(1,26);
    dma_display->setTextColor(disp_color(0x00, 0x80, 0x80));
    dma_display->print(dispLineFilter);
    dispUpdFlag = false;
  }
}

//an "unspecified" color for anything unassigned
#define DEFAULT_COLOR   disp_color(0x50,0x20,0x20)

/*
mapping MC101/707 pads to midi notes, indices go like:
 1  3  5  7  9 11 13 15
 0  2  4  6  8 10 12 14
 so look at the map from left:
*/
const uint8_t mc_pad_map[16]{
  36,37,
  38,39,
  41,42,
  45,46,
  48,49,
  62,51,
  63,54,
  64,56
};



//either I don't know how to use the PAJ sensor...
//...or the other functions than up/down/left/right seem ureliable
const uint8_t gesture_to_note_map[] = {
  0, //unused, just for alignment
  63, //up
  60, //down
  67, //left
  68, //right
  0, //forward - unused
  0, //backward - unused
  0, //clockwise - unused
  0 //ccw - unused
/*  38, //up - snare
  36, //down - kick
  63, //left - hiq
  49, //right - cymbal crash
  56, //forward - exp
  64, //backward - rev cym
  48, //clockwise - tom h
  41 //ccw - tom l*/
};

const char * gesture_to_text[] = {
  "--", //unused, just for alignment
  "Up ", //up - snare
  "Dn ", //down - kick
  "Lt ", //left - hiq
  "Rt ", //right - cymbal crash
  "Fw ", //forward - exp
  "Bk ", //backward - rev cym
  "Cw ", //clockwise - tom h
  "Cc " //ccw - tom l
};



uint32_t color565to888(uint16_t col){
  uint32_t res = 0;
  uint32_t b = (col & 0x001F) << 3;
  uint32_t g = (col & 0x07E0) << 4;
  uint32_t r = (col & 0x00F800) << 8;
  res = r | g | b;
  return res;
}


//modify to suit your taste :)
void show_splash_screen(void){
  dma_display->clearScreen();
  dma_display->setFont(&Picopixel);
  dma_display->setCursor(4,4);
  dma_display->setTextColor(disp_color(0x80, 0x80, 0x80));
  dma_display->print("MIDI Ctrler");
  vTaskDelay(2000);
}


//a general-purpose hex-dump function for simple serial port debugging
void dump(void* mem, uint16_t len){
  Serial.printf("dump of address %08X, len=%d, contents:\n",(unsigned int)mem, len);
  uint8_t* membyteptr = (uint8_t*)mem;
  for(int i=0; i<len; i++){
    Serial.printf(" %02X",membyteptr[i]);
  }
  Serial.printf("\ndump in text: ");
  for(int i=0; i<len; i++){
    char c = membyteptr[i];
    if(isprint(c)){
      Serial.printf("%c",c);
    }
  }
  Serial.printf("\n");
}

const uint8_t GESTURES_NB = 10;
static int16_t timeouts[GESTURES_NB];

void setup(){
  MidiSerial.begin(31250,SERIAL_8N1,MIDI_RX_PIN,MIDI_TX_PIN,false,10);
  Serial.begin(115200);

  //config buttons
  for(int i=0;i<SW_NUMBER;i++){
    pinMode(sw_pin[i], INPUT_PULLUP);
  }
  

  Serial.printf("Gesture MIDI ctrler");

  if( !gestureSensor.begin() ){
    Serial.printf("gesture sensor init error\n");
  }



  HUB75_I2S_CFG::i2s_pins _pins={R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN, A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, LAT_PIN, OE_PIN, CLK_PIN};
  HUB75_I2S_CFG mxconfig(
    DISP_RES_X,   // module width
    DISP_RES_Y,   // module height
    DISP_CHAIN,    // chain length (how many modules are connected in chain)
    _pins // pin mapping
  );
  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  dma_display->setBrightness8(80); //0-255
  dma_display->clearScreen();
  Serial.printf("splash screen...\n");
  show_splash_screen();
  Serial.printf("setup exit\n");
}

#define MIDI_DEFAULT_CHANNEL 1
#define MIDI_DEFAULT_VELOCITY 100

void midi_tx_status(uint8_t ch, uint8_t status, uint8_t data1, uint8_t data2) {
  if(ch<1) return;
  if(ch>16) return;
  ch -= 1;
  ch &= 0x0F;
  status &=0xF0;
  MidiSerial.write(status | ch);//send status & channel information
  MidiSerial.write(data1);
  MidiSerial.write(data2);
}

void midi_tx_program_change(uint8_t ch, uint8_t status, uint8_t data) {
  if(ch<1) return;
  if(ch>16) return;
  ch -= 1;
  ch &= 0x0F;
  status &=0xF0;
  MidiSerial.write(status | ch);
  MidiSerial.write(data);
}


//wrapper for midi_tx_status but with default values
void midi_note_on(uint8_t note){
  if(note==0) return;
  midi_tx_status(MIDI_DEFAULT_CHANNEL,MIDI_STATUS_NOTE_ON,note,MIDI_DEFAULT_VELOCITY);
  Serial.printf("Note ON: %02d\n",note);
}

void midi_note_off(uint8_t note){
  if(note==0) return;
  midi_tx_status(MIDI_DEFAULT_CHANNEL,MIDI_STATUS_NOTE_OFF,note,MIDI_DEFAULT_VELOCITY);
  Serial.printf("Note OFF: %02d\n",note);
}

void midi_all_notes_off(uint8_t ch=MIDI_DEFAULT_CHANNEL){
  midi_tx_status(MIDI_DEFAULT_CHANNEL,MIDI_STATUS_CONTROL_CHANGE,0x7B,0x00);
}

void midi_tx_clip_change(uint8_t channel, uint8_t clip){
  if(clip > 15) return;
  Serial.printf("TX PC, clip=%d\n",clip);
  midi_tx_program_change(channel,MIDI_STATUS_PROGRAM_CHANGE,clip);
  dispUpdFlag=true;
}

void midi_tx_scene_change(uint8_t scene){
  scene &= 0x7F;
  Serial.printf("TX PC, scene=%d\n",scene);
  midi_tx_program_change(MC_CONTROL_CHANNEL, MIDI_STATUS_PROGRAM_CHANGE, scene);
  dispUpdFlag=true;
}

void disp_gesture(const char* gesture_name){
  Serial.printf("disp gesture name: %s\n",gesture_name);
  dispLineGesture = "G " + String(gesture_name);
  dispUpdFlag = true;
}

void disp_filter(uint8_t setting){
  dispLineFilter = "F " + String(setting);
}

void filter_process(void){
  static int fltTimeout;
  if(fltTimeout){
    fltTimeout--;
  }
  else{
    fltTimeout = 100;
    uint16_t fltRaw = analogRead(ANALOG_IN);
    Serial.printf("fltRaw: %05d\n",fltRaw);
    uint16_t flt = fltRaw / 100;
    if(fltRaw > 350){
      //practical range of a directly connected sharp 2Y0A02 sensor: 8 (far) to 30 (near)
      const int min = 8;
      const int max = 35;
      if(flt < min) flt = min;
      if(flt > max) flt = max;
      int fltSetting = ((max - flt) * 128 / (max-min)) & 0x7F;
      if(fltSetting > 0){
        dispLineFilter = "F " + String(fltSetting);
        midi_tx_status(2,0xB0,80,(uint8_t)fltSetting);
      }
    }
    else{
      dispLineFilter = "F --";
      //midi_tx_status(2,0xB0,80,(uint8_t)64);
    }
    dispUpdFlag=true;
  }

}

void mode_process(void){
  for(int i=0;i<SW_NUMBER;i++){
    if(digitalRead(sw_pin[i])==LOW){
      Serial.printf("sw %i is low\n",i);
      GlobalMode=(GlobalMode_e)i;
      //if changing mode to clip, reset the energy level
      switch(GlobalMode){
        case MODE_CLIP:
          globalClipSel= 0;
          midi_tx_clip_change(1, 0);
          break;
        case MODE_SCENE:
          globalScene = 0;
          midi_tx_scene_change(0);
          break;
      }
      dispUpdFlag=true;
    }
  }
}

void gesture_process(void){
  Gesture gesture;                  // Gesture is an enum type from RevEng_PAJ7620.h
  gesture = gestureSensor.readGesture();   // Read back current gesture (if any) of type Gesture

  //for debug only
  #if 0
    Serial.printf("Tos: ");
    for(int i=0;i<GESTURES_NB;i++){
      Serial.printf("%d ",timeouts[i]);
    }
    Serial.printf("\n");
  #endif

  switch (GlobalMode) {

    case MODE_SCENE:{
      switch(gesture){
        case GES_UP:
          globalScene = 0;
          midi_tx_scene_change(0);
          break;
        case GES_DOWN:
          globalScene = 1;
          midi_tx_scene_change(1);
          break;
        case GES_LEFT:
          globalScene = 2;
          midi_tx_scene_change(2);
          break;
        case GES_RIGHT:
          globalScene = 3;
          midi_tx_scene_change(3);
          break;
      }
      break;
    }

    case MODE_CLIP:{
      switch(gesture){
        case GES_UP:
        case GES_RIGHT:
        {
          if(globalClipSel < GLOBAL_CLIP_SEL_MAX){
            globalClipSel++;
          }
          else{
            globalClipSel = 0;
          }
          midi_tx_clip_change(1,globalClipSel);
          break;
        }
        case GES_DOWN:{
          globalClipSel = 0;
          midi_tx_clip_change(1,globalClipSel);
          break;
        }
      }
      break;
    }

    case MODE_NOTE:{
      const int16_t GESTURE_DEFAULT_TIMEOUT = 200;
      if(gesture > GES_NONE){
        timeouts[gesture] = GESTURE_DEFAULT_TIMEOUT;
        midi_note_on(gesture_to_note_map[gesture]);
        disp_gesture(gesture_to_text[gesture]);
      }
      else{
        for(int i=0;i<GESTURES_NB;i++){
          if(timeouts[i]>0){
            timeouts[i]--;
            if(timeouts[i]==0){
              Serial.printf("g %02d timeout exp\n",i);
              midi_note_off(gesture_to_note_map[i]);
              disp_gesture(gesture_to_text[GES_NONE]);
            }
          }//if(timeouts[i]){
        }//for(int i=0;i<GESTURES_NB;i++){
      }
      break;
    }
  }


  if( gesture != GES_NONE )
  {
    Serial.printf(", Code: %02X \n",gesture);
  }
}



uint16_t global_timeout;

void loop(){
  int8_t ch;
  int8_t note;

  if(global_timeout){
    global_timeout--;
  }
  else{
    midi_all_notes_off();
    global_timeout = 4000;
  }

  mode_process();
  gesture_process();
  filter_process();

  updateDisplay();

  vTaskDelay(1);
}
