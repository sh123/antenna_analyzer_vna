/**
 *  Panoramic antenna analyzer based on si5341 clock 
 *    generator and pcd8544 Nokia 5110 display
 *
 **/
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <AD9850.h>
#include <Rotary.h>
#include <SimpleTimer.h>
#include "Wire.h"

#define BANDS_CNT          12

// AD9850 pins
#define PIN_GEN_RESET     2
#define PIN_GEN_CLK       8
#define PIN_GEN_UD        9
#define PIN_GEN_DATA      10

// analog read pins
#define PIN_SWR_FWD        0
#define PIN_SWR_RFL        1

// rotary encoder pins
#define PIN_ROTARY_CLK     11
#define PIN_ROTARY_DATA    12
#define PIN_ROTARY_BTN     13

// screen pins
#define PIN_PCD_CLK        7
#define PIN_PCD_DIN        6
#define PIN_PCD_DC         5
#define PIN_PCD_CS         4
#define PIN_PCD_RST        3

#define SWR_MAX            32
#define SWR_LIST_SIZE      84
#define SWR_SCREEN_HEIGHT  48
#define SWR_SCREEN_CHAR    8
#define SWR_GRAPH_HEIGHT   (SWR_SCREEN_HEIGHT - SWR_SCREEN_CHAR)
#define SWR_GRAPH_CROP     6

#define FREQ_STEP_INC      25000ULL
#define FREQ_STEP_MAX      1000000ULL
#define FREQ_MAX           75000000ULL
#define FREQ_DELAY_MS      5

#define TO_KHZ(freq)       (freq / 1000)
#define VALID_RANGE(freq)  (freq < FREQ_MAX)

enum MAIN_SCREEN_STATE {
  S_MAIN_SCREEN = 0,
  S_GRAPH_MANUAL,
  S_GRAPH_AUTOMATIC,
  S_SETTINGS
};

struct band_map_t {
  uint32_t freq;
  uint32_t freq_step;
  char *band_name;
} const g_bands[BANDS_CNT] PROGMEM = {
  {   1800000ULL,  10000ULL, "TOP" },
  {   3500000ULL,  10000ULL, "80m" },
  {   5350000ULL,  20000ULL, "60m" },
  {   7100000ULL,  20000ULL, "40m" },
  {  10110000ULL,  25000ULL, "30m" },
  {  14100000ULL,  25000ULL, "20m" },
  {  18100000ULL,  25000ULL, "17m" },
  {  21070000ULL,  25000ULL, "15m" },
  {  24900000ULL,  25000ULL, "12m" },
  {  27000000ULL,  50000ULL, "11m" },
  {  28100000ULL,  50000ULL, "10m" },
  {  50100000ULL, 100000ULL, "6m " }
};

// band state
int g_active_band_index = 0;
struct band_map_t g_active_band;

// swr state
long g_freq_min;
double g_swr_min;
unsigned char g_swr_list[SWR_LIST_SIZE];

// UI state
bool g_do_update = true;
MAIN_SCREEN_STATE g_screen_state = S_MAIN_SCREEN;

// peripherals
SimpleTimer g_timer;
Rotary g_rotary = Rotary(PIN_ROTARY_CLK, PIN_ROTARY_DATA, PIN_ROTARY_BTN);
Adafruit_PCD8544 g_display = Adafruit_PCD8544(PIN_PCD_CLK, PIN_PCD_DIN, 
                                              PIN_PCD_DC, PIN_PCD_CS, 
                                              PIN_PCD_RST);

/* --------------------------------------------------------------------------*/

void setup()
{
  Serial.begin(9600);

  analogReference(EXTERNAL) ;
  analogRead(PIN_SWR_FWD);
  analogRead(PIN_SWR_RFL);
 
  generator_initialize();

  swr_list_clear(); 
  band_select(g_active_band_index);

  g_timer.setInterval(500, process_display_swr);
  g_timer.setInterval(100, process_rotary_button);
  //attachInterrupt(digitalPinToInterrupt(PIN_ROTARY_BTN), process_rotary_button, CHANGE);
  g_timer.setInterval(1, process_rotary);
  //attachInterrupt(digitalPinToInterrupt(PIN_ROTARY_CLK), process_rotary, CHANGE);

  g_display.begin();
  g_display.setContrast(60);
  g_display.display();
  delay(500);
  
  g_display.clearDisplay();
  g_display.display();
}

/* --------------------------------------------------------------------------*/

void generator_initialize()
{
  DDS.begin(PIN_GEN_CLK, PIN_GEN_UD, PIN_GEN_DATA, PIN_GEN_RESET);
}

void generator_set_frequency(uint64_t freq)
{
  DDS.setfreq(freq, 0);
  delay(FREQ_DELAY_MS);
}

/* --------------------------------------------------------------------------*/

void swr_list_clear() 
{
  for (int i = 0; i < SWR_LIST_SIZE; i++) {
    g_swr_list[i] = 0;
  }
}

void swr_list_shift_right() 
{
  g_swr_list[0] = 0;
  
  for (int i = SWR_LIST_SIZE - 1; i != 0; i--) {
    g_swr_list[i + 1] = g_swr_list[i];
  }
}

void swr_list_shift_left() 
{
  g_swr_list[SWR_LIST_SIZE - 1] = 0;
  
  for (int i = 0; i < SWR_LIST_SIZE - 1; i++) {
    g_swr_list[i] = g_swr_list[i + 1];
  }
}

void swr_list_store_center(double swr) 
{
  g_swr_list[SWR_LIST_SIZE / 2] = (unsigned char)swr_screen_normalize(swr);
}

void swr_list_draw() 
{  
  for (int i = 0; i < SWR_LIST_SIZE; i++) {
    
    if (g_swr_list[i] != 0) {
      
      g_display.drawFastVLine(i, SWR_SCREEN_HEIGHT - g_swr_list[i] + SWR_GRAPH_CROP, g_swr_list[i] - SWR_GRAPH_CROP, BLACK);
      
      process_rotary();
      process_rotary_button();
    }
  } // i
}

void swr_list_sweep_and_fill() 
{
  uint64_t freq_hz = g_active_band.freq - g_active_band.freq_step * SWR_LIST_SIZE / 2;

  double swr = SWR_MAX;
    
  for (int i = 0; i < SWR_LIST_SIZE; i++) {

    if (VALID_RANGE(freq_hz)) {
      generator_set_frequency(freq_hz);
      
      process_rotary();
      process_rotary_button();

      swr = swr_read();
      swr_update_minimum_swr(swr, TO_KHZ(freq_hz));
    }
    
    g_swr_list[i] = (unsigned char)swr_screen_normalize(swr);
    
    freq_hz += g_active_band.freq_step;
    
  } // over swr list

  generator_set_frequency(g_active_band.freq);
}

void swr_list_grid_draw() 
{
  g_display.drawFastVLine(SWR_LIST_SIZE / 2, SWR_SCREEN_CHAR, SWR_SCREEN_CHAR / 2, BLACK);

  for (unsigned char x = 0; x <= SWR_LIST_SIZE; x += SWR_LIST_SIZE / 12) {
    
    for (unsigned char y = SWR_SCREEN_CHAR; y <= SWR_GRAPH_HEIGHT + SWR_GRAPH_CROP; y += SWR_GRAPH_HEIGHT / SWR_GRAPH_CROP) {

      g_display.drawPixel(x + 6, y + SWR_SCREEN_CHAR - 1, BLACK);

    } // y
    
  } // x
}

unsigned int swr_screen_normalize(double swr)
{  
  unsigned int swr_norm = swr * (double)SWR_GRAPH_HEIGHT / (double)SWR_GRAPH_CROP;
  
  if (swr_norm > SWR_GRAPH_HEIGHT) {
    swr_norm = SWR_GRAPH_HEIGHT;
  }
  return swr_norm;
}

void swr_update_minimum_swr(double swr, long freq_khz)
{
  if (swr < g_swr_min) {
    g_swr_min = swr;
    g_freq_min = freq_khz;
  }
}

double swr_read()
{
  int val_fwd = analogRead(PIN_SWR_FWD);
  int val_rfl = analogRead(PIN_SWR_RFL);
  return swr_calculate(val_fwd, val_rfl);
}

double swr_calculate(int fwd, int rfl) 
{
  int val_fwd = fwd;
  int val_rfl = rfl;
  
  if (val_rfl > val_fwd) {
    val_rfl = val_fwd;
  }
  
  // NOTE, no imaginary reactive part
  double gamma = (double)val_rfl / (double)val_fwd;
  
  double swr = (1 + gamma) / (1 - gamma);
  
  if (swr > SWR_MAX || isnan(swr)) {
    swr = SWR_MAX;
  }
  return swr;
}

/* --------------------------------------------------------------------------*/

void band_select_next() 
{
  g_active_band_index += 1;
  
  if (g_active_band_index >= BANDS_CNT) {
    g_active_band_index = 0;
  }
  
  band_select(g_active_band_index);
}

void band_select(int index) 
{
  if (index < BANDS_CNT) {
    
    memcpy_PF((void*)&g_active_band, (uint_farptr_t)&g_bands[index], sizeof(g_bands[index]));
    
    swr_list_clear();
    
    g_swr_min = SWR_MAX;
    g_freq_min = g_active_band.freq / 1000ULL;

    generator_set_frequency(g_active_band.freq);
  }
}

void band_rotate_frequency(int dir)
{
  g_active_band.freq += dir * g_active_band.freq_step;

  if (g_active_band.freq > FREQ_MAX) {
    g_active_band.freq = FREQ_MAX;
  }
  
  generator_set_frequency(g_active_band.freq);
}

void band_rotate_step(int dir)
{
  g_active_band.freq_step += dir * FREQ_STEP_INC;

  if (g_active_band.freq_step > FREQ_STEP_MAX) { 
    if (dir < 0)
      g_active_band.freq_step = 0;
    if (dir > 0)
      g_active_band.freq_step = FREQ_STEP_MAX;
  }
}

/* --------------------------------------------------------------------------*/

void screen_select_next() 
{
  switch (g_screen_state) {

    case S_MAIN_SCREEN:
      g_screen_state = S_GRAPH_MANUAL;
      break;

    case S_GRAPH_MANUAL:
      g_screen_state = S_GRAPH_AUTOMATIC;
      swr_list_sweep_and_fill();
      break;

    case S_GRAPH_AUTOMATIC:
      g_screen_state = S_SETTINGS;
      break;

    case S_SETTINGS:
      g_screen_state = S_MAIN_SCREEN;
      break;

     default:
       break;
       
  } // current screen state
}

/* --------------------------------------------------------------------------*/

void process_rotary() 
{
  unsigned char rotary_state = g_rotary.process();
  
  if (rotary_state) {

    int dir = (rotary_state == DIR_CW) ? -1 : 1;

    switch (g_screen_state) {

      case S_GRAPH_AUTOMATIC:
        band_rotate_frequency(dir);
        break;

      case S_MAIN_SCREEN:
      case S_GRAPH_MANUAL:
        band_rotate_frequency(dir);
        if (rotary_state == DIR_CW) 
          swr_list_shift_right();
        else 
          swr_list_shift_left();
        break;

      case S_SETTINGS:
        band_rotate_step(dir);
        break;
        
    } // screen state

    g_do_update = true;
    
  } // rotary changed
}

void process_rotary_button() 
{
  unsigned char rotary_btn_state = g_rotary.process_button();

  switch (rotary_btn_state) {

    case BTN_RELEASED:
    
      switch (g_screen_state) {
        
        case S_SETTINGS:
          break;
          
        default:
          band_select_next(); 
          g_do_update = true;
          break;
          
      } // screen state
      break;

    case BTN_PRESSED_LONG:
      screen_select_next();
      g_do_update = true;
      break;

    default:
      break;
      
  } // button state
}

void process_display_swr() 
{
  long freq_khz = TO_KHZ(g_active_band.freq);
    
  int val_fwd = analogRead(PIN_SWR_FWD);
  int val_rfl = analogRead(PIN_SWR_RFL);
  double swr = swr_calculate(val_fwd, val_rfl);

  swr_list_store_center(swr);
  swr_update_minimum_swr(swr, freq_khz);

  g_display.clearDisplay();
  g_display.setTextSize(1);
  g_display.setTextColor(BLACK);
  g_display.setCursor(0,0);

  switch (g_screen_state) {

    case S_MAIN_SCREEN:

      /*
      Serial.print(freq_khz); Serial.print(F(" "));
      Serial.print(val_fwd); Serial.print(F(" "));
      Serial.print(val_rfl); Serial.print(F(" "));
      Serial.print(swr); Serial.println(F(""));
      */
      
      g_display.print(g_active_band.band_name); g_display.print(F(": ")); 
      g_display.print(freq_khz); g_display.println(F(" k"));
      g_display.print(F("SWR: ")); g_display.println(swr);
      g_display.print(F("FWD: ")); g_display.println(val_fwd);
      g_display.print(F("RFL: ")); g_display.println(val_rfl);
      g_display.println(F("MIN:"));
      g_display.print(g_swr_min); g_display.print(F(" ")); g_display.println(g_freq_min);

      break;

    case S_GRAPH_AUTOMATIC:
    
      g_display.print(F("A "));
      swr_list_sweep_and_fill();

    case S_GRAPH_MANUAL:

      g_display.print(freq_khz);
      g_display.print(F(" "));
      g_display.println(swr);

      swr_list_grid_draw();
      swr_list_draw();

      break;

    case S_SETTINGS:

      g_display.print(F("STEP: ")); 
      g_display.print((long)(g_active_band.freq_step/1000UL));
      g_display.print(F(" kHz"));

      break;

  } // screen state

  g_display.display();
}

/* --------------------------------------------------------------------------*/

void loop()
{
  g_timer.run();
  
  if (g_do_update) {
    
    process_display_swr();
    g_do_update = false;
  }
}

/* --------------------------------------------------------------------------*/
