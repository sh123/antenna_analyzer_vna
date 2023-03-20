#ifndef MAIN_H
#define MAIN_H

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <AD9850.h>
#include <Rotary.h>
#include <SimpleTimer.h>
#include <EEPROM.h>
#include "Wire.h"

/* --------------------------------------------------------------------------*/

//#define DEBUG_SERIAL
//#define USE_SMITH_CHART

#define BANDS_CNT         12

// AD9850 pins
#define PIN_GEN_RESET     2
#define PIN_GEN_CLK       8
#define PIN_GEN_UD        9
#define PIN_GEN_DATA      10

// analog read pins
#define PIN_SWR_AMP       0
#define PIN_SWR_PHS       1

// rotary encoder pins
#define PIN_ROTARY_CLK    A4
#define PIN_ROTARY_DATA   A3
#define PIN_ROTARY_BTN    A2

// screen pins
#define PIN_PCD_CLK       7
#define PIN_PCD_DIN       6
#define PIN_PCD_DC        5
#define PIN_PCD_CS        4
#define PIN_PCD_RST       3

// EEPROM addresses
#define EEPROM_CONTRAST   0

// swr related parameters
#define SWR_MAX           9999

#define SWR_SCREEN_CONTRAST 60
#define SWR_SCREEN_HEIGHT 48
#define SWR_SCREEN_WIDTH  84

#define SWR_SCREEN_CHAR   8
#define SWR_LIST_SIZE     SWR_SCREEN_WIDTH
#define SWR_GRAPH_HEIGHT  (SWR_SCREEN_HEIGHT - SWR_SCREEN_CHAR)

#define SWR_MAX_VALUE     4
#define SWR_Z_MAX_OHM     100
#define SWR_Z_MIN_OHM     0

#define SWR_GRID_COUNT_X  12
#define SWR_GRID_STEP_X   SWR_LIST_SIZE / SWR_GRID_COUNT_X
#define SWR_GRID_PAD_X    ((SWR_LIST_SIZE - 1) % SWR_GRID_STEP_X) / 2

#define SWR_GRID_COUNT_Y  SWR_MAX_VALUE
#define SWR_GRID_STEP_Y   SWR_GRAPH_HEIGHT / SWR_GRID_COUNT_Y
#define SWR_GRID_SMITH    5

// generator related
#define FREQ_STEP_INC     5000
#define FREQ_STEP_MAX     1000000
#define FREQ_MAX          75000000
#define FREQ_DELAY_MS     5

// adc converter
#define ADC_ITER_CNT      16
#define ADC_DB_RES        60.0 / 1024.0
#define ADC_DB_CENTER     1024 / 2
#define ADC_DB_OFFSET     (-30.0)
#define ADC_DEG_RES       180.0 / 1024.0

// utils
#define DEG_TO_RAD(deg)   (deg * 3.14159 / 180.0)
#define TO_KHZ(freq)      (freq / 1000)
#define VALID_RANGE(freq) (freq < FREQ_MAX)

enum MAIN_SCREEN_STATE 
{
  S_MAIN_SCREEN = 0,
  S_GRAPH_SWR,
  S_GRAPH_SWR_AUTO,
  S_GRAPH_Z,
  S_GRAPH_Z_AUTO,
  S_GRAPH_SMITH,
  S_GRAPH_SMITH_AUTO,
  S_SETTINGS
};

enum SETTINGS_SCREEN_STATE
{
  S_SETTINGS_STEP = 0,
  S_SETTINGS_CONTRAST,
  S_SETTINGS_CAL_50OHM,
  S_SETTINGS_CAL_OPEN,
  S_SETTINGS_CAL_SHORT,
  S_SETTINGS_CAL_DEFAULT
};

// HF band related data
struct band_t 
{
  uint32_t freq;
  uint32_t freq_step;
  
  char *band_name;

  // TODO, automatic calibration through settings
  
  int16_t adc_amp_cal_open;
  int16_t adc_amp_cal_50ohm;
  
  int16_t adc_phs_cal_open;
  int16_t adc_phs_cal_short;
};

// one swr measurement point
struct measurement_t 
{  
  uint32_t freq_khz;

  // read from adc
  int16_t amp;      // amplitude
  int16_t phs;      // phase

  // adjusted by calibration
  int16_t amp_adj;
  int16_t phs_adj;
  
  float rl_db;    // return loss, S11
  float phi_deg;  // phase shift angle
  
  float rho;
  
  float rs;       // real impedance part
  float xs;       // complex impedance part
  
  float swr;
  float z;        // impedance vector length
};

void reflectometer_initialize();

void generator_initialize();
void generator_set_frequency(uint32_t freq);

void screen_initialize();
void screen_rotate_contrast(int8_t dir);
void screen_load_settings();
void screen_save_settings();

void swr_list_clear();
void swr_list_shift_right();
void swr_list_shift_left();
void swr_list_store_center(int amp, int phs);
void swr_list_grid_draw();

#ifdef USE_SMITH_CHART
int circle_circle_intersection(float x0, float y0, float r0,
                               float x1, float y1, float r1,
                               float *xi, float *yi,
                               float *xi_prime, float *yi_prime)
void swr_list_smith_grid_draw();
void swr_smith_pt_from_z(float rs, float xs, uint8_t &x, uint8_t &y);
void swr_list_smith_draw();
#endif

uint8_t swr_screen_normalize(float swr);
uint8_t swr_screen_z_normalize(float z);
void swr_list_draw();
void swr_list_z_draw();
void swr_list_sweep_and_fill();

void swr_update_minimum_swr(float swr, uint32_t freq_khz);
uint16_t swr_phs_cal_adjust(uint16_t phs);
uint16_t swr_amp_cal_adjust(uint16_t amp);
void swr_measure();
void swr_calculate();
void swr_print_info();

void band_select_next();
void band_select(uint8_t index);
void band_rotate_frequency(int8_t dir);
void band_rotate_step(int8_t dir);

void settings_draw();
void settings_select_next_screen();
void settings_select_prev_screen();
void settings_rotate_screen(int8_t dir);
void settings_save();
void settings_rotate(int8_t dir);
void settings_process_button();
void screen_select_next();

void process_rotary();
void process_rotary_button();
void process_display_swr();

#endif // MAIN_H