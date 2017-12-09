/**
 *  Panoramic antenna analyzer based on AD9850 clock 
 *  generator, AD8302 phase/amplitude detector and 
 *  pcd8544 Nokia 5110 display
 **/
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

/* --------------------------------------------------------------------------*/

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

/* --------------------------------------------------------------------------*/

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

/* --------------------------------------------------------------------------*/

// band state
int16_t g_active_band_index = 0;
struct band_t g_active_band;
const struct band_t g_bands[BANDS_CNT] PROGMEM = {
  {   1800000,  10000, "TOP", 782, 550, 345, 1013 },
  {   3500000,  10000, "80m", 808, 490, 120, 1010 },
  {   5350000,  20000, "60m", 812, 495, 70,  1002 },
  {   7100000,  20000, "40m", 814, 495, 51,  995 },
  {  10110000,  25000, "30m", 814, 497, 42,  985 },
  {  14100000,  25000, "20m", 814, 517, 38,  949 },
  {  18100000,  25000, "17m", 814, 531, 44,  918 },
  {  21070000,  25000, "15m", 814, 537, 50,  933 },
  {  24900000,  25000, "12m", 800, 547, 55,  910 },
  {  27000000,  50000, "11m", 811, 568, 60,  904 },
  {  28100000,  50000, "10m", 811, 573, 60,  898 },
  {  50100000, 100000, "6m ", 800, 550, 170, 800 }
};

// swr measurement state
float g_swr_min;
uint32_t g_freq_min;
struct measurement_t g_pt;
int16_t g_amp_list[SWR_LIST_SIZE];
int16_t g_phs_list[SWR_LIST_SIZE];
#ifdef USE_SMITH_CHART
const float g_smith_grid[SWR_GRID_SMITH] = {0.2, 0.5, 1.0, 2.0, 5.0};
#endif

// UI state
uint8_t g_contrast = SWR_SCREEN_CONTRAST;
bool g_do_update = true;
MAIN_SCREEN_STATE g_screen_state = S_MAIN_SCREEN;

// UI state, settings
bool g_settings_selected = false;
SETTINGS_SCREEN_STATE g_settings_screen_state = S_SETTINGS_STEP;

// peripherals
SimpleTimer g_timer;
Rotary g_rotary = Rotary(PIN_ROTARY_CLK, PIN_ROTARY_DATA, PIN_ROTARY_BTN);
Adafruit_PCD8544 g_disp = Adafruit_PCD8544(PIN_PCD_CLK, PIN_PCD_DIN, 
                                              PIN_PCD_DC, PIN_PCD_CS, 
                                              PIN_PCD_RST);

/* --------------------------------------------------------------------------*/

void setup()
{
#ifdef DEBUG_SERIAL
  Serial.begin(9600);
#endif

  reflectometer_initialize();
  generator_initialize();

  swr_list_clear(); 
  band_select(g_active_band_index);

  // periodic execution
  g_timer.setInterval(500, process_display_swr);
  g_timer.setInterval(100, process_rotary_button);
  g_timer.setInterval(1, process_rotary);
  
  //attachInterrupt(digitalPinToInterrupt(PIN_ROTARY_BTN), process_rotary_button, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(PIN_ROTARY_CLK), process_rotary, CHANGE);

  screen_initialize();
}

/* --------------------------------------------------------------------------*/

void reflectometer_initialize()
{  
  // from AD8302 gain/phase detector
  analogReference(EXTERNAL) ;
  analogRead(PIN_SWR_AMP);
  analogRead(PIN_SWR_PHS);
}

/* --------------------------------------------------------------------------*/

void generator_initialize()
{
  DDS.begin(PIN_GEN_CLK, PIN_GEN_UD, PIN_GEN_DATA, PIN_GEN_RESET);
}

void generator_set_frequency(uint32_t freq)
{
  DDS.setfreq(freq, 0);
  delay(FREQ_DELAY_MS);
}

/* --------------------------------------------------------------------------*/

void screen_initialize()
{  
  g_disp.begin();
  screen_load_settings();
  g_disp.display();
}

void screen_rotate_contrast(int8_t dir)
{
  g_contrast += dir;
  g_disp.setContrast(g_contrast);
}

void screen_load_settings()
{  
  EEPROM.get(EEPROM_CONTRAST, g_contrast);
  if (g_contrast == 0 || g_contrast == 255)
  {
    g_contrast = SWR_SCREEN_CONTRAST;
    EEPROM.put(EEPROM_CONTRAST, g_contrast);
  }
  g_disp.setContrast(g_contrast);  
}

void screen_save_settings()
{
  uint8_t contrast;
  EEPROM.get(EEPROM_CONTRAST, contrast);
  
  if (contrast != g_contrast) 
  {
    EEPROM.put(EEPROM_CONTRAST, g_contrast);
  }
}

/* --------------------------------------------------------------------------*/

void swr_list_clear() 
{
  for (uint8_t i = 0; i < SWR_LIST_SIZE; i++) 
  {
    g_amp_list[i] = 0;
    g_phs_list[i] = 0;
  }
}

void swr_list_shift_right() 
{
  for (uint8_t i = SWR_LIST_SIZE - 2; i != 0; i--) 
  {
    g_amp_list[i + 1] = g_amp_list[i];
    g_phs_list[i + 1] = g_phs_list[i];
  }
}

void swr_list_shift_left() 
{
  for (uint8_t i = 0; i < SWR_LIST_SIZE - 2; i++) 
  {
    g_amp_list[i] = g_amp_list[i + 1];
    g_phs_list[i] = g_phs_list[i + 1];
  }
}

void swr_list_store_center(int amp, int phs)
{
  g_amp_list[SWR_LIST_SIZE / 2] = amp;
  g_phs_list[SWR_LIST_SIZE / 2] = phs;
}

void swr_list_grid_draw() 
{
  // center marker
  g_disp.drawFastVLine(SWR_LIST_SIZE / 2, SWR_SCREEN_CHAR, SWR_SCREEN_CHAR / 2, BLACK);

  // grid
  for (uint8_t x = 0; x <= SWR_LIST_SIZE; x += SWR_GRID_STEP_X)
  {
    for (uint8_t y = 0; y <= SWR_GRAPH_HEIGHT - SWR_GRID_STEP_Y; y += SWR_GRID_STEP_Y)
    {
      g_disp.drawPixel(x + SWR_GRID_PAD_X, SWR_GRAPH_HEIGHT - y + 1, BLACK);
    }
  }
}

/* --------------------------------------------------------------------------*/

// NOTE, experimental, just for fun, does not look good on small screen

#ifdef USE_SMITH_CHART

// http://paulbourke.net/geometry/circlesphere/tvoght.c

int circle_circle_intersection(float x0, float y0, float r0,
                               float x1, float y1, float r1,
                               float *xi, float *yi,
                               float *xi_prime, float *yi_prime)
{
  float a, dx, dy, d, h, rx, ry;
  float x2, y2;

  dx = x1 - x0;
  dy = y1 - y0;

  //d = sqrt((dy*dy) + (dx*dx));
  d = hypot(dx,dy); // Suggested by Keith Briggs

  if (d > (r0 + r1))
  {
    return 0;
  }
  if (d < fabs(r0 - r1))
  {
    return 0;
  }
  a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d) ;

  x2 = x0 + (dx * a/d);
  y2 = y0 + (dy * a/d);

  h = sqrt((r0*r0) - (a*a));

  rx = -dy * (h/d);
  ry = dx * (h/d);

  *xi = x2 + rx;
  *xi_prime = x2 - rx;
  *yi = y2 + ry;
  *yi_prime = y2 - ry;

  return 1;
}

void swr_list_smith_grid_draw()
{
  g_disp.drawLine(0, SWR_SCREEN_HEIGHT / 2, 
    SWR_SCREEN_WIDTH, SWR_SCREEN_HEIGHT / 2, BLACK);
      
  // R circles: r = 1 / (R + 1); p = (R / (R + 1), 0);
  for (uint8_t i = 0; i < SWR_GRID_SMITH; i++)
  {
    float r = 1.0 / (g_smith_grid[i] + 1.0);
    float x = g_smith_grid[i] / (g_smith_grid[i] + 1.0);

    g_disp.drawCircle(
      SWR_SCREEN_WIDTH / 2 * x + SWR_SCREEN_WIDTH / 2, 
      SWR_SCREEN_HEIGHT / 2, 
      SWR_SCREEN_WIDTH / 2 * r, 
      BLACK);
  }
  
  // X circles: r = 1 / X; p = (1, 1 / X);
  for (uint8_t i = 0; i < SWR_GRID_SMITH; i++)
  {
    float r = 1.0 / g_smith_grid[i];
    float y = 1.0 / g_smith_grid[i];
    
    g_disp.drawCircle(
      SWR_SCREEN_WIDTH, 
      SWR_SCREEN_WIDTH / 2 * y + SWR_SCREEN_HEIGHT / 2, 
      SWR_SCREEN_WIDTH / 2 * r, 
      BLACK);

    g_disp.drawCircle(
      SWR_SCREEN_WIDTH, 
      -SWR_SCREEN_WIDTH / 2 * y + SWR_SCREEN_HEIGHT / 2, 
      SWR_SCREEN_WIDTH / 2 * r, 
      BLACK);
  }
}

void swr_smith_pt_from_z(float rs, float xs, uint8_t &x, uint8_t &y)
{
    float rs_r = 1.0 / (rs / 50.0 + 1.0);
    float rs_x = (rs / 50.0) / (rs / 50.0 + 1.0);

    if (xs == 0) 
    {
      x = SWR_SCREEN_WIDTH / 2;
      y = SWR_SCREEN_HEIGHT / 2;
      return;
    }
    
    float xs_r = 1.0 / (xs / 50.0);
    float xs_y = 1.0 / (xs / 50.0);

    float sect_x1, sect_y1, sect_x2, sect_y2;

    bool is_found = circle_circle_intersection(rs_x, 0, rs_r, 0, xs_y, xs_r, 
      &sect_x1, &sect_y1, &sect_x2, &sect_y2);

    x = SWR_SCREEN_WIDTH / 2 * sect_x1 + SWR_SCREEN_WIDTH / 2;
    y = SWR_SCREEN_WIDTH / 2 * sect_y1 + SWR_SCREEN_HEIGHT / 2;
}

void swr_list_smith_draw()
{
  uint8_t x, y, prev_x, prev_y;
  
  for (uint8_t i = 0; i < SWR_LIST_SIZE; i++) 
  {
    g_pt.amp = g_amp_list[i];
    g_pt.phs = g_phs_list[i];
    
    swr_calculate();
    
    swr_smith_pt_from_z(g_pt.rs, g_pt.xs, x, y);

    if (i > 0) 
    {
      g_disp.drawLine(prev_x, prev_y, x, y, BLACK);
    }

    prev_x = x;
    prev_y = y;
  }
}

#endif // USE_SMITH_CHART

/* --------------------------------------------------------------------------*/

uint8_t swr_screen_normalize(float swr)
{  
  float swr_norm = swr - 1;
  if (swr_norm > SWR_MAX_VALUE)
  {
    swr_norm = SWR_MAX_VALUE;
  }
  return swr_norm * (float)SWR_GRAPH_HEIGHT / (float)SWR_MAX_VALUE;
}

uint8_t swr_screen_z_normalize(float z)
{ 
  float z_norm = z;
  if (z < SWR_Z_MIN_OHM)
  {
    z_norm = SWR_Z_MIN_OHM;
  }
  if (z > SWR_Z_MAX_OHM)
  {
    z_norm = SWR_Z_MAX_OHM;
  }
  z_norm -= SWR_Z_MIN_OHM;
  return z_norm * (float)SWR_GRAPH_HEIGHT / (float)(SWR_Z_MAX_OHM - SWR_Z_MIN_OHM);
}

void swr_list_draw() 
{  
  for (uint8_t i = 0; i < SWR_LIST_SIZE; i++) 
  {
    g_pt.amp = g_amp_list[i];
    g_pt.phs = g_phs_list[i];
    
    swr_calculate();

    uint8_t swr_norm = swr_screen_normalize(g_pt.swr);

    g_disp.drawFastVLine(i, SWR_SCREEN_HEIGHT - swr_norm, swr_norm, BLACK);
  }
}

void swr_list_z_draw()
{  
  uint8_t prev_rs;
  uint8_t prev_xs;
  
  for (uint8_t i = 0; i < SWR_LIST_SIZE; i++) 
  {
    g_pt.amp = g_amp_list[i];
    g_pt.phs = g_phs_list[i];
    
    swr_calculate();

    uint8_t rs = swr_screen_z_normalize(g_pt.rs);
    uint8_t xs = swr_screen_z_normalize(g_pt.xs);

    if (i > 0) 
    {
      g_disp.drawLine(i - 1, SWR_SCREEN_HEIGHT - prev_rs, 
        i, SWR_SCREEN_HEIGHT - rs, BLACK);
      g_disp.drawLine(i - 1, SWR_SCREEN_HEIGHT - prev_xs, 
        i, SWR_SCREEN_HEIGHT - xs, BLACK);
    }

    prev_rs = rs;
    prev_xs = xs;
  }
}

void swr_list_sweep_and_fill() 
{
  uint32_t freq_hz = g_active_band.freq - g_active_band.freq_step * SWR_LIST_SIZE / 2;
    
  for (uint8_t i = 0; i < SWR_LIST_SIZE; i++) 
  {
    if (VALID_RANGE(freq_hz)) 
    {
      generator_set_frequency(freq_hz);
      
      process_rotary();
      process_rotary_button();

      swr_measure();
      swr_calculate();
      
      swr_update_minimum_swr(g_pt.swr, TO_KHZ(freq_hz));
    }
    
    g_amp_list[i] = g_pt.amp;
    g_phs_list[i] = g_pt.phs;
    
    freq_hz += g_active_band.freq_step;
    
  } // amp/phs list

  generator_set_frequency(g_active_band.freq);
}

void swr_update_minimum_swr(float swr, uint32_t freq_khz)
{
  if (swr < g_swr_min) 
  {
    g_swr_min = swr;
    g_freq_min = freq_khz;
  }
}

uint16_t swr_phs_cal_adjust(uint16_t phs)
{
  int16_t phs_cal_diff = g_active_band.adc_phs_cal_short - g_active_band.adc_phs_cal_open;
  int32_t phs_result = (long)abs((int)phs - g_active_band.adc_phs_cal_open) * 1024 / phs_cal_diff;
  if (phs_result <= 0) 
  {
    phs_result = 1;
  }
  return phs_result;
}

uint16_t swr_amp_cal_adjust(uint16_t amp)
{
  int16_t amp_cal_diff = g_active_band.adc_amp_cal_open - g_active_band.adc_amp_cal_50ohm;
  int32_t amp_result = (long)abs((int)amp - g_active_band.adc_amp_cal_open) * ADC_DB_CENTER / amp_cal_diff + ADC_DB_CENTER;
  if (amp_result <= ADC_DB_CENTER) 
  {
    amp_result = ADC_DB_CENTER + 1;
  }
  return amp_result;
}

/* --------------------------------------------------------------------------*/

void swr_measure()
{
  g_pt.freq_khz = TO_KHZ(g_active_band.freq);
  
  g_pt.amp = 0;
  g_pt.phs = 0;

  for (uint8_t i = 0; i < ADC_ITER_CNT; i++) 
  {
    g_pt.amp += analogRead(PIN_SWR_AMP);
    g_pt.phs += analogRead(PIN_SWR_PHS);
  }

  g_pt.amp /= ADC_ITER_CNT;
  g_pt.phs /= ADC_ITER_CNT;
}

void swr_calculate()
{
  g_pt.amp_adj = swr_amp_cal_adjust(g_pt.amp);
  g_pt.phs_adj = swr_phs_cal_adjust(g_pt.phs);

  g_pt.rl_db = fabs(((float)g_pt.amp_adj * ADC_DB_RES) + ADC_DB_OFFSET);
  g_pt.phi_deg = ((float)g_pt.phs_adj * ADC_DEG_RES);

  g_pt.rho = pow(10.0, g_pt.rl_db / -20.0);
  
  float re = g_pt.rho * cos(DEG_TO_RAD(g_pt.phi_deg));
  float im = g_pt.rho * sin(DEG_TO_RAD(g_pt.phi_deg));
  
  float denominator = ((1 - re) * (1 - re) + (im * im));
  
  g_pt.rs = fabs((1 - (re * re) - (im * im)) / denominator) * 50.0;
  g_pt.xs = fabs(2.0 * im) / denominator * 50.0;
  
  g_pt.z = sqrt(g_pt.rs * g_pt.rs + g_pt.xs * g_pt.xs);
  
  g_pt.swr = fabs(1.0 + g_pt.rho) / (1.001 - g_pt.rho);

  g_pt.rl_db *= -1;
}

void swr_print_info()
{
  g_disp.print(g_active_band.band_name); g_disp.print(F(": ")); 
    g_disp.print(g_pt.freq_khz); g_disp.println(F(" k"));
  
  g_disp.print(F("SWR: ")); g_disp.println(g_pt.swr);  
  
  g_disp.print(F("S11: ") );g_disp.print(g_pt.rl_db); 
    g_disp.println(F("dB"));
    
  g_disp.print(F("Z:")); g_disp.println(g_pt.z);
  
  g_disp.print(F("R:")); g_disp.print((uint16_t)g_pt.rs); 
    g_disp.print(F("+j")); g_disp.println((uint16_t)g_pt.xs); 
    
  g_disp.print(F("p:")); g_disp.print((uint16_t)g_pt.phi_deg); 
    g_disp.print(F(" ")); g_disp.print(g_pt.phs); 
    g_disp.print(F("/")); g_disp.println(g_pt.amp); 
}

/* --------------------------------------------------------------------------*/

void band_select_next() 
{
  g_active_band_index += 1;
  
  if (g_active_band_index >= BANDS_CNT) 
  {
    g_active_band_index = 0;
  }
  band_select(g_active_band_index);
}

void band_select(uint8_t index) 
{
  if (index < BANDS_CNT) 
  {
    memcpy_PF((void*)&g_active_band, (uint_farptr_t)&g_bands[index], sizeof(g_bands[index]));
    
    swr_list_clear();
    
    g_swr_min = SWR_MAX;
    g_freq_min = TO_KHZ(g_active_band.freq);

    generator_set_frequency(g_active_band.freq);
  }
}

void band_rotate_frequency(int8_t dir)
{
  g_active_band.freq += dir * g_active_band.freq_step;

  if (g_active_band.freq > FREQ_MAX) 
  {
    g_active_band.freq = FREQ_MAX;
  }
  generator_set_frequency(g_active_band.freq);
}

void band_rotate_step(int8_t dir)
{
  g_active_band.freq_step += dir * FREQ_STEP_INC;

  if (g_active_band.freq_step > FREQ_STEP_MAX) 
  { 
    if (dir < 0)
      g_active_band.freq_step = 0;
    if (dir > 0)
      g_active_band.freq_step = FREQ_STEP_MAX;
  }
}

/* --------------------------------------------------------------------------*/

void settings_draw()
{
  if (g_settings_selected) 
  {
    g_disp.print(F("*"));  
  }
  switch (g_settings_screen_state)
  {
    case S_SETTINGS_STEP:
        g_disp.println(F("<FREQ STEP>\n"));
        g_disp.print(F("STEP: ")); 
          g_disp.print(TO_KHZ(g_active_band.freq_step));
          g_disp.println(F(" kHz"));
        break;

    case S_SETTINGS_CONTRAST:
      g_disp.println(F("<CONTRAST>\n"));
      g_disp.print(F("CONTRAST: "));
        g_disp.println(g_contrast);
      break;
          
    case S_SETTINGS_CAL_50OHM:
      g_disp.println(F("<CAL 50 OHM>\n"));
      break;
      
    case S_SETTINGS_CAL_OPEN:
      g_disp.println(F("<CAL OPEN>\n"));
      break;
      
    case S_SETTINGS_CAL_SHORT:
      g_disp.println(F("<CAL SHORT>\n"));
      break;

    case S_SETTINGS_CAL_DEFAULT:
      g_disp.println(F("<CAL DEFAULT>\n"));
      break;
      
    default:
      break;
      
  } // settings screen 
}

void settings_select_next_screen()
{  
  switch (g_settings_screen_state)
  {
    case S_SETTINGS_STEP:
      g_settings_screen_state = S_SETTINGS_CONTRAST;
      break;
      
    case S_SETTINGS_CONTRAST:
      g_settings_screen_state = S_SETTINGS_CAL_50OHM;
      break;
      
    case S_SETTINGS_CAL_50OHM:
      g_settings_screen_state = S_SETTINGS_CAL_OPEN;
      break;
      
    case S_SETTINGS_CAL_OPEN:
      g_settings_screen_state = S_SETTINGS_CAL_SHORT;
      break;
      
    case S_SETTINGS_CAL_SHORT:
      g_settings_screen_state = S_SETTINGS_CAL_DEFAULT;
      break;

    case S_SETTINGS_CAL_DEFAULT:
      g_settings_screen_state = S_SETTINGS_STEP;
      break;
      
    default:
      break;
  }
}

void settings_select_prev_screen()
{
  switch (g_settings_screen_state)
  {
    case S_SETTINGS_STEP:
      g_settings_screen_state = S_SETTINGS_CAL_DEFAULT;
      break;

    case S_SETTINGS_CONTRAST:
      g_settings_screen_state = S_SETTINGS_STEP;
      break;
   
    case S_SETTINGS_CAL_50OHM:
      g_settings_screen_state = S_SETTINGS_CONTRAST;
      break;
      
    case S_SETTINGS_CAL_OPEN:
      g_settings_screen_state = S_SETTINGS_CAL_50OHM;
      break;
      
    case S_SETTINGS_CAL_SHORT:
      g_settings_screen_state = S_SETTINGS_CAL_OPEN;
      break;

    case S_SETTINGS_CAL_DEFAULT:
      g_settings_screen_state = S_SETTINGS_CAL_SHORT;
      break;
      
    default:
      break;
  }
}

void settings_rotate_screen(int8_t dir)
{
  if (dir == 1) 
  {
    settings_select_next_screen();  
  }
  else 
  {
    settings_select_prev_screen();
  }
}

void settings_save()
{  
  screen_save_settings();
}

void settings_rotate(int8_t dir)
{
  if (g_settings_selected)
  {
    switch (g_settings_screen_state) 
    {
      case S_SETTINGS_STEP:
        band_rotate_step(dir);
        break;

      case S_SETTINGS_CONTRAST:
        screen_rotate_contrast(dir);
        break;
        
      case S_SETTINGS_CAL_50OHM:
        break;
      
      case S_SETTINGS_CAL_OPEN:
        break;

      case S_SETTINGS_CAL_SHORT:
        break;

      case S_SETTINGS_CAL_DEFAULT:
        break;
        
      default:
        break;  
    }
  }
  else 
  {
    settings_rotate_screen(dir);  
  }
}

void settings_process_button()
{
  if (g_settings_selected)
  {
    settings_save();
    g_settings_selected = false;
  }
  else 
  {
    g_settings_selected = true;
  }
}

/* --------------------------------------------------------------------------*/

void screen_select_next() 
{
  switch (g_screen_state) 
  {
    case S_MAIN_SCREEN:
      g_screen_state = S_GRAPH_SWR;
      break;

    case S_GRAPH_SWR:
      g_screen_state = S_GRAPH_SWR_AUTO;
      swr_list_sweep_and_fill();
      break;

    case S_GRAPH_SWR_AUTO:
      g_screen_state = S_GRAPH_Z;
      break;

    case S_GRAPH_Z:
      g_screen_state = S_GRAPH_Z_AUTO;
      break;
      
    case S_GRAPH_Z_AUTO:
#ifdef USE_SMITH_CHART
      g_screen_state = S_GRAPH_SMITH;
#else
      g_screen_state = S_SETTINGS;
#endif
      break;
      
#ifdef USE_SMITH_CHART      
    case S_GRAPH_SMITH:
      g_screen_state = S_GRAPH_SMITH_AUTO;
      break;

    case S_GRAPH_SMITH_AUTO:
      g_screen_state = S_SETTINGS;
      break;
#endif

    case S_SETTINGS:
      g_screen_state = S_MAIN_SCREEN;
      break;

     default:
       break;
       
  }
}

/* --------------------------------------------------------------------------*/

void process_rotary() 
{
  uint8_t rotary_state = g_rotary.process();
  
  if (rotary_state) 
  {
    int8_t dir = (rotary_state == DIR_CW) ? -1 : 1;

    switch (g_screen_state) 
    {
      case S_GRAPH_SWR_AUTO:
      case S_GRAPH_Z_AUTO:
      case S_GRAPH_SMITH_AUTO:
        band_rotate_frequency(dir);
        break;

      case S_MAIN_SCREEN:
      case S_GRAPH_SWR:
      case S_GRAPH_Z:
      case S_GRAPH_SMITH:
        band_rotate_frequency(dir);
        if (rotary_state == DIR_CW) 
          swr_list_shift_right();
        else 
          swr_list_shift_left();
        break;

      case S_SETTINGS:
        settings_rotate(dir);
        break;
        
    } // screen state

    g_do_update = true;
    
  } // rotary changed
}

void process_rotary_button() 
{
  uint8_t rotary_btn_state = g_rotary.process_button();

  switch (rotary_btn_state) 
  {
    case BTN_RELEASED:
    
      switch (g_screen_state) 
      {
        case S_SETTINGS:
          settings_process_button();
          break;
          
        default:
          band_select_next(); 
          break;
      }
      g_do_update = true;
      break;

    case BTN_PRESSED_LONG:

      switch (g_screen_state)
      {
        case S_SETTINGS:
          settings_process_button();
          break;
          
         default:
          break;
      }
      screen_select_next();
      g_do_update = true;
      break;

    default:
      break;
      
  } // button state
}

void process_display_swr() 
{   
  swr_measure();
  swr_calculate();
  
  float swr = g_pt.swr;
  float rs = g_pt.rs;
  float xs = g_pt.xs;
  
  swr_list_store_center(g_pt.amp, g_pt.phs);
  swr_update_minimum_swr(g_pt.swr, 0);

  g_disp.clearDisplay();
  g_disp.setTextSize(1);
  g_disp.setTextColor(BLACK);
  g_disp.setCursor(0,0);

  if (g_screen_state == S_GRAPH_SWR_AUTO || 
      g_screen_state == S_GRAPH_Z_AUTO ||
      g_screen_state == S_GRAPH_SMITH_AUTO)
  {
      g_disp.print(F("*"));
      swr_list_sweep_and_fill();  
  }
  
  switch (g_screen_state) 
  {
    case S_MAIN_SCREEN:
      swr_print_info();
      break;

    case S_GRAPH_SWR_AUTO:
    case S_GRAPH_SWR:
      g_disp.print(g_pt.freq_khz); 
        g_disp.print(F(" ")); 
        g_disp.println(swr);
      swr_list_grid_draw();
      swr_list_draw();
      break;

    case S_GRAPH_Z_AUTO:
    case S_GRAPH_Z:
      g_disp.print((uint32_t)rs); 
        g_disp.print(F("+j")); 
        g_disp.println((uint32_t)xs);
      g_disp.setCursor(0, SWR_SCREEN_HEIGHT - SWR_SCREEN_CHAR);
      g_disp.print(g_pt.freq_khz);
      swr_list_grid_draw();
      swr_list_z_draw();
      break;
      
#ifdef USE_SMITH_CHART
    case S_GRAPH_SMITH:
    case S_GRAPH_SMITH_AUTO:
      g_disp.print((uint32_t)rs); 
        g_disp.print(F("+j")); 
        g_disp.println((uint32_t)xs);
      g_disp.setCursor(0, SWR_SCREEN_HEIGHT - SWR_SCREEN_CHAR);
      g_disp.print(g_pt.freq_khz);
      swr_list_smith_grid_draw();
      swr_list_smith_draw();
      break;
#endif

    case S_SETTINGS:
      settings_draw();
      break;

    default:
      break;

  } // screen state

  g_disp.display();
}

/* --------------------------------------------------------------------------*/

void loop()
{
  g_timer.run();
  
  if (g_do_update) 
  {
    process_display_swr();
    g_do_update = false;
  }
}

/* --------------------------------------------------------------------------*/
