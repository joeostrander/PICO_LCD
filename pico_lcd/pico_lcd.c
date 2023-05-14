// Joe Ostrander
// PICO-LCD
// 2023.05.07

// Note about scanvideo:
// When scaling, there is a bug in scanvideo.c that prevents line zero from repeating
// see my fix --> https://github.com/joeostrander/pico-extras/commit/3ed9467f0203acd9bedfdbb08bed8f31b61b320c

#include <stdio.h>
#include <stdlib.h>
#include "time.h"
#include "pico.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/scanvideo.h"
#include "pico/scanvideo/composable_scanline.h"
#include "pico/sync.h"
#include "hardware/vreg.h"
#include "pico/stdio.h"
#include "hardware/pwm.h"
#include "math.h"

#define MIN_RUN 3

#define ONBOARD_LED_PIN             25
#define BACKLIGHT_PWM_PIN           12
#define BACKLIGHT_PWM_MAX           0xFFFF

// #define PCB_V1  // MSB-->LSB bits reversed

const scanvideo_timing_t vga_timing_480x272 =
{
        .clock_freq = 12000000,

        .h_active = 480,
        .v_active = 272,

        .h_front_porch = 2,
        .h_pulse = 12,
        .h_total = 525,
        .h_sync_polarity = 1,

        .v_front_porch = 1,
        .v_pulse = 1,
        .v_total = 285,
        .v_sync_polarity = 1,

        .enable_clock = 1,
        .clock_polarity = 1,

        .enable_den = 1
};

const scanvideo_mode_t vga_mode_tft_480x272 =
{
        .default_timing = &vga_timing_480x272,
        .pio_program = &video_24mhz_composable,
        .width = 480,
        .height = 272,
        .xscale = 2,
        .yscale = 2,
};

const scanvideo_timing_t vga_timing_800x480 =
{
        .clock_freq = 24000000,

        .h_active = 800,
        .v_active = 480,

        .h_front_porch = 40,
        .h_pulse = 12,
        .h_total = 960,
        .h_sync_polarity = 1,

        .v_front_porch = 2,
        .v_pulse = 2,
        .v_total = 500, 
        .v_sync_polarity = 1,

        .enable_clock = 1,
        .clock_polarity = 0,

        .enable_den = 0
};

const scanvideo_mode_t vga_mode_tft_800x480 =
{
        .default_timing = &vga_timing_800x480,
        .pio_program = &video_24mhz_composable,
        .width = 800,
        .height = 480,
        .xscale = 4,
        .yscale = 4,
};

//#define VGA_MODE        vga_mode_tft_480x272
#define VGA_MODE        vga_mode_tft_800x480

typedef enum
{
    COLOR_BLACK = 0,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_YELLOW,
    COLOR_WHITE,
    COLOR_LIGHT_GREY,
    COLOR_DARK_GREY,
    COLOR_PURPLE,
    COLOR_ORANGE,
    NUMBER_OF_COLORS
} COLORS;

static uint32_t basic_colors[NUMBER_OF_COLORS] = 
{
    [COLOR_BLACK]       = 0x000000,
    [COLOR_BLUE]        = 0x0000FF,
    [COLOR_WHITE]       = 0xFFFFFF,
    [COLOR_LIGHT_GREY]  = 0x808080,
    [COLOR_DARK_GREY]   = 0x404040,
    [COLOR_RED]         = 0xFF0000,
    [COLOR_GREEN]       = 0x00FF00,
    [COLOR_YELLOW]      = 0xFFFF00,
    [COLOR_PURPLE]      = 0xFF00FF,
    [COLOR_ORANGE]      = 0xFF7F00
};

static int backlight_level = 10;    // 1 to 10

static semaphore_t video_initted;

static void core1_func(void);
static void render_scanline(scanvideo_scanline_buffer_t *buffer);
int32_t single_scanline(uint32_t *buf, size_t buf_length, int line_index);
static void initialize_gpio(void);
static uint16_t rgb888_to_rgb332(uint32_t color);
static uint16_t rgb888_to_rgb332_alt(uint8_t r, uint8_t g, uint8_t b);
static void blink(uint8_t count, uint16_t millis_on, uint16_t millis_off);
static void set_backlight_level(int backlight_level);
static uint8_t reverse(uint8_t b);
static int32_t single_solid_line(uint32_t *buf, size_t buf_length, uint16_t color);

int main(void) 
{
    hw_set_bits(&vreg_and_chip_reset_hw->vreg, VREG_AND_CHIP_RESET_VREG_VSEL_BITS);
    sleep_ms(10);

    //set_sys_clock_khz(300000, true);
    set_sys_clock_khz(240000, true);

    // Create a semaphore to be posted when video init is complete.
    sem_init(&video_initted, 0, 1);

    // Launch all the video on core 1.
    multicore_launch_core1(core1_func);

    // Wait for initialization of video to be complete.
    sem_acquire_blocking(&video_initted);

    initialize_gpio();

    while (true) 
    {
        blink(3, 100, 2000);
    }
}

static int32_t single_solid_line(uint32_t *buf, size_t buf_length, uint16_t color)
{
    uint16_t *p16 = (uint16_t *) buf;

    *p16++ = COMPOSABLE_COLOR_RUN;
    *p16++ = color; 
    *p16++ = VGA_MODE.width/VGA_MODE.xscale;

    // black pixel to end line
    *p16++ = COMPOSABLE_RAW_1P;
    *p16++ = 0;

    *p16++ = COMPOSABLE_EOL_ALIGN;
    
    return ((uint32_t *) p16) - buf;
}

static void render_scanline(scanvideo_scanline_buffer_t *dest) 
{
  uint32_t *buf = dest->data;
  size_t buf_length = dest->data_max;
  int line_num = scanvideo_scanline_number(dest->scanline_id);
  dest->data_used = single_scanline(buf, buf_length, line_num);
  dest->status = SCANLINE_OK;
}


int32_t single_scanline(uint32_t *buf, size_t buf_length, int line_index)
{
    uint16_t* p16 = (uint16_t *) buf;
    uint16_t* first_pixel;

    *p16++ = COMPOSABLE_RAW_RUN;
    first_pixel = p16;
    *p16++ = 0; // first pixel, should replace later, but maybe just leave black
    *p16++ = VGA_MODE.width - MIN_RUN;

    uint16_t pixel_count = 1;
    uint16_t lines_per_section = VGA_MODE.height/VGA_MODE.yscale/5;
    uint16_t color;
    uint16_t total_columns = VGA_MODE.width/VGA_MODE.xscale;
    uint8_t bits_red = (uint16_t)PICO_SCANVIDEO_DPI_PIXEL_RCOUNT;
    uint8_t bits_green = (uint16_t)PICO_SCANVIDEO_DPI_PIXEL_GCOUNT;
    uint8_t bits_blue = (uint16_t)PICO_SCANVIDEO_DPI_PIXEL_BCOUNT;
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    uint8_t shift_red = 8 - bits_red;
    uint8_t shift_green = 8 - bits_green;
    uint8_t shift_blue = 8 - bits_blue;

    uint16_t levels_red = pow(2,bits_red);
    uint16_t levels_green = pow(2,bits_green);
    uint16_t levels_blue = pow(2,bits_blue);

    uint16_t step_red = 1 << shift_red;
    uint16_t step_green = 1 << shift_green;
    uint16_t step_blue = 1 << shift_blue;

    uint16_t max_red = (levels_red - 1) << shift_red;
    uint16_t max_green = (levels_green - 1) << shift_green;
    uint16_t max_blue = (levels_blue - 1) << shift_blue;

    
    uint16_t color_sections = 5;
    uint16_t cols_per_section = total_columns/color_sections;

    uint16_t total_steps_red = max_red / step_red + 1;
    uint16_t total_steps_green = max_green / step_green + 1;
    uint16_t total_steps_blue = max_blue / step_blue + 1;
    uint16_t cols_per_step_red = cols_per_section/total_steps_red;
    uint16_t cols_per_step_green = cols_per_section/total_steps_green;
    uint16_t cols_per_step_blue = cols_per_section/total_steps_blue;

    int i;

    if (line_index < lines_per_section)
    {
      for (i = levels_red-1; i > 0; i--)
      {
        red = i << shift_red;
        color = rgb888_to_rgb332_alt(red, green, blue);
        for (int j = 0; j < (total_columns/(levels_red-1)); j++)
        {
          *p16++ = color;
          pixel_count++;
        }
      }
    }
    else if (line_index < lines_per_section*2)
    {
      for (i = levels_green-1; i > 0; i--)
      {
        green = i << shift_green;
        color = rgb888_to_rgb332_alt(red, green, blue);
        for (int j = 0; j < (total_columns/(levels_green-1)); j++)
        {
          *p16++ = color;
          pixel_count++;
        }
      }
    }
    else if (line_index < lines_per_section*3)
    {

      for (i = levels_blue-1; i > 0; i--)
      {
        blue = i << shift_blue;
        color = rgb888_to_rgb332_alt(red, green, blue);
        for (int j = 0; j < (total_columns/(levels_blue-1)); j++)
        {
          *p16++ = color;
          pixel_count++;
        }
      }
    }
    else if (line_index < lines_per_section*4)
    {
      for (i = levels_blue-1; i >= 0; i--)
      {
        red = i << shift_blue;
        green = i << shift_blue;
        blue = i << shift_blue;
        color = rgb888_to_rgb332_alt(red, green, blue);
        for (int j = 0; j < (total_columns/levels_blue); j++)
        {
          *p16++ = color;
          pixel_count++;
        }
      }
    }
    else
    {
      red = max_red;
      for (i = 0; i < cols_per_section; i++)
      {
          green = (i / cols_per_step_green) * step_green;
          color = rgb888_to_rgb332_alt(red, green, blue);
          
          *p16++ = color;
          pixel_count++;
      }

      for (i = cols_per_section-1; i >= 0; i--)
      {
          red = (i / cols_per_step_red) * step_red;
          color = rgb888_to_rgb332_alt(red, green, blue);
          *p16++ = color;
          pixel_count++;
      }

      for (i = 0; i < cols_per_section; i++)
      {
          blue = (i / cols_per_step_blue) * step_blue;
          color = rgb888_to_rgb332_alt(red, green, blue);
          *p16++ = color;
          pixel_count++;
      }

      for (i = cols_per_section-1; i >= 0; i--)
      {
          green = (i / cols_per_step_green) * step_green;
          color = rgb888_to_rgb332_alt(red, green, blue);
          *p16++ = color;
          pixel_count++;
      }

      for (i = 0; i < cols_per_section; i++)
      {
          red = (i / cols_per_step_red) * step_red;
          color = rgb888_to_rgb332_alt(red, green, blue);
          *p16++ = color;
          pixel_count++;
      }


    }

    while (pixel_count < VGA_MODE.width/VGA_MODE.xscale)
    {
      *p16++ = color;
      pixel_count++;
    }
    
    // black pixel to end line
    *p16++ = COMPOSABLE_RAW_1P;
    *p16++ = 0;


    *p16++ = COMPOSABLE_EOL_ALIGN;  // TODO... how to determine when to do skip align

    return ((uint32_t *) p16) - buf;

}

static void core1_func(void) 
{
    
    hard_assert(VGA_MODE.width + 4 <= PICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS * 2);    

    // Initialize video and interrupts on core 1.
    scanvideo_setup(&VGA_MODE);
    scanvideo_timing_enable(true);
    sem_release(&video_initted);

    while (true) 
    {
        scanvideo_scanline_buffer_t *scanline_buffer = scanvideo_begin_scanline_generation(true);
        render_scanline(scanline_buffer);
        scanvideo_end_scanline_generation(scanline_buffer);
    }
}

static void initialize_gpio(void)
{    
    //Onboard LED
    gpio_init(ONBOARD_LED_PIN);
    gpio_set_dir(ONBOARD_LED_PIN, GPIO_OUT);
    gpio_put(ONBOARD_LED_PIN, 0);

    // LCD Backlight
    gpio_set_function(BACKLIGHT_PWM_PIN, GPIO_FUNC_PWM);
    uint sliceNum=pwm_gpio_to_slice_num (BACKLIGHT_PWM_PIN); 
    pwm_config config = pwm_get_default_config();
    pwm_init(sliceNum, &config, true);
    set_backlight_level(10);  // Max
    // gpio_init(BACKLIGHT_PWM_PIN);
    // gpio_set_dir(BACKLIGHT_PWM_PIN, GPIO_OUT);
    // gpio_put(BACKLIGHT_PWM_PIN, 1);
    
}

static uint16_t rgb888_to_rgb332(uint32_t color)
{
    uint8_t red = (color >> 16) & 0xE0;
    uint8_t green = (color >> 8) & 0xE0;
    uint8_t blue = color & 0xC0;
    return rgb888_to_rgb332_alt(red, green, blue);
}

#ifdef PCB_V1
static uint16_t rgb888_to_rgb332_alt(uint8_t r, uint8_t g, uint8_t b)
{
  r = reverse(r);
  g = reverse(g);
  b = reverse(b);
  return (uint16_t)((r<<5) | (g<<2) | b);
}
#else
static uint16_t rgb888_to_rgb332_alt(uint8_t r, uint8_t g, uint8_t b)
{
  return (uint16_t)(r | (g>>3) | (b>>6));
}
#endif  



static uint8_t reverse(uint8_t b) 
{
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

static void blink(uint8_t count, uint16_t millis_on, uint16_t millis_off)
{
  static uint8_t blink_count = 0;
  static uint16_t current_rate = 200;
  static uint32_t last_led_change = 0;

  if (count == 0)
  {
    gpio_put(ONBOARD_LED_PIN, 0);
    return;
  }

  uint32_t current_us = time_us_32();
  bool current_state = gpio_get(ONBOARD_LED_PIN);

  bool timeup = (current_us - last_led_change) > (current_rate*1000);
  if (timeup)
  {
    last_led_change = current_us;
    if (current_rate == millis_on)
    {
      bool new_state = !current_state;
      gpio_put(ONBOARD_LED_PIN, new_state);
      if (new_state ==  false)
      {
        blink_count++;
        if (blink_count >= count)
        {
          blink_count = 0;
          current_rate = millis_off;
        }
      }
      return;
    }
    
    blink_count = 0;
    current_rate = millis_on;
  }
}

static void set_backlight_level(int backlight_level)
{
    // level 1 to 10 only
    backlight_level = backlight_level < 1 ? 1 : backlight_level > 10 ? 10 : backlight_level;

    uint pwm_value = (uint)(BACKLIGHT_PWM_MAX * backlight_level / 10);
    pwm_set_gpio_level(BACKLIGHT_PWM_PIN, pwm_value);
}
