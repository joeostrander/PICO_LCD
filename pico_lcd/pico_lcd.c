// Joe Ostrander
// PICO-LCD
// 2023.05.07

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

#define MIN_RUN 3

#define ONBOARD_LED_PIN             25
#define BACKLIGHT_PWM_PIN           12
#define BACKLIGHT_PWM_MAX           0xFFFF

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
        .xscale = 1,
        .yscale = 1,
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
static void initialize_gpio(void);
static uint16_t rgb888_to_rgb332(uint32_t color);
static void blink(uint8_t count, uint16_t millis_on, uint16_t millis_off);
static void set_backlight_level(int backlight_level);
int32_t single_solid_line(uint32_t *buf, size_t buf_length, uint16_t color);

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

int32_t single_solid_line(uint32_t *buf, size_t buf_length, uint16_t color)
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
    uint16_t line_num = scanvideo_scanline_number(dest->scanline_id);

    uint16_t color;
    uint16_t total_lines = VGA_MODE.height/VGA_MODE.yscale;
    uint16_t bar_height = (total_lines)/NUMBER_OF_COLORS; 

    uint16_t index = line_num / bar_height;
    index = index >= NUMBER_OF_COLORS ? 0 : index;
    color = rgb888_to_rgb332(basic_colors[index]);

    dest->data_used = single_solid_line(buf, buf_length, color);
    dest->status = SCANLINE_OK;
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
    uint32_t red = (color & 0xE00000) >> 21;
    uint32_t green = (color & 0xE000) >> 13;
    uint32_t blue = (color & 0xC0) >> 6;
    return (uint16_t)( ( blue<<PICO_SCANVIDEO_PIXEL_BSHIFT ) |( green<<PICO_SCANVIDEO_PIXEL_GSHIFT ) |( red<<PICO_SCANVIDEO_PIXEL_RSHIFT ) );
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
