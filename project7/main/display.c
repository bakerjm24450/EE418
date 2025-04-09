/*
 * Functions for interfacing with LCD display. Also runs the GUI.
 *
 * The display is an HX8357D LCD connected by SPI, and we're using LVGL for the GUI. We
 * also use an STMPE610-based touch controller.
 * The interface to the LCD is based on the esp_lcd_touch example code
 * from Espressif.
 *
 * This code initializes the SPI interface, the LCD and touch devices, and the LVGL library,
 * including a background task to update the LVGL timer tick.
 * This code then creates the UI and another task to run the UI, updating the
 * values being displayed.
 *
 * Following the structure of the esp_lcd_touch code, the code includes high-level GUI code
 * that uses the LVGL library, code that supports the HX8357D TFT panel (including specialized
 * commands specific to HX8357D) and integrages it with LVGL, and code for the SPI interface
 * to the HX8357D.
 */

#include "esp_err.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_stmpe610.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include <time.h>
#include "esp_wifi.h"

#include "esp_timer.h"
#include <sys/lock.h>
#include <sys/param.h>
#include <unistd.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_check.h"

#include "lvgl.h"

#include "display.h"

static const char *TAG = "display";

/************************************************************************************
 * External functions needed for the display
 ************************************************************************************/
extern double dhtGetTemperature();
extern double dhtGetHumidity();
extern double thermostatGetSetpoint();
extern bool thermostatGetStatus();
extern void dataloggerGetLoggedData(int period, double *temperature,
                                    double *humidity, double *setpoint, bool *status);

/************************************************************************************
 * Constants
 ************************************************************************************/

// Settings from config
#define LCD_SCLK CONFIG_SCLK_GPIO         // pin used for SCLK
#define LCD_MOSI CONFIG_MOSI_GPIO         // MOSI SPI pin
#define LCD_MISO CONFIG_MISO_GPIO         // MISO SPI pin
#define LCD_DC CONFIG_DC_GPIO             // data / command pin
#define LCD_CS CONFIG_CS_GPIO             // CS pin for LCD display
#define LCD_TOUCH_CS CONFIG_TOUCH_CS_GPIO // CS pin for touch controller

// LCD settings that are not configurable
#define LCD_HOST SPI2_HOST                   // which SPI interface
#define LCD_SPI_CLOCK_HZ SPI_MASTER_FREQ_20M // LCD can run at 26MHz, but touch says 1 MHz (we're pushing it)
#define LCD_H_RES 320                        // HX8357D
#define LCD_V_RES 480                        // HX8357D
#define LCD_RST -1                           // reset is hard-wired on Adafruit TFT
#define LCD_BK_LIGHT -1                      // backlight is hardwired on Adafruit TFT
#define LCD_BK_LIGHT_ON 1
#define LCD_BK_LIGHT_OFF !LCD_BK_LIGHT_ON
#define LCD_CMD_BITS 8   // commands are 8 bits each
#define LCD_PARAM_BITS 8 // parameters are 8 bits each

// HX8357 commands -- these are commands specific to HX8357D in addition to the standard
// LCD commands
#define HX8357_CMD_OSC 0xb0   /* internal oscillator */
#define HX8357_CMD_PWR1 0xb1  /* power control */
#define HX8357_CMD_RGB 0xb3   /* set RGB interface */
#define HX8357_CMD_CYC 0xb4   /* display cycle */
#define HX8357_CMD_COM 0xb6   /* VCOM voltage */
#define HX8357_CMD_SETC 0xb9  /* enable extension commands */
#define HX8357_CMD_STBA 0xc0  /* set source option */
#define HX8357_CMD_PANEL 0xcc /* panel characteristics */
#define HX8357_CMD_GAMMA 0xe0 /* set gamma correction */

// LVGL specific constants
#define LVGL_DRAW_BUF_LINES 20     // how many lines to update at a time
#define LVGL_TICK_PERIOD_MS 2      // how often we update LVGL clock tick
#define LVGL_TASK_MAX_DELAY_MS 500 // longest allowed delay for updating LVGL timer
#define LVGL_TASK_MIN_DELAY_MS 1   // shortest allowed delay for updating LVGL timer
#define LVGL_TASK_STACK_SIZE 8192  // stack size for LVGL task
#define LVGL_TASK_PRIORITY 2       // priority for background task

#define SMALL_FONT lv_font_montserrat_18 // small font size
#define MED_FONT lv_font_montserrat_24   // medium font size
#define BIG_FONT lv_font_montserrat_48   // large font size
/************************************************************************************
 * Struct definitions
 ************************************************************************************/

// esp_lcd_panel struct specific to HX8357D
typedef struct
{
    esp_lcd_panel_t base;         // parent struct
    esp_lcd_panel_io_handle_t io; // I/O interface
    int reset_gpio_num;           // reset pin
    bool reset_level;             // reset active high or low
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel; // bits for each pixel
    uint8_t madctl_val;        // current value of LCD_CMD_MADCTL register
    uint8_t colmod_val;        // current value of LCD_CMD_COLMOD register
} hx8357d_panel_t;

/************************************************************************************
 * Static variables accessible by all functions
 ************************************************************************************/
static SemaphoreHandle_t mutex;

/************************************************************************************
 * Function prototypes
 ************************************************************************************/
static void init_devices(esp_lcd_panel_handle_t *panel, esp_lcd_touch_handle_t *touchpad);
static void init_graphics(esp_lcd_panel_handle_t *panel, esp_lcd_touch_handle_t *touchpad);

static void init_spi();
static void init_panel(esp_lcd_panel_handle_t *panel);
static void init_touchpad(esp_lcd_touch_handle_t *tp);

static esp_err_t esp_lcd_new_panel_hx8357d(const esp_lcd_panel_io_handle_t io,
                                           const esp_lcd_panel_dev_config_t *panel_dev_config,
                                           esp_lcd_panel_handle_t *ret_panel);
static esp_err_t panel_hx8357d_del(esp_lcd_panel_t *panel);
static esp_err_t panel_hx8357d_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_hx8357d_init(esp_lcd_panel_t *panel);
static esp_err_t panel_hx8357d_draw_bitmap(esp_lcd_panel_t *panel,
                                           int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_hx8357d_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_hx8357d_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_hx8357d_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_hx8357d_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_hx8357d_disp_on_off(esp_lcd_panel_t *panel, bool on_off);
static esp_err_t panel_hx8357d_sleep(esp_lcd_panel_t *panel, bool sleep);
static void panel_update_rotation(lv_display_t *disp);

static bool draw_bitmap_complete_callback(esp_lcd_panel_io_handle_t panel_io,
                                          esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
static void draw_bitmap_begin_callback(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);
static void touch_callback(lv_indev_t *indev, lv_indev_data_t *data);

static void increase_lvgl_tick(TimerHandle_t xTimer);
static void lvgl_timer_handler_task(void *arg);
static void guiTask(void *args);
static void createGUI(lv_display_t *disp);
static void updateGUI();

/*****************************************************************************************************
 * Top-level functions
 *****************************************************************************************************/

/**
 * Initialize the display and GUI. Also creates the task to update the GUI
 */
void displayInit()
{
    // create task to run the GUI. The task handles all initialization, too
    xTaskCreate(
        guiTask,
        "GUI task",
        8192,
        NULL,
        2,
        NULL);
}

/**
 * Task function for managing the display and GUI. Performs all initialization
 * and then updated GUI as needed.
 */
static void guiTask(void *args)
{
    static esp_lcd_panel_handle_t panel = NULL;
    static esp_lcd_touch_handle_t touchpad = NULL;

    // initialize the devices
    init_devices(&panel, &touchpad);

    // initialize the graphics system
    init_graphics(&panel, &touchpad);

    for (;;)
    {
        // update GUI
        xSemaphoreTakeRecursive(mutex, portMAX_DELAY);
        updateGUI();
        xSemaphoreGiveRecursive(mutex);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * Initialize the LCD and touchpad devices
 *
 * @param panel Pointer to LCD panel device
 * @param touchpad Pointer to touchpad device
 */
static void init_devices(esp_lcd_panel_handle_t *panel, esp_lcd_touch_handle_t *touchpad)
{
    // Initialize SPI interface
    init_spi();

    // initialize LCD device
    if (panel)
        init_panel(panel);

    // initialize touchpad
    if (touchpad)
        init_touchpad(touchpad);
}

/**
 * Initialize the LVGL graphics
 * @param panel Pointer to LCD panel device (if any)
 * @param touchpad Pointer to touchpad device (if any)
 */
static void init_graphics(esp_lcd_panel_handle_t *panel, esp_lcd_touch_handle_t *touchpad)
{
    /************* Initialize LVGL library *************/
    lv_init();

    /************** Create LVGL display with draw buffers ******/
    // create a lvgl display
    lv_display_t *display = lv_display_create(LCD_H_RES, LCD_V_RES);

    // alloc draw buffers used by LVGL
    size_t draw_buffer_sz = LCD_H_RES * LVGL_DRAW_BUF_LINES * sizeof(lv_color16_t);

    void *buf1 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf1);

    void *buf2 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf2);

    // initialize LVGL draw buffers
    lv_display_set_buffers(display, buf1, buf2, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);

    /************** associate the LCD to the display  **************/
    lv_display_set_user_data(display, *panel);

    // set color depth
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);

    /************************* register callback functions  *******************************/
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = draw_bitmap_complete_callback,
    };

    /* Register done callback */
    hx8357d_panel_t *hx8357d = __containerof(*panel, hx8357d_panel_t, base);
    esp_lcd_panel_io_handle_t io = hx8357d->io;

    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io, &cbs, display));

    // set the callback which can copy the rendered image to an area of the display
    lv_display_set_flush_cb(display, draw_bitmap_begin_callback);

    /***************** connect touchpad device to LVGL *****************/
    if (touchpad)
    {
        static lv_indev_t *indev;
        indev = lv_indev_create();
        lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
        lv_indev_set_display(indev, display);
        lv_indev_set_user_data(indev, *touchpad);
        lv_indev_set_read_cb(indev, touch_callback);
    }

    /****************** set up tick timer ****************************/
    // periodic timer to update LVGL tick
    TimerHandle_t timer = xTimerCreate("GUI timer", pdMS_TO_TICKS(LVGL_TICK_PERIOD_MS),
                                       pdTRUE, NULL, increase_lvgl_tick);
    xTimerStart(timer, portMAX_DELAY);

    // create mutex
    mutex = xSemaphoreCreateRecursiveMutex();

    xTaskCreate(
        lvgl_timer_handler_task,
        "GUI Timer task",
        LVGL_TASK_STACK_SIZE,
        NULL,
        LVGL_TASK_PRIORITY,
        NULL);

    // Lock the mutex due to the LVGL APIs are not thread-safe
    xSemaphoreTakeRecursive(mutex, portMAX_DELAY);
    createGUI(display);
    xSemaphoreGiveRecursive(mutex);
}

/************************************************************************************
 * I/O interface functions
 ************************************************************************************/

/**
 * Configure and initialize SPI interface
 */
static void init_spi()
{
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_SCLK,
        .mosi_io_num = LCD_MOSI,
        .miso_io_num = LCD_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4 * LCD_H_RES * LVGL_DRAW_BUF_LINES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
}

/**
 * Initialize LCD panel I/O and attach to SPI bus
 * @param panel Pointer to LCD panel
 */
static void init_panel(esp_lcd_panel_handle_t *panel)
{
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_DC,
        .cs_gpio_num = LCD_CS,
        .pclk_hz = LCD_SPI_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_hx8357d(io_handle, &panel_config, panel));

    // initialize the HX8357D
    ESP_ERROR_CHECK(esp_lcd_panel_reset(*panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(*panel));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(*panel, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(*panel, true));
}

/**
 * Initialize touch pad and attach to SPI bus
 * @param touchpad Pointer to touchpad controller
 */
static void init_touchpad(esp_lcd_touch_handle_t *tp)
{
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_spi_config_t tp_io_config = ESP_LCD_TOUCH_IO_SPI_STMPE610_CONFIG(LCD_TOUCH_CS);

    // attach touch controller to spi bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                             &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0},
    };

    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_stmpe610(tp_io_handle, &tp_cfg, tp));
}

/************************************************************************************
 * HX8357D panel interface functions
 ************************************************************************************/
/**
 * Create a new ESP_LCD panel using HX8357D
 * @param io I/O handle (typcially SPI-based)
 * @param panel_dev_config configuration for HX8357D
 * @param ret_panel Output parameter for storing panel that is created
 * @return Error code (ESP_OK if ok)
 */
static esp_err_t esp_lcd_new_panel_hx8357d(const esp_lcd_panel_io_handle_t io,
                                           const esp_lcd_panel_dev_config_t *panel_dev_config,
                                           esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;

    hx8357d_panel_t *hx8357d = NULL;

    // make sure arguments are sensible
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");

    // allocate memory for panel struct
    hx8357d = calloc(1, sizeof(hx8357d_panel_t));
    ESP_GOTO_ON_FALSE(hx8357d, ESP_ERR_NO_MEM, err, TAG, "no mem for hx8357d panel");

    // BGR or RGB order?
    switch (panel_dev_config->rgb_ele_order)
    {
    case LCD_RGB_ELEMENT_ORDER_RGB:
        hx8357d->madctl_val = 0;
        break;
    case LCD_RGB_ELEMENT_ORDER_BGR:
        hx8357d->madctl_val = LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported RGB element order");
        break;
    }

    // how many bits per pixel?
    uint8_t fb_bits_per_pixel = 0;
    switch (panel_dev_config->bits_per_pixel)
    {
    case 16: // RGB565
        hx8357d->colmod_val = 0x55;
        fb_bits_per_pixel = 16;
        break;
    case 18: // RGB666
        hx8357d->colmod_val = 0x66;
        // each color component (R/G/B) should occupy the 6 high bits of a byte, which means 3 full bytes are required for a pixel
        fb_bits_per_pixel = 24;
        break;
    case 24: // RGB888
        hx8357d->colmod_val = 0x77;
        fb_bits_per_pixel = 24;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    // fill in rest of struct
    hx8357d->io = io;
    hx8357d->fb_bits_per_pixel = fb_bits_per_pixel;
    hx8357d->reset_gpio_num = panel_dev_config->reset_gpio_num;
    hx8357d->reset_level = panel_dev_config->flags.reset_active_high;
    hx8357d->base.del = panel_hx8357d_del;
    hx8357d->base.reset = panel_hx8357d_reset;
    hx8357d->base.init = panel_hx8357d_init;
    hx8357d->base.draw_bitmap = panel_hx8357d_draw_bitmap;
    hx8357d->base.invert_color = panel_hx8357d_invert_color;
    hx8357d->base.set_gap = panel_hx8357d_set_gap;
    hx8357d->base.mirror = panel_hx8357d_mirror;
    hx8357d->base.swap_xy = panel_hx8357d_swap_xy;
    hx8357d->base.disp_on_off = panel_hx8357d_disp_on_off;
    hx8357d->base.disp_sleep = panel_hx8357d_sleep;
    *ret_panel = &(hx8357d->base);

    return ESP_OK;

err:
    // if error, then delete memory we allocated and release gpio for Reset pin
    if (hx8357d)
    {
        if (panel_dev_config->reset_gpio_num >= 0)
        {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(hx8357d);
    }
    return ret;
}

/**
 * Delete the panel and free its resources
 */
static esp_err_t panel_hx8357d_del(esp_lcd_panel_t *panel)
{
    hx8357d_panel_t *hx8357d = __containerof(panel, hx8357d_panel_t, base);

    if (hx8357d->reset_gpio_num >= 0)
    {
        gpio_reset_pin(hx8357d->reset_gpio_num);
    }

    free(hx8357d);
    return ESP_OK;
}

/**
 * Reset the panel
 */
static esp_err_t panel_hx8357d_reset(esp_lcd_panel_t *panel)
{
    hx8357d_panel_t *hx8357d = __containerof(panel, hx8357d_panel_t, base);
    esp_lcd_panel_io_handle_t io = hx8357d->io;

    // perform hardware reset
    if (hx8357d->reset_gpio_num >= 0)
    {
        gpio_set_level(hx8357d->reset_gpio_num, hx8357d->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(hx8357d->reset_gpio_num, !hx8357d->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    else
    {
        // perform software reset
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG,
                            "io tx param failed");
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0));
        vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5m before sending new command
    }

    return ESP_OK;
}

/**
 * Initialize the panel by sending configuration commands.
 */
static esp_err_t panel_hx8357d_init(esp_lcd_panel_t *panel)
{
    hx8357d_panel_t *hx8357d = __containerof(panel, hx8357d_panel_t, base);
    esp_lcd_panel_io_handle_t io = hx8357d->io;

    // LCD goes into sleep mode and display will be turned off after power on reset, exit sleep mode first
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0), TAG,
                        "io tx param failed");
    ;
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]){
                                                                          hx8357d->madctl_val,
                                                                      },
                                                  1),
                        TAG, "io tx param failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_COLMOD, (uint8_t[]){
                                                                          hx8357d->colmod_val,
                                                                      },
                                                  1),
                        TAG, "io tx param failed");

    // init specific to HX8357D

    // enable extensions
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, HX8357_CMD_SETC, (uint8_t[]){0xff, 0x83, 0x57}, 3), TAG, "io tx param failed");
    // RGB interface -- enable SDO
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, HX8357_CMD_RGB, (uint8_t[]){0x80, 0x00, 0x06, 0x06}, 4), TAG, "io tx param failed");
    // Display inversion OFF
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_INVOFF, NULL, 0), TAG, "io tx param failed");
    // VCOM voltage = -1.65
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, HX8357_CMD_COM, (uint8_t[]){0x25}, 1), TAG, "io tx param failed");
    // internal oscillator, idle 60 Hz, normal 70 Hz
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, HX8357_CMD_OSC, (uint8_t[]){0x68, 0x01}, 2), TAG, "io tx param failed");
    // white background, use BGR, reverse gate driver
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, HX8357_CMD_PANEL, (uint8_t[]){0x05}, 1), TAG, "io tx param failed");
    // Power control
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, HX8357_CMD_PWR1, (uint8_t[]){
                                                                           0x00, // not deep standby
                                                                           0x15, // step up circuit 1 2xVCi, BT=VCi+2VSP / VCi-2VSP
                                                                           0x1c, // VRH = 4.41/-4.41
                                                                           0x1c, // NVRH = 4.41/-4.41
                                                                           0x83, // GASEN=1, AP=small (0x3)
                                                                           0xaa, // FS1=FS0=Fosc/352
                                                                           0x20  // COMG, PON, DK, STB all off
                                                                       },
                                                  7),
                        TAG, "io tx param failed");
    // Source options
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, HX8357_CMD_STBA, (uint8_t[]){
                                                                           0x50, // normal mode 80 cycles
                                                                           0x50, // idle mode 80 cycles
                                                                           0x01, // source bias fine tune = large
                                                                           0x3c, // source bias coarse tune = middle
                                                                           0x1c, // gamma bias driving level = large
                                                                           0x08  // gamma OP on period
                                                                       },
                                                  6),
                        TAG, "io tx param failed");
    // display cycle
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, HX8357_CMD_CYC, (uint8_t[]){
                                                                          0x02, // no z-inversion, i_nw=0, n_nw=2 (2-dot inversion)
                                                                          0x40, // 320 cycles per line
                                                                          0x00, // no clock divide, use fosc/1
                                                                          0x2a, // dummy line number in normal mode
                                                                          0x2a, // dummy line number in idle mdoe
                                                                          0x0d, // GDON=13 (valid gate output start time)
                                                                          0x78  // GDOF=120 (gate output end time)
                                                                      },
                                                  7),
                        TAG, "io tx param failed");
    // Gamma correction
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, HX8357_CMD_GAMMA, (uint8_t[]){0x02, 0x0a, 0x11, 0x1d, 0x23, 0x35, 0x41, 0x4b, 0x4b, 0x42, 0x3a, 0x27, 0x1b, 0x08, 0x09, 0x03, 0x02, 0x0a, 0x11, 0x1d, 0x23, 0x35, 0x41, 0x4b, 0x4b, 0x42, 0x3a, 0x27, 0x1b, 0x08, 0x09, 0x03, 0x00, 0x01}, 34), TAG, "io tx param failed");

    return ESP_OK;
}

/**
 * Draw a (portion of a) bitmap on display
 */
static esp_err_t panel_hx8357d_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end,
                                           const void *color_data)
{
    hx8357d_panel_t *hx8357d = __containerof(panel, hx8357d_panel_t, base);
    esp_lcd_panel_io_handle_t io = hx8357d->io;

    x_start += hx8357d->x_gap;
    x_end += hx8357d->x_gap;
    y_start += hx8357d->y_gap;
    y_end += hx8357d->y_gap;

    // define an area of frame memory where MCU can access
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, (uint8_t[]){
                                                                         (x_start >> 8) & 0xFF,
                                                                         x_start & 0xFF,
                                                                         ((x_end - 1) >> 8) & 0xFF,
                                                                         (x_end - 1) & 0xFF,
                                                                     },
                                                  4),
                        TAG, "send command failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, (uint8_t[]){
                                                                         (y_start >> 8) & 0xFF,
                                                                         y_start & 0xFF,
                                                                         ((y_end - 1) >> 8) & 0xFF,
                                                                         (y_end - 1) & 0xFF,
                                                                     },
                                                  4),
                        TAG, "send command failed");

    // transfer frame buffer
    size_t len = (x_end - x_start) * (y_end - y_start) * hx8357d->fb_bits_per_pixel / 8;
    esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len);

    return ESP_OK;
}

/**
 * Invert colors on the display
 */
static esp_err_t panel_hx8357d_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    hx8357d_panel_t *hx8357d = __containerof(panel, hx8357d_panel_t, base);
    esp_lcd_panel_io_handle_t io = hx8357d->io;
    int command = 0;
    if (invert_color_data)
    {
        command = LCD_CMD_INVON;
    }
    else
    {
        command = LCD_CMD_INVOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param failed");
    return ESP_OK;
}

/**
 * Mirror the display
 */
static esp_err_t panel_hx8357d_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    hx8357d_panel_t *hx8357d = __containerof(panel, hx8357d_panel_t, base);
    esp_lcd_panel_io_handle_t io = hx8357d->io;
    if (mirror_x)
    {
        hx8357d->madctl_val |= LCD_CMD_MX_BIT;
    }
    else
    {
        hx8357d->madctl_val &= ~LCD_CMD_MX_BIT;
    }
    if (mirror_y)
    {
        hx8357d->madctl_val |= LCD_CMD_MY_BIT;
    }
    else
    {
        hx8357d->madctl_val &= ~LCD_CMD_MY_BIT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]){hx8357d->madctl_val}, 1), TAG, "io tx param failed");
    return ESP_OK;
}

/**
 * Swap x- and y- on display
 */
static esp_err_t panel_hx8357d_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    hx8357d_panel_t *hx8357d = __containerof(panel, hx8357d_panel_t, base);
    esp_lcd_panel_io_handle_t io = hx8357d->io;
    if (swap_axes)
    {
        hx8357d->madctl_val |= LCD_CMD_MV_BIT;
    }
    else
    {
        hx8357d->madctl_val &= ~LCD_CMD_MV_BIT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]){hx8357d->madctl_val}, 1), TAG, "io tx param failed");
    return ESP_OK;
}

/**
 * Set the gap on the display
 */
static esp_err_t panel_hx8357d_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    hx8357d_panel_t *hx8357d = __containerof(panel, hx8357d_panel_t, base);
    hx8357d->x_gap = x_gap;
    hx8357d->y_gap = y_gap;
    return ESP_OK;
}

/**
 * Turn display on/off
 */
static esp_err_t panel_hx8357d_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    hx8357d_panel_t *hx8357d = __containerof(panel, hx8357d_panel_t, base);
    esp_lcd_panel_io_handle_t io = hx8357d->io;
    int command = 0;
    if (on_off)
    {
        command = LCD_CMD_DISPON;
    }
    else
    {
        command = LCD_CMD_DISPOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param failed");
    return ESP_OK;
}

/**
 * Enter/exit sleep mode
 */
static esp_err_t panel_hx8357d_sleep(esp_lcd_panel_t *panel, bool sleep)
{
    hx8357d_panel_t *hx8357d = __containerof(panel, hx8357d_panel_t, base);
    esp_lcd_panel_io_handle_t io = hx8357d->io;
    int command = 0;
    if (sleep)
    {
        command = LCD_CMD_SLPIN;
    }
    else
    {
        command = LCD_CMD_SLPOUT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}

/**
 * Update the rotation of the display
 * @param disp Pointer to display
 */
static void panel_update_rotation(lv_display_t *disp)
{
    static lv_display_rotation_t current_rotation = LV_DISPLAY_ROTATION_0;

    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);

    // get the updated rotation
    lv_display_rotation_t rotation = lv_display_get_rotation(disp);

    // has it changed?
    if (rotation != current_rotation)
    {
        switch (rotation)
        {
        case LV_DISPLAY_ROTATION_0:
            // Rotate LCD display
            esp_lcd_panel_swap_xy(panel_handle, false);
            esp_lcd_panel_mirror(panel_handle, true, false);
            break;
        case LV_DISPLAY_ROTATION_90:
            // Rotate LCD display
            esp_lcd_panel_swap_xy(panel_handle, true);
            esp_lcd_panel_mirror(panel_handle, true, true);
            break;
        case LV_DISPLAY_ROTATION_180:
            // Rotate LCD display
            esp_lcd_panel_swap_xy(panel_handle, false);
            esp_lcd_panel_mirror(panel_handle, false, true);
            break;
        case LV_DISPLAY_ROTATION_270:
            // Rotate LCD display
            esp_lcd_panel_swap_xy(panel_handle, true);
            esp_lcd_panel_mirror(panel_handle, false, false);
            break;
        }

        // remember this for next time
        current_rotation = rotation;
    }
}

/*****************************************************************************************************
 * LVGL callback functions
 *****************************************************************************************************/

/**
 * Notify display that it is finished with transferring pixels. Called after SPI transfer is complete.
 */
static bool draw_bitmap_complete_callback(esp_lcd_panel_io_handle_t panel_io,
                                          esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    // signal the LVGL display that draw is complete
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

/**
 * Start transfer of draw buffer to display
 * @param disp Display to draw to
 * @param area Area of draw screen to draw
 * @param px_map Array of pixel data
 */
static void draw_bitmap_begin_callback(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    // update display rotation?
    panel_update_rotation(disp);

    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    // because SPI LCD is big-endian, we need to swap the RGB bytes order
    lv_draw_sw_rgb565_swap(px_map, (offsetx2 + 1 - offsetx1) * (offsety2 + 1 - offsety1));

    // copy buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
}

/**
 * Callback for responding to touchpad
 * @param indev Touchpad device
 * @param data Touch data (output parameter)
 */
static void touch_callback(lv_indev_t *indev, lv_indev_data_t *data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint16_t touchpad_strength[1] = {0};
    uint8_t touch_cnt = 0;

    esp_lcd_touch_handle_t touch_pad = lv_indev_get_user_data(indev);
    esp_lcd_touch_read_data(touch_pad);

    // get coordinates
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(touch_pad, touchpad_x,
                                                          touchpad_y, touchpad_strength, &touch_cnt, 1);

    if (touchpad_pressed && touch_cnt > 0)
    {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

/****************************************************************************************************
 * LVGL tick management
 ****************************************************************************************************/

// static void increase_lvgl_tick(void *arg)
static void increase_lvgl_tick(TimerHandle_t xTimer)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void lvgl_timer_handler_task(void *arg)
{
    uint32_t delay_time = 0; // in ms

    for (;;)
    {
        xSemaphoreTakeRecursive(mutex, portMAX_DELAY);
        delay_time = lv_timer_handler();
        xSemaphoreGiveRecursive(mutex);

        // make sure delay time is reasonable
        if (delay_time > LVGL_TASK_MAX_DELAY_MS)
            delay_time = LVGL_TASK_MAX_DELAY_MS;
        else if (delay_time < LVGL_TASK_MIN_DELAY_MS)
            delay_time = LVGL_TASK_MIN_DELAY_MS;

        vTaskDelay(pdMS_TO_TICKS(delay_time));
    }
}

/*******************************************************************************************************
 * GUI functions
 *******************************************************************************************************/

/**
 * Labels shown on the screen
 */
static lv_obj_t *dateLabel = NULL;                 // current date
static lv_obj_t *timeLabel = NULL;                 // current time
static lv_obj_t *wifiLabel = NULL;                 // wifi status
static lv_obj_t *humidityValue = NULL;             // measured humidity value
static lv_obj_t *temperatureValue = NULL;          // measured temperature
static lv_obj_t *setpointValue = NULL;             // thermostat setpoint
static lv_obj_t *heatLabel = NULL;                 // system status (On or Off)
static lv_obj_t *chart = NULL;                     // chart of logged data
static lv_chart_series_t *chartTemperature = NULL; // chart data for temperature
static lv_chart_series_t *chartSetpoint = NULL;    // chart data for setpoints

/**
 * Create the main window
 */
static void createGUI(lv_display_t *disp)
{
    // rotate 90 degrees
    lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_90);

    /*Change the active screen's background color*/
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_black(), LV_PART_MAIN);

    // create styles for the different font sizes
    static lv_style_t big_font;
    lv_style_init(&big_font);
    lv_style_set_text_font(&big_font, &BIG_FONT);
    lv_style_set_text_align(&big_font, LV_TEXT_ALIGN_CENTER);
    lv_style_set_text_color(&big_font, lv_color_white());

    static lv_style_t med_font;
    lv_style_init(&med_font);
    lv_style_set_text_font(&med_font, &lv_font_montserrat_24);
    lv_style_set_text_align(&med_font, LV_TEXT_ALIGN_CENTER);
    lv_style_set_text_color(&med_font, lv_color_white());

    static lv_style_t small_font;
    lv_style_init(&small_font);
    lv_style_set_text_font(&small_font, &lv_font_montserrat_18);
    lv_style_set_text_align(&small_font, LV_TEXT_ALIGN_CENTER);
    lv_style_set_text_color(&small_font, lv_color_white());

    /******************************************************************
     * Header row -- Date   Time   WiFi status
     * This info stays on all screens
     */
    lv_obj_t *header = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(header);
    lv_obj_set_size(header, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);

    // create a label for the date
    if (!dateLabel)
        dateLabel = lv_label_create(header);
    lv_obj_add_style(dateLabel, &small_font, 0);
    lv_obj_set_flex_grow(dateLabel, 1);
    lv_obj_set_style_text_color(dateLabel, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_text_align(dateLabel, LV_TEXT_ALIGN_LEFT, 0);
    lv_label_set_text(dateLabel, "-");

    // create a label for the time
    if (!timeLabel)
        timeLabel = lv_label_create(header);
    lv_obj_add_style(timeLabel, &small_font, 0);
    lv_obj_set_flex_grow(timeLabel, 1);
    lv_obj_set_style_text_color(timeLabel, lv_palette_main(LV_PALETTE_CYAN), 0);
    lv_obj_set_style_text_align(timeLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(timeLabel, "--:--");

    // create a label for the wifi status
    if (!wifiLabel)
        wifiLabel = lv_label_create(header);
    lv_obj_add_style(wifiLabel, &small_font, 0);
    lv_obj_set_flex_grow(wifiLabel, 1);
    lv_obj_set_style_text_align(wifiLabel, LV_TEXT_ALIGN_RIGHT, 0);
    lv_label_set_text(wifiLabel, "");

    /************************************************************************************
     * Tileview for the main content area
     * This allows us to switch between main display and the 24-hour log
     */
    lv_obj_t *tv = lv_tileview_create(lv_screen_active());
    lv_obj_set_height(tv, lv_pct(90));
    lv_obj_set_align(tv, LV_ALIGN_BOTTOM_MID);
    lv_obj_set_style_bg_color(tv, lv_color_black(), 0);

    /********************************************
     * Tile for main display
     */

    lv_obj_t *main_tile = lv_tileview_add_tile(tv, 0, 0, LV_DIR_RIGHT);
    lv_obj_set_style_bg_color(main_tile, lv_color_black(), 0);
    lv_obj_set_flex_flow(main_tile, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(main_tile, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);

    /**
     * First row -- Humidity  Temperature  Set To labels
     */
    // "Humidity" label
    lv_obj_t *humidityString = lv_label_create(main_tile);
    lv_obj_add_style(humidityString, &small_font, 0);
    lv_obj_set_flex_grow(humidityString, 1);
    lv_label_set_text_static(humidityString, "Humidity");

    // "Temperature" label
    lv_obj_t *temperatureString = lv_label_create(main_tile);
    lv_obj_add_style(temperatureString, &small_font, 0);
    lv_obj_set_flex_grow(temperatureString, 1);
    lv_label_set_text_static(temperatureString, "Temperature");

    // "Set to" label
    lv_obj_t *settingString = lv_label_create(main_tile);
    lv_obj_add_style(settingString, &small_font, 0);
    lv_obj_set_flex_grow(settingString, 1);
    lv_label_set_text_static(settingString, "Set To");

    /**
     * 2nd row -- Humidity   Temperature    Setpoint values
     */
    // humidity value
    humidityValue = lv_label_create(main_tile);
    lv_obj_add_flag(humidityValue, LV_OBJ_FLAG_FLEX_IN_NEW_TRACK);
    lv_obj_add_style(humidityValue, &big_font, 0);
    lv_obj_set_height(humidityValue, lv_pct(30));
    lv_obj_set_flex_grow(humidityValue, 1);
    lv_label_set_text(humidityValue, "--%");

    // temperature value
    temperatureValue = lv_label_create(main_tile);
    lv_obj_add_style(temperatureValue, &big_font, 0);
    lv_obj_set_flex_grow(temperatureValue, 1);
    lv_label_set_text(temperatureValue, "--\u00B0");

    // thermostat setpoint value
    setpointValue = lv_label_create(main_tile);
    lv_obj_add_style(setpointValue, &big_font, 0);
    lv_obj_set_flex_grow(setpointValue, 1);
    lv_label_set_text(setpointValue, "--\u00B0");

    /**
     * 3rd row -- Heat status
     */
    // create a label for the heat status
    heatLabel = lv_label_create(main_tile);
    lv_obj_add_flag(heatLabel, LV_OBJ_FLAG_FLEX_IN_NEW_TRACK);
    lv_obj_add_style(heatLabel, &med_font, 0);
    lv_obj_set_height(heatLabel, 100);
    lv_obj_set_flex_grow(heatLabel, 1);
    lv_label_set_text(heatLabel, "Heat: --");

    /*******************************************************************************
     * Create 24-hr log chart on a second tile
     */
    lv_obj_t *chart_tile = lv_tileview_add_tile(tv, 1, 0, LV_DIR_LEFT);
    lv_obj_set_style_bg_color(chart_tile, lv_color_white(), 0);

    /**
     * Chart to show last 24 hrs
     */

    chart = lv_chart_create(chart_tile);
    lv_obj_set_style_bg_color(chart, lv_color_black(), 0);
    lv_obj_set_size(chart, lv_pct(70), lv_pct(70));
    lv_obj_center(chart);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_div_line_count(chart, 2, 5);
    lv_chart_set_point_count(chart, 24 * 4);
    lv_obj_set_style_size(chart, 0, 0, LV_PART_INDICATOR);
    lv_obj_set_style_pad_all(chart, 0, 0);

    // y-axis with labels
    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 50, 100);

    static lv_style_t left_style;
    lv_style_init(&left_style);
    lv_style_set_text_font(&left_style, LV_FONT_DEFAULT);
    lv_style_set_text_color(&left_style, lv_color_white());
    lv_style_set_line_color(&left_style, lv_palette_main(LV_PALETTE_BLUE));

    lv_obj_t *scale_left = lv_scale_create(chart_tile);
    lv_scale_set_label_show(scale_left, true);
    lv_scale_set_mode(scale_left, LV_SCALE_MODE_VERTICAL_LEFT);
    lv_obj_set_size(scale_left, 25, lv_pct(70));
    lv_obj_align_to(scale_left, chart, LV_ALIGN_OUT_LEFT_MID, -10, 0);

    lv_scale_set_total_tick_count(scale_left, 11);
    lv_scale_set_major_tick_every(scale_left, 2);
    lv_scale_set_range(scale_left, 50, 100);
    lv_obj_add_style(scale_left, &left_style, LV_PART_INDICATOR);
    lv_obj_add_style(scale_left, &left_style, LV_PART_ITEMS);
    lv_obj_add_style(scale_left, &left_style, LV_PART_MAIN);

    // x-axis with labels
    static const char *hr[] = {"12am", "3", "6", "9", "12pm", "3", "6", "9", NULL};
    lv_obj_t *scale_bottom = lv_scale_create(chart_tile);
    lv_scale_set_mode(scale_bottom, LV_SCALE_MODE_HORIZONTAL_BOTTOM);
    lv_obj_set_size(scale_bottom, lv_pct(70), 25);
    lv_obj_align_to(scale_bottom, chart, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_scale_set_total_tick_count(scale_bottom, 9);
    lv_scale_set_major_tick_every(scale_bottom, 1);
    lv_obj_add_style(scale_bottom, &left_style, LV_PART_INDICATOR);
    lv_obj_add_style(scale_bottom, &left_style, LV_PART_ITEMS);
    lv_obj_add_style(scale_bottom, &left_style, LV_PART_MAIN);
    lv_scale_set_text_src(scale_bottom, hr);

    // add temperature and setpoints as data series
    chartTemperature = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    chartSetpoint = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);

    // zero out chart to start
    lv_chart_set_all_value(chart, chartTemperature, LV_CHART_POINT_NONE);
    lv_chart_set_all_value(chart, chartSetpoint, LV_CHART_POINT_NONE);
    lv_chart_refresh(chart);
}

/**
 * Update the info on the main screen
 */
static void updateGUI()
{
    // get current date and time
    time_t t = time(NULL);

    // update the date
    static char dateString[20];
    strftime(dateString, 20, "%e %B %Y", localtime(&t));
    if (dateLabel)
        lv_label_set_text(dateLabel, dateString);

    // update the time
    static char timeString[20];
    strftime(timeString, 20, "%r", localtime(&t));
    if (timeLabel)
        lv_label_set_text(timeLabel, timeString);

    // update the wifi status
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
    {
        // we're connected
        if (wifiLabel)
            lv_label_set_text(wifiLabel, LV_SYMBOL_WIFI);
    }
    else
    {
        // not connected
        if (wifiLabel)
            lv_label_set_text(wifiLabel, "");
    }

    // update current humidity
    if (humidityValue)
        lv_label_set_text_fmt(humidityValue, "%2.0f%%", dhtGetHumidity());

    // update current temperature
    if (temperatureValue)
        lv_label_set_text_fmt(temperatureValue, "%2.0f\u00B0", dhtGetTemperature());

    // update current setpoint temperature
    if (setpointValue)
        lv_label_set_text_fmt(setpointValue, "%2.0f\u00B0", thermostatGetSetpoint());

    // update heat status
    if (thermostatGetStatus())
    {
        // ON in red
        if (heatLabel)
        {
            lv_obj_set_style_text_color(heatLabel, lv_palette_main(LV_PALETTE_RED), 0);
            lv_label_set_text(heatLabel, "Heat: ON");
        }
    }
    else
    {
        // OFF in white
        if (heatLabel)
        {
            lv_obj_set_style_text_color(heatLabel, lv_color_white(), 0);
            lv_label_set_text(heatLabel, "Heat: OFF");
        }
    }

    // update chart
    if (chart)
    {
        // get data logger values and chart them
        for (int i = 0; i < 24 * 4; ++i)
        {
            // variables to hold logged data
            double temperature;
            double humidity;
            double setpoint;
            bool status;

            // get logged data
            dataloggerGetLoggedData(i, &temperature, &humidity, &setpoint, &status);

            // make sure not too cold or too hot
            if ((temperature >= 50) && (temperature <= 100))
            {
                lv_chart_set_value_by_id(chart, chartTemperature, i, (int)(temperature + 0.5));
            }
            else
            {
                lv_chart_set_value_by_id(chart, chartTemperature, i, LV_CHART_POINT_NONE);
            }

            // graph setpoint
            if ((setpoint >= 50) && (setpoint <= 100))
            {
                lv_chart_set_value_by_id(chart, chartSetpoint, i, (int)(setpoint + 0.5));
            }
            else
            {
                lv_chart_set_value_by_id(chart, chartSetpoint, i, LV_CHART_POINT_NONE);
            }
        }

        // get current time
        time_t rawtime;
        struct tm *now;
        time(&rawtime);
        now = localtime(&rawtime);

        int nextIndex = (4 * now->tm_hour + (now->tm_min / 15) + 1) % (24*4);
        
        // zero out next 2 data points
        lv_chart_set_value_by_id(chart, chartTemperature, nextIndex, LV_CHART_POINT_NONE);
        lv_chart_set_value_by_id(chart, chartTemperature, (nextIndex+1)%(24*4), LV_CHART_POINT_NONE);
    }
}
