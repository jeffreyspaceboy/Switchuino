#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"

// For basic GPIO use
#include "driver/gpio.h"
// For basic PWM control
#include "driver/ledc.h"
// For basic Servo control
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

//
// Basic GPIO Output Test
//

#define GPIO_OUTPUT_IO_0    18
#define GPIO_OUTPUT_IO_1    19
#define GPIO_OUTPUT_IO_2    21
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | \
		(1ULL<<GPIO_OUTPUT_IO_1) | (1ULL<<GPIO_OUTPUT_IO_2))

void app_init_gpio(void) {
	gpio_config_t io_conf;
	//disable interrupt
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 0;
	//configure GPIO with the given settings
	gpio_config(&io_conf);

    gpio_set_level(GPIO_OUTPUT_IO_0, 1);
    gpio_set_level(GPIO_OUTPUT_IO_1, 1);
    gpio_set_level(GPIO_OUTPUT_IO_2, 1);
}

//
// Basic PWM Output Test (use mcpwm for motor control).
//

#define GPIO_PWM_OUT_0 GPIO_OUTPUT_IO_0
#define GPIO_PWM_OUT_1 GPIO_OUTPUT_IO_1
#define GPIO_PWM_OUT_2 GPIO_OUTPUT_IO_2

#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)

void app_init_pwm(void) {
	int ch;
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_HIGH_SPEED_MODE,   // timer mode
        .timer_num = LEDC_TIMER_0,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel[3] = {
        {
            .channel    = LEDC_CHANNEL_0,
            .duty       = 100,
            .gpio_num   = GPIO_PWM_OUT_0,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        },
        {
            .channel    = LEDC_CHANNEL_1,
            .duty       = 500,
            .gpio_num   = GPIO_PWM_OUT_1,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        },
        {
            .channel    = LEDC_CHANNEL_2,
            .duty       = 1000,
            .gpio_num   = GPIO_PWM_OUT_2,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0
        },
    };
    for (ch = 0; ch < 3; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
        ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, ledc_channel[ch].duty);
        ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
    }

}

//
// Basic Servo Control Test
//

//Most standard servos work in pulse width range between 1000 to 2000 microseconds
#define SERVO_MIN_PULSEWIDTH 500 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2500 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 180 //Maximum angle in degrees

#define GPIO_SERVO_OUT_0 22
#define GPIO_SERVO_OUT_1 23

static uint32_t servo_per_degree_init(uint32_t degree_of_rotation) {
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

static void app_init_servo(void *arg) {
	uint32_t angle, count;
	//1. mcpwm gpio initialization
    printf("Initializing mcpwm servo control GPIOs...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_SERVO_OUT_0);    //Set GPIO_SERVO_OUT_0 as PWM0A
    //mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_SERVO_OUT_1);    //Set GPIO_SERVO_OUT_1 as PWM0B
    //2. initial mcpwm configuration
	printf("Configuring Initial Parameters of mcpwm......\n");
	mcpwm_config_t pwm_config;
	pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
	pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
	pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
	while (1) {
		for (count = 0; count < SERVO_MAX_DEGREE; count++) {
			printf("Angle of rotation: %d\n", count);
			angle = servo_per_degree_init(count);
			printf("pulse width: %dus\n", angle);
			mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
			vTaskDelay(1);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
		}
		for (count = SERVO_MAX_DEGREE; count > 0; count--) {
			printf("Angle of rotation: %d\n", count);
			angle = servo_per_degree_init(count);
			printf("pulse width: %dus\n", angle);
			mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
			vTaskDelay(1);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
		}
	}
}

void app_main(void)
{
    nvs_flash_init();
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );
    xTaskCreate(app_init_servo, "app_init_servo", 4096, NULL, 5, NULL);
}

