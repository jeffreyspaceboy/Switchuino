//Needed for task delays
#include "esp_event_loop.h"
// For PWM LED control
#include "driver/ledc.h"
// For Servo control
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#define DEBUG


//esp_err_t event_handler(void *ctx, system_event_t *event)
//{
//    return ESP_OK;
//}

//
// Basic RGB LED Control
//

#define LED_PWM_RED   18
#define LED_PWM_GREEN 19
#define LED_PWM_BLUE  21

void LED_init(void) {
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_HIGH_SPEED_MODE,   // timer mode
        .timer_num = LEDC_TIMER_0,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);
}

void LED_setColor(uint32_t RED, uint32_t GREEN, uint32_t BLUE) {
	int ch;
	ledc_channel_config_t ledc_channel[3] = {
		{
			.channel    = LEDC_CHANNEL_0,
			.duty       = RED,
			.gpio_num   = LED_PWM_RED,
			.speed_mode = LEDC_HIGH_SPEED_MODE,
			.hpoint     = 0,
			.timer_sel  = LEDC_TIMER_0
		},
		{
			.channel    = LEDC_CHANNEL_1,
			.duty       = GREEN,
			.gpio_num   = LED_PWM_GREEN,
			.speed_mode = LEDC_HIGH_SPEED_MODE,
			.hpoint     = 0,
			.timer_sel  = LEDC_TIMER_0
		},
		{
			.channel    = LEDC_CHANNEL_2,
			.duty       = BLUE,
			.gpio_num   = LED_PWM_BLUE,
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
// Basic Servo Control
//

//Most standard servos work in pulse width range between 1000 to 2000 microseconds
#define SERVO_MIN_PULSEWIDTH 500 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2500 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 180 //Maximum angle in degrees

#define GPIO_SERVO_OUT_0 22

uint32_t SERVO_CURRENT_DEGREE = 0;


static uint32_t servo_per_degree_init(uint32_t degree_of_rotation) {
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}


static void servo_init(void) {
	//1. mcpwm gpio initialization
	#ifdef DEBUG
    printf("Initializing MCPWM Servo Control GPIO...\n");
	printf("Configuring Initial MCPWM Parameters...\n");
	#endif
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_SERVO_OUT_0);    //Set GPIO_SERVO_OUT_0 as PWM0A
	//2. initial mcpwm configuration
	mcpwm_config_t pwm_config;
	pwm_config.frequency = 50; //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
	pwm_config.cmpr_a = 0;     //duty cycle of PWMxA = 0
	pwm_config.cmpr_b = 0;     //duty cycle of PWMxB = 0
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
	SERVO_CURRENT_DEGREE = servo_per_degree_init(0);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
}

static void servo_moveTo(uint32_t angle) {
	SERVO_CURRENT_DEGREE = servo_per_degree_init(angle);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, SERVO_CURRENT_DEGREE);
	vTaskDelay(50);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
}

void app_main(void){
    servo_init();
    LED_init();
    while(1){
		LED_setColor(5000,0,0);
		servo_moveTo(0);
		vTaskDelay(50);
		LED_setColor(0,5000,0);
		servo_moveTo(90);
		vTaskDelay(50);
		LED_setColor(0,0,5000);
		servo_moveTo(180);
		vTaskDelay(50);
    }
}

