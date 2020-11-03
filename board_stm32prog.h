
#ifdef TARGET_STM32PROG

// Repurposed STM32 programmer stick (st-link chinese clone)
// Features two LEDs, only one can be lit simultaneously.

static char led_en = 0, led_act = 0;

static void update_led_enable(int is_on) {
	led_en = is_on;

	if (!led_act) {
		if (led_en) {
			gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO9);
			gpio_clear(GPIOA, GPIO9);
		}
		else
			gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO9);
	}
}

static void update_led_activity(int is_on) {
	led_act = is_on;

	if (led_act) {
		gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO9);
		gpio_set(GPIOA, GPIO9);
	}
	else
		update_led_enable(led_en);
}

#endif

