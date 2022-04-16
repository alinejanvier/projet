#include <stm32f4xx.h>
#include <system_clock_config.h>
#include <gpio.h>
#include <selector.h>
#include <timer.h>
#include <main.h>

// Init function required by __libc_init_array
void _init(void) {}

// Simple delay function
void delay(unsigned int n)
{
    while (n--) {
        __asm__ volatile ("nop");
    }
}

/***************** LED SEQUENCES *****************/

// LEDs sequences with order LED7, LED5, LED3, LED1
static const uint8_t seq1[8][4] = {
    {0, 0, 0, 1},	// ON1
	{0, 0, 0, 0},	// OFF1
	{0, 0, 1, 0},	// ON3
	{0, 0, 0, 0},	// OFF3
    {0, 1, 0, 0},	// ON5
	{0, 0, 0, 0},	// OFF5
	{1, 0, 0, 0},	// ON7
	{0, 0, 0, 0},	// OFF7
};

static const uint8_t seq2[8][4] = {
    {0, 0, 0, 1},	// ON1
	{0, 0, 1, 1},	// ON3
    {0, 1, 1, 1},	// ON5
	{1, 1, 1, 1},	// ON7
    {1, 1, 1, 0},	// OFF1
	{1, 1, 0, 0},	// OFF3
    {1, 0, 0, 0},	// OFF5
	{0, 0, 0, 0},	// OFF7
};

/**
 * @brief   Updates the LED states
 *
 * @param[in] out       pointer to the table containing the state
 *
 */


static void LEDs_update(const uint8_t *out)
{
    /* LEDs */
    out[3] ? gpio_clear(LED1) : gpio_set(LED1);
    out[2] ? gpio_clear(LED3) : gpio_set(LED3);
    out[1] ? gpio_clear(LED5) : gpio_set(LED5);
    out[0] ? gpio_clear(LED7) : gpio_set(LED7);
}

int main(void)
{
	int selector, old_selector = 0;
	int sequence_pos = 0;
    SystemClock_Config();

    // Enable GPIOB and GPIOD peripheral clock for the LEDs
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN;

    // BODY_LED init
    gpio_set(BODY_LED);
    gpio_config_output_pushpull(BODY_LED);

    //config timer7 for the toggle of BODY_LED
    timer7_start();

    // LEDs defined in main.h
    gpio_config_output_opendrain(LED1);
    gpio_config_output_opendrain(LED3);
    gpio_config_output_opendrain(LED5);
    gpio_config_output_opendrain(LED7);

    init_selector();

    while (1) {
        delay(SystemCoreClock/32);
        selector = get_selector();
        if (selector != old_selector) {
        	sequence_pos = 0;
			gpio_set(LED1);
			gpio_set(LED3);
			gpio_set(LED5);
			gpio_set(LED7);
        }
        old_selector = selector;
        switch (selector)
        {
        	case 0: // All LEDs off
				gpio_set(LED1);
				gpio_set(LED3);
				gpio_set(LED5);
				gpio_set(LED7);
				break;
        	case 1:
        		gpio_toggle(LED1);
				break;
        	case 2:
        		gpio_toggle(LED3);
				break;
        	case 3:
        		gpio_toggle(LED5);
				break;
        	case 4:
        		gpio_toggle(LED7);
				break;
        	case 5: // Use Sequence 1
        		LEDs_update(seq1[sequence_pos]);
        		sequence_pos++;
        		sequence_pos %= 8;
				break;
        	case 6: // Use Sequence 2
        		LEDs_update(seq2[sequence_pos]);
        		sequence_pos++;
        		sequence_pos %= 8;
				break;
        	default:
			break;
        }
    }
}

