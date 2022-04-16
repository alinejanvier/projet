#include <gpio.h>
#include <selector.h>

#define Sel0     	GPIOC, 13
#define Sel1     	GPIOC, 14
#define Sel2     	GPIOC, 15
#define Sel3     	GPIOD, 4


void init_selector(void)
{
    // Enable GPIOC and GPIOD peripheral clock
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;

    gpio_config_input_pd(Sel0);
	gpio_config_input_pd(Sel1);
	gpio_config_input_pd(Sel2);
	gpio_config_input_pd(Sel3);

}

int get_selector()
{
	return gpio_read(Sel0) + 2 * gpio_read(Sel1) + 4 * gpio_read(Sel2) + 8 * gpio_read(Sel3);
}

