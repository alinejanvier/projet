#include <ch.h>
#include <hal.h>
#include <math.h>
#include <chprintf.h>
#include <messagebus.h>
#include <i2c_bus.h>
#include <imu.h>

#define NB_SAMPLES_OFFSET     200

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void panic_handler(const char *reason)
{
    (void)reason;

    palClearPad(GPIOD, GPIOD_LED1);
    palClearPad(GPIOD, GPIOD_LED3);
    palClearPad(GPIOD, GPIOD_LED5);
    palClearPad(GPIOD, GPIOD_LED7);
    palClearPad(GPIOD, GPIOD_LED_FRONT);
    palClearPad(GPIOB, GPIOB_LED_BODY);
    
    while (true) {

    }
}

static void serial_start(void)
{
    static SerialConfig ser_cfg = {
        115200,
        0,
        0,
        0,
    };

    sdStart(&SD3, &ser_cfg); // UART3. Connected to the second com port of the programmer
}

static void timer11_start(void){
    //General Purpose Timer configuration   
    //timer 11 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt11cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD11, &gpt11cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD11, 0xFFFF);
}

static THD_WORKING_AREA(waThdFrontLed, 128);
static THD_FUNCTION(ThdFrontLed, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    while(1){
        time = chVTGetSystemTime();
        palTogglePad(GPIOD, GPIOD_LED_FRONT);
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

static THD_WORKING_AREA(waThdBodyLed, 128);
static THD_FUNCTION(ThdBodyLed, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
        palTogglePad(GPIOB, GPIOB_LED_BODY);

        /*
        *   1st case :  pause the thread during 500ms
        */
        chThdSleepMilliseconds(500);

        /*
        *   2nd case :  make the thread work during the 500ms
        */

        // //about 500ms at 168MHz
        // for(uint32_t i = 0 ; i < 21000000 ; i++){
        //     __asm__ volatile ("nop");
        // }

        /*
        *   3rd case :  make the thread work during the 500ms
        *               and block the preemption
        */

        // chSysLock();
        // for(uint32_t i = 0 ; i < 21000000 ; i++){
        //     __asm__ volatile ("nop");
        // }
        // chSysUnlock();
    }
}

void show_gravity(imu_msg_t *imu_values){

    //we create variables for the led in order to turn them off at each loop and to 
    //select which one to turn on
    uint8_t led1 = 0, led3 = 0, led5 = 0, led7 = 0;
    //threshold value to not use the leds when the robot is too horizontal
    float threshold = 0.2;
    //create a pointer to the array for shorter name
    float *accel = imu_values->acceleration;
    //variable to measure the time some functions take
    //volatile to not be optimized out by the compiler if not used
    volatile uint16_t time = 0;


    /*
    *   example 1 with trigonometry.
    */

    /*
    * Quadrant:
    *
    *       BACK
    *       ####
    *    #    0   #
    *  #            #
    * #-PI/2 TOP PI/2#
    * #      VIEW    #
    *  #            #
    *    # -PI|PI #
    *       ####
    *       FRONT
    */

    if(fabs(accel[X_AXIS]) > threshold || fabs(accel[Y_AXIS]) > threshold){

        chSysLock();
        //reset the timer counter
        GPTD11.tim->CNT = 0;
        //clock wise angle in rad with 0 being the back of the e-puck2 (Y axis of the IMU)
        float angle = atan2(accel[X_AXIS], accel[Y_AXIS]);
        //by reading time with the debugger, we can know the computational time of atan2 function
        time = GPTD11.tim->CNT;
        chSysUnlock();

        //rotates the angle by 45 degrees (simpler to compare with PI and PI/2 than with 5*PI/4)
        angle += M_PI/4;

        //if the angle is greater than PI, then it has shifted on the -PI side of the quadrant
        //so we correct it
        if(angle > M_PI){
            angle = -2 * M_PI + angle; 
        }

        if(angle >= 0 && angle < M_PI/2){
            led5 = 1;
        }else if(angle >= M_PI/2 && angle < M_PI){
            led7 = 1;
        }else if(angle >= -M_PI && angle < -M_PI/2){
            led1 = 1;
        }else if(angle >= -M_PI/2 && angle < 0){
            led3 = 1;
        }
    }

    /*
     *   example 2 with only conditions
     */

    // chSysLock();
    // GPTD11.tim->CNT = 0;

    // //we find which led of each axis should be turned on
    // if(accel[X_AXIS] > threshold)
    //     led7 = 1;
    // else if(accel[X_AXIS] < -threshold)
    //     led3 = 1;

    // if(accel[Y_AXIS] > threshold)
    //     led5 = 1;
    // else if(accel[Y_AXIS] < -threshold)
    //     led1 = 1;

    // //if two leds are turned on, turn off the one with the smaller
    // //accelerometer value
    // if(led1 && led3){
    //     if(accel[Y_AXIS] < accel[X_AXIS])
    //         led3 = 0;
    //     else
    //         led1 = 0;
    // }else if(led3 && led5){
    //     if(accel[X_AXIS] < -accel[Y_AXIS])
    //         led5 = 0;
    //     else
    //         led3 = 0;
    // }else if(led5 && led7){
    //     if(accel[Y_AXIS] > accel[X_AXIS])
    //         led7 = 0;
    //     else
    //         led5 = 0;
    // }else if(led7 && led1){
    //     if(accel[X_AXIS] > -accel[Y_AXIS])
    //         led1 = 0;
    //     else
    //         led7 = 0;
    // }
    // time = GPTD11.tim->CNT;
    // chSysUnlock();

    //to see the duration on the console
    chprintf((BaseSequentialStream *)&SD3, "time = %dus\n",time);
    //we invert the values because a led is turned on if the signal is low
    palWritePad(GPIOD, GPIOD_LED1, led1 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED3, led3 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED5, led5 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED7, led7 ? 0 : 1);

}

int main(void)
{
    /* System init */
    halInit();
    chSysInit();
    serial_start();
    timer11_start();
    i2c_start();
    imu_start();
    

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    chThdCreateStatic(waThdFrontLed, sizeof(waThdFrontLed), NORMALPRIO, ThdFrontLed, NULL);
    chThdCreateStatic(waThdBodyLed, sizeof(waThdBodyLed), NORMALPRIO, ThdBodyLed, NULL);

    //to change the priority of the thread invoking the function. The main function in this case
    //chThdSetPriority(NORMALPRIO+2);

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;

    //wait 2 sec to be sure the e-puck is in a stable position
    chThdSleepMilliseconds(2000);
    imu_compute_offset(imu_topic, NB_SAMPLES_OFFSET);

    while(1){
        //wait for new measures to be published
        messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
        //prints raw values
        chprintf((BaseSequentialStream *)&SD3, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n", 
                imu_values.acc_raw[X_AXIS], imu_values.acc_raw[Y_AXIS], imu_values.acc_raw[Z_AXIS], 
                imu_values.gyro_raw[X_AXIS], imu_values.gyro_raw[Y_AXIS], imu_values.gyro_raw[Z_AXIS]);

        //prints raw values with offset correction
        chprintf((BaseSequentialStream *)&SD3, "%Ax=%-7d Ay=%-7d Az=%-7d Gx=%-7d Gy=%-7d Gz=%-7d\r\n", 
                imu_values.acc_raw[X_AXIS]-imu_values.acc_offset[X_AXIS], 
                imu_values.acc_raw[Y_AXIS]-imu_values.acc_offset[Y_AXIS], 
                imu_values.acc_raw[Z_AXIS]-imu_values.acc_offset[Z_AXIS], 
                imu_values.gyro_raw[X_AXIS]-imu_values.gyro_offset[X_AXIS], 
                imu_values.gyro_raw[Y_AXIS]-imu_values.gyro_offset[Y_AXIS], 
                imu_values.gyro_raw[Z_AXIS]-imu_values.gyro_offset[Z_AXIS]);

        //prints values in readable units
        chprintf((BaseSequentialStream *)&SD3, "%Ax=%.2f Ay=%.2f Az=%.2f Gx=%.2f Gy=%.2f Gz=%.2f (%x)\r\n\n", 
                imu_values.acceleration[X_AXIS], imu_values.acceleration[Y_AXIS], imu_values.acceleration[Z_AXIS], 
                imu_values.gyro_rate[X_AXIS], imu_values.gyro_rate[Y_AXIS], imu_values.gyro_rate[Z_AXIS], 
                imu_values.status);
       
        show_gravity(&imu_values);
        chThdSleepMilliseconds(100);
    }

}
