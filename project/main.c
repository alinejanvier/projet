#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"

#include "aseba_vm/aseba_node.h"
#include "aseba_vm/skel_user.h"
#include "aseba_vm/aseba_can_interface.h"
#include "aseba_vm/aseba_bridge.h"
#include "audio/audio_thread.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "audio/microphone.h"
#include "camera/po8030.h"
#include "epuck1x/Asercom.h"
#include "epuck1x/Asercom2.h"
#include "epuck1x/a_d/advance_ad_scan/e_acc.h"
#include "sensors/battery_level.h"
#include "sensors/imu.h"
#include "sensors/mpu9250.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "cmd.h"
#include "config_flash_storage.h"
#include "exti.h"
#include "i2c_bus.h"
#include "ir_remote.h"
#include "leds.h"
#include <main.h>
#include "memory_protection.h"
#include "motors.h"
#include "sdio.h"
#include "selector.h"
#include "spi_comm.h"
#include "usbcfg.h"
#include "communication.h"
#include "uc_usage.h"
#include "audio_processing.h"

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

const int speed = 1000;
static bool bounce_enabled = false;
static bool start_enabled = true;


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

parameter_namespace_t parameter_root, aseba_ns;

static bool load_config(void)
{
    extern uint32_t _config_start;

    return config_load(&parameter_root, &_config_start);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

void turn(double angle){
	if(angle>180){
		angle-=360;
	}
	if(angle<-180){
			angle+=360;
		}

	right_motor_set_speed(0);
	left_motor_set_speed(0);
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	if(angle > 0){
		right_motor_set_speed(speed);
		left_motor_set_speed(-speed);
		while(-left_motor_get_pos()<85*(15.7*angle/360)){
			chThdSleepMilliseconds(10);
		}
	}
	else{
		right_motor_set_speed(-speed);
		left_motor_set_speed(speed);
		while(left_motor_get_pos()<85*(-15.7*angle/360)){
			chThdSleepMilliseconds(10);
		}
	}
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

double incidence_angle(){
	int sum=0;
	double angle=0;
	int z;

	// Z IS NOT CORRECT CAUSE EXPONENTIAL BUT WE WANT A PROPER RATIO?? OR GOOD? IDK

	z = get_prox(0);
	if(z < 4000 && z > 200){
		sum += z;
		angle += z*22.5;
	}
	z = get_prox(1);
	if(z < 4000 && z > 200){
		sum += z;
		angle += z*67.5;
	}
	z = get_prox(2);
	if(z < 4000 && z > 200){
		sum += z;
		angle += z*112.5;
	}
	z = get_prox(3);
	if(z < 4000 && z > 200){
		sum += z;
		angle += z*170;
	}
	z = get_prox(4);
	if(z < 4000 && z > 200){
		sum += z;
		angle += z*-170;
	}
	z = get_prox(5);
	if(z < 4000 && z > 200){
		sum += z;
		angle += z*-112.5;
	}
	z = get_prox(6);
	if(z < 4000 && z > 200){
		sum += z;
		angle += z*-67.5;
	}
	z = get_prox(7);
	if(z < 4000 && z > 200){
		sum += z;
		angle += z*-22.5;
	}

	return angle/sum;
}

bool goal(){
	int acc = get_acc(2);
	if(acc>14000){
		return true;
	}
	return false;
}


void bouncing(){
	double angle = incidence_angle();

	if(angle >= -90 && angle <=90){
		turn(180-angle*2); // A VERIFIER ???
		right_motor_set_speed(speed);
		left_motor_set_speed(speed);
	}
	else if(angle >= -180 && angle <= 180){
		turn(180-angle/2); // A VERIFIER
		right_motor_set_speed(speed);
		left_motor_set_speed(speed);
	}
}

void start(){
	for(int i = 0; i < 3; i++){
		chThdSleepMilliseconds(800);
		set_led(-1,1);
		dac_play(402);
		chThdSleepMilliseconds(300);
		set_led(-1,0);
		dac_stop();
	}
	chThdSleepMilliseconds(800);
	while(true){
		turn(15);
		if(whistle())
			break;
	}
	set_body_led(1);
	right_motor_set_speed(speed);
	left_motor_set_speed(speed);
	bounce_enabled = true;
}


static THD_WORKING_AREA(BounceThreadWorkingArea, 256);
static THD_WORKING_AREA(GoalThreadWorkingArea, 256);

static THD_FUNCTION(BounceThread, arg) {
	int z;
	chprintf((BaseSequentialStream *)&SD3,"bounce thread started");
	while(true){
		chThdSleepMilliseconds(100);
		if(bounce_enabled){
			bounce_enabled = false;
			for(int i = 0; i < 8; i++){
				z =  get_prox(i);
				if(z < 3800 && z > 200  && i != 2){ // A VERIFIER !!!!!
					set_front_led(1);
					bouncing();
					set_front_led(0);
					break;
				}
			}
			bounce_enabled = true;
		}
	}
}

static THD_FUNCTION(GoalThread, arg) {
	chprintf((BaseSequentialStream *)&SD3,"goal thread started");

	while(true){
		chThdSleepMilliseconds(100);
		if(start_enabled && goal()){
			start_enabled = false;
			bounce_enabled = false;
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			set_body_led(0);
			chThdSleepMilliseconds(5000);
			start();
			bounce_enabled = true; //also already enabled by start()
			start_enabled = true;
		}
	}
}


int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    parameter_namespace_declare(&parameter_root, NULL, NULL);

    // Init the peripherals.
	clear_leds();
	set_body_led(0);
	set_front_led(0);
	usb_start();
	//dcmi_start();
	po8030_start();
	motors_init();
	proximity_start();
	battery_level_start();
	dac_start();
	exti_start();
	imu_start();
	ir_remote_start();
	spi_comm_start();
	VL53L0X_start();
	serial_start();
	mic_start(&processAudioData);
	sdio_start();
	playMelodyStart();
	playSoundFileStart();

	// Initialise Aseba system, declaring parameters
    parameter_namespace_declare(&aseba_ns, &parameter_root, "aseba");
    aseba_declare_parameters(&aseba_ns);

    /* Load parameter tree from flash. */
    load_config();

    /* Start AsebaCAN. Must be after config was loaded because the CAN id
     * cannot be changed at runtime. */
    aseba_vm_init();
    aseba_can_start(&vmState);

    calibrate_ir();


    //OUR WORKING AREA -------------------

    start();

	(void)chThdCreateStatic(BounceThreadWorkingArea, sizeof(BounceThreadWorkingArea), NORMALPRIO, BounceThread, NULL);

	(void)chThdCreateStatic(GoalThreadWorkingArea, sizeof(GoalThreadWorkingArea), NORMALPRIO, GoalThread, NULL);


    //END OF OUR WORKING AREA ------------


}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
