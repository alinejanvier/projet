#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];


#define MIN_VALUE_TRESHOLD	10000
#define MIN_FREQ	10
#define	MAX_FREQ	30
#define FREQ_START	26 //900???Hz
#define FREQ_START_L 	(FREQ_START-1)
#define FREQ_START_H	(FREQ_START+1)

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/


uint16_t frequency_processing(float* data){
	float max_norm = MIN_VALUE_TRESHOLD;

	int16_t max_norm_index = 0;

	//search for the highest peak
		for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
			if( (data[i]) > max_norm){
				max_norm = data[i];
				max_norm_index = i;
			}
		}
		return max_norm_index;
//		if (max_norm > MIN_VALUE_TRESHOLD ){
//			return max_norm_index;
//		}else{
//			return 0;
//		}
}

bool whistle(){
	if(frequency_processing(micLeft_output) >= FREQ_START_L && frequency_processing(micLeft_output) <= FREQ_START_H){
		return true;
	}
	return false;
}


void processAudioData(int16_t *data, uint16_t num_samples){
	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

		//loop to fill the buffers
		for(uint16_t i = 0 ; i < num_samples ; i+=4){
			//construct an array of complex numbers. Put 0 to the imaginary part
			micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
			micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];

			nb_samples++;

			micRight_cmplx_input[nb_samples] = 0;
			micLeft_cmplx_input[nb_samples] = 0;

			nb_samples++;

			//stop when buffer is full
			if(nb_samples >= (2 * FFT_SIZE)){
				break;
			}
		}

		if(nb_samples >= (2 * FFT_SIZE)){
			/*	FFT proccessing
			*
			*	This FFT function stores the results in the input buffer given.
			*	This is an "In Place" function.
			*/

			doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
			doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);

			/*	Magnitude processing
			*
			*	Computes the magnitude of the complex numbers and
			*	stores them in a buffer of FFT_SIZE because it only contains
			*	real numbers.
			*
			*/
			arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
			arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);


			//sends only one FFT result over 10 for 1 mic to not flood the computer
			//sends to UART3
			if(mustSend > 8){
				//signals to send the result to the computer
				chBSemSignal(&sendToComputer_sem);
				mustSend = 0;
			}
			nb_samples = 0;
			mustSend++;

		}
	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/


	
}


void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}



