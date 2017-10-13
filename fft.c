#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ifft.h>
#include "contiki.h"
#include "dev/leds.h"
#include "button-sensor.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "board-peripherals.h"
#include "rf-core/rf-ble.h"
#include "ti-lib.h"
#include "rf-core/rf-core.h"



#define SAMPLERATE 128   //less than 256
#define SENSORTAGRUNINGTIME (CLOCK_SECOND * 120)
#define DATASENDINGINTERVAL CLOCK_SECOND
#define ON 1
#define OFF 0
#define GYRP_SCALE 256
#define ACC_SCALE 2

#define DEBUG 1

const uint8_t processing = OFF;

//store the raw mpu data list
typedef struct RawData{
	int16_t gyrp_x_list[SAMPLERATE];
	int16_t gyrp_y_list[SAMPLERATE];
	int16_t gyrp_z_list[SAMPLERATE];
	int16_t acc_x_list[SAMPLERATE];
	int16_t acc_y_list[SAMPLERATE];
	int16_t acc_z_list[SAMPLERATE];
	uint8_t index;
}Data;

static Data data = {
	.index = 0
};
static Data *dataPtr = &data;

//store the result data list, size = 31 bytes
typedef struct Features{
	int16_t gyrp_x_amp;
	int16_t gyrp_y_amp;
	int16_t gyrp_z_amp;
	int16_t acc_x_amp;
	int16_t acc_y_amp;
	int16_t acc_z_amp;
	int8_t gyrp_x_freq;
	int8_t gyrp_y_freq;
	int8_t gyrp_z_freq;
	int8_t acc_x_freq;
	int8_t acc_y_freq;
	int8_t acc_z_freq;
	int16_t gyrp_x_mean;
	int16_t gyrp_y_mean;
	int16_t gyrp_z_mean;
	int16_t acc_x_mean;
	int16_t acc_y_mean;
	int16_t acc_z_mean;
	char end;
}SendBuff;

static SendBuff sendBuff = {
	.end = '\0'
};
static SendBuff * buffPtr = &sendBuff;

static int16_t xim[SAMPLERATE] = {0};//store the result of ifft array
static int8_t ximfreq[SAMPLERATE] = {0};//store the result of ifft freq array

/* Parameters: n: int Window length
 *             d: scalar Time steps
 *             result: globle variable
 * returns:    Array of length n containing the sample freq
 */
void getFreqList(uint8_t n, double d, int8_t * result){
	if (n <= 0){
		printf("Input n: %d is less or equal to ZERO.\n", n);
	}else if (n % 2 == 0){
		uint16_t halfn = n / 2;
		for (uint16_t i = 0; i < n; i++){
			if (i < halfn){
				result[i] = (int8_t) (i / (d * n));
			}else {
				result[i] = (int8_t) ((i - 2 * halfn) / (d * n));
			}
		}
	}else{
		uint16_t halfn = (n - 1) / 2;
		for (uint16_t i = 0; i < n; i++){
			if (i <= halfn){
				result[i] = (int8_t) (i / (d * n));
			}else{
				result[i] = (int8_t) ((i - 2 * halfn - 1) / (d * n));
			}
		}
	}
}

//this is v2 of getFreqList (above)
void getFreqListOfOneSecond(uint8_t n, double d, int8_t * result){
	if (n % 2 == 0){
		uint16_t halfn = n / 2;
		for (uint16_t i = 0; i < n; i++){
			if (i < halfn){
				result[i] = (int8_t) (i);
			}else {
				result[i] = (int8_t) (i - 2 * halfn);
			}
		}
	}else{
		uint16_t halfn = (n - 1) / 2;
		for (uint16_t i = 0; i < n; i++){
			if (i <= halfn){
				result[i] = (int8_t) (i);
			}else{
				result[i] = (int8_t) (i - 2 * halfn - 1);
			}
		}
	}
}
 
 
 //get index of max of ifft array, a is result array
uint8_t getMaxIfftIndex(int16_t * a, uint8_t n){
 	int16_t max = a[1];
 	uint8_t maxIndex = 1;
	for (uint8_t i = 1; i < n/2; i++){    //scan start from second item to prevent 0 freq
 		if (a[i] > max){
 			max = a[i];
 			maxIndex = i;
 		}
 	}
 	//printf("Maxindex = %d\n\r", maxIndex);
 	return maxIndex;
}


//get amplitude from sample array
int16_t getAmplitude(int16_t * in, int16_t * out, uint8_t n){
	ifft(in, out, n);
	uint8_t maxIndex= getMaxIfftIndex(in, n);
	return in[maxIndex];
}


//get ifft freq with the same index of getAmplitude()
uint8_t getFreq(uint8_t n, double d, int8_t * out, uint8_t maxIndex){
	//getFreqList(n, d, out);
	getFreqListOfOneSecond(n, d, out);
	return out[maxIndex];
}

//get max freq
uint8_t getMaxFreq(int8_t * in, uint8_t maxIndex){
	return in[maxIndex];
}

//get mean of array
int16_t getMean(int16_t * in, uint8_t n){
	int32_t sum = 0;
	for (uint8_t i = 0; i < n; i++){
		sum += (int32_t) in[i];
	}
	return (int16_t) (sum/n);
}

// process the raw data and store the result features to buff
void dataProcessing(Data * in, SendBuff * out, int16_t * xim_t, int8_t * ximfreq_t){
	uint8_t maxIndex;
	// gyrp_x
	out->gyrp_x_mean = getMean(in->gyrp_x_list, in->index);  //mean
	out->gyrp_x_amp = getAmplitude(in->gyrp_x_list, xim_t, in->index); //amp
	maxIndex = getMaxIfftIndex(in->gyrp_x_list, in->index);
	out->gyrp_x_freq = getFreq(in->index, 1/in->index, ximfreq_t, maxIndex); //freq
	
	// gyrp_y
	out->gyrp_y_mean = getMean(in->gyrp_y_list, in->index);  //mean
	out->gyrp_y_amp = getAmplitude(in->gyrp_y_list, xim_t, in->index);
	maxIndex = getMaxIfftIndex(in->gyrp_y_list, in->index);
	out->gyrp_y_freq = getMaxFreq(ximfreq_t, maxIndex); //freq, ximfreq_t does not change
	
	// gyrp_z
	out->gyrp_z_mean = getMean(in->gyrp_z_list, in->index);  //mean
	out->gyrp_z_amp = getAmplitude(in->gyrp_z_list, xim_t, in->index);
	maxIndex = getMaxIfftIndex(in->gyrp_z_list, in->index);
	out->gyrp_z_freq = getMaxFreq(ximfreq_t, maxIndex); //freq, ximfreq_t does not change
	
	// acc_x
	out->acc_x_mean = getMean(in->acc_x_list, in->index);  //mean
	out->acc_x_amp = getAmplitude(in->acc_x_list, xim_t, in->index);
	maxIndex = getMaxIfftIndex(in->acc_x_list, in->index);
	out->acc_x_freq = getMaxFreq(ximfreq_t, maxIndex); //freq, ximfreq_t does not change
	
	// acc_y
	out->acc_y_mean = getMean(in->acc_y_list, in->index);  //mean
	out->acc_y_amp = getAmplitude(in->acc_y_list, xim_t, in->index);
	maxIndex = getMaxIfftIndex(in->acc_y_list, in->index);
	out->acc_y_freq = getMaxFreq(ximfreq_t, maxIndex); //freq, ximfreq_t does not change
	
	// acc_z
	out->acc_z_mean = getMean(in->acc_z_list, in->index);  //mean
	out->acc_z_amp = getAmplitude(in->acc_z_list, xim_t, in->index);
	maxIndex = getMaxIfftIndex(in->acc_z_list, in->index);
	out->acc_z_freq = getMaxFreq(ximfreq_t, maxIndex); //freq, ximfreq_t does not change
	
	//clear all raw data
	//in->index = 0;
}


// send the result features via BLE
void dataSend(struct Features * in){
	
	int ble_buff_size = sizeof(SendBuff);
	//char ble_buff[32] = {0};
	
	//snprintf(ble_buff, ble_buff_size + 1, "%s", (char *) in); //copy the data to buff
	
	if(DEBUG){
		printf("***ble_buff_size = %d***\n\r", ble_buff_size);//test
		printf("***SendBuff = [%s]***\n\r", (char *) in);
		//printf("***ble_buff = [%s]***\n\r", ble_buff);
	}
	
	my_ble_adv_send(37, (char *) in, ble_buff_size);
	
}

// ctimer ct1 callback function
void mpu_next_reading(void *not_used)
{
//  mpu_9250_sensor.notify_ready(NULL);
	sensors_changed(&mpu_9250_sensor);
}

// print sending data
void printSendingData(struct Features * in){
	printf(
		"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n\r", 
		in->gyrp_x_amp,
		in->gyrp_y_amp,
		in->gyrp_z_amp,
		in->acc_x_amp,
		in->acc_y_amp,
		in->acc_z_amp,
		in->gyrp_x_freq,
		in->gyrp_y_freq,
		in->gyrp_z_freq,
		in->acc_x_freq,
		in->acc_y_freq,
		in->acc_z_freq,
		in->gyrp_x_mean,
		in->gyrp_y_mean,
		in->gyrp_z_mean,
		in->acc_x_mean,
		in->acc_y_mean,
		in->acc_z_mean
	);
}

//print int16_t list
void printList(int16_t * list, uint8_t n){
	for(uint8_t i = 0; i < n; i++){
		printf(" %d,", list[i]);
	}
	printf("\n\r");
}

//print raw data
void printRawData(Data * in){
	printf("**************************************\n\r");
	printf("gyrp_x_list = [");
	printList(in->gyrp_x_list, in->index);
	printf("]\n\r");
	printf("gyrp_y_list = [");
	printList(in->gyrp_y_list, in->index);
	printf("]\n\r");
	printf("gyrp_z_list = [");
	printList(in->gyrp_z_list, in->index);
	printf("]\n\r");
	printf("acc_x_list = [");
	printList(in->acc_x_list, in->index);
	printf("]\n\r");
	printf("acc_y_list = [");
	printList(in->acc_y_list, in->index);
	printf("]\n\r");
	printf("acc_z_list = [");
	printList(in->acc_z_list, in->index);
	printf("]\n\r");
	printf("Index = %d\n\r", in->index);
	printf("**************************************\n\r");
}
 
 
//test

void test(){
	int16_t xre[9] = {-2,4,32,-40,35,-4,-27,25,-47};
	int16_t amp = getAmplitude(xre, xim, 9);
	uint8_t maxIndex = getMaxIfftIndex(xre, 9);
	uint8_t freq = getFreq(9, 1/9, ximfreq, maxIndex);
	printf("\r###############################\n\r");
	printf("The amp is %d.\n\rThe freq is %d.\n\r", amp, freq);
	printf("###############################\n\r");
	printf("fft:\n\r");
	for (uint8_t i = 0; i < 9; i++){
		printf(" %d", xre[i]);
	}
	printf("\n\r");
	printf("fftfreq:\n\r");
	for (uint8_t i = 0; i < 9; i++){
		printf(" %d", ximfreq[i]);
	}
	printf("\n\r");
}

/*---------------------------------------------------------------------------*/
PROCESS(button_input_process, "Button process");
AUTOSTART_PROCESSES(&button_input_process);
/*---------------------------------------------------------------------------*/


//Button INput Thread
PROCESS_THREAD(button_input_process, ev, data) {
	
	static struct etimer et1;	// etimer to control sending
	static struct ctimer ct1;	// ctimer to control reading mpu

	static uint8_t MPU_SWITCH = OFF;	//press left button to toggle the switch
	
  	PROCESS_BEGIN();	//Start of thread

	leds_on(LEDS_GREEN); // Green On
	leds_off(LEDS_RED);  // Red Off
	rf_ble_beacond_config(0, NULL); // BLE init


	//Processing loop of thread
	while (1) {

        //Wait for event to occur
		PROCESS_YIELD();
  		
        //Check if sensor event has occured
		if(ev == sensors_event) {
            
            //Check if left push button event has occured
      		if(data == &button_left_sensor) {
      		
        		printf("\n\rLeft: Pin %d, press duration %d clock ticks\n", button_left_sensor.value(BUTTON_SENSOR_VALUE_STATE), button_left_sensor.value(BUTTON_SENSOR_VALUE_DURATION));

                //test();//test function
                
                if(MPU_SWITCH == OFF){	// switch on the device if device is off
                
                	leds_on(LEDS_RED);
                	dataPtr->index = 0;
                	mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
                	etimer_set(&et1, DATASENDINGINTERVAL);	// One second interval
                	ctimer_set(&ct1, (CLOCK_SECOND/SAMPLERATE + 1), mpu_next_reading, NULL);// 128 readings per Second
                	MPU_SWITCH = ON;

                }else{					// switch off the device if device is on
                
                	etimer_stop(&et1);
                	ctimer_stop(&ct1);
                	SENSORS_DEACTIVATE(mpu_9250_sensor);
                	dataPtr->index = 0; //clear the data buff
                	leds_off(LEDS_RED);
                	MPU_SWITCH = OFF;
                }
                
			}if(data == &mpu_9250_sensor){
			
				if (dataPtr->index < SAMPLERATE){
				
					//printf("mpu sensor event\n\r");
					//collect mpu sensor readings and store in data
					dataPtr->gyrp_x_list[dataPtr->index] = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X) / GYRP_SCALE;
					dataPtr->gyrp_y_list[dataPtr->index] = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y) / GYRP_SCALE;
					dataPtr->gyrp_z_list[dataPtr->index] = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z) / GYRP_SCALE;
					dataPtr->acc_x_list[dataPtr->index] = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X) / ACC_SCALE;
					dataPtr->acc_y_list[dataPtr->index] = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y) / ACC_SCALE;
					dataPtr->acc_z_list[dataPtr->index] = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X) / ACC_SCALE;
					dataPtr->index++;
				}

				ctimer_reset(&ct1);
				
			}
			
		}else if(ev == PROCESS_EVENT_TIMER){
		
			if(data == &et1){
			
				if(DEBUG){
					printRawData(dataPtr);
				}
				
				leds_toggle(LEDS_RED);
				
				if(dataPtr->index <= SAMPLERATE){
					
					//to process the data AND clear the data buffer
					dataProcessing(dataPtr, buffPtr, xim, ximfreq);
					
					//debug print
					if(DEBUG){
						printf("SendBuff = [");
						printSendingData(buffPtr);
						printf("]\n\r");
					}
					
					//clear the raw data index
					dataPtr->index = 0;
					
					//send out the features by BLE
					dataSend(buffPtr);
				}
				
				etimer_reset(&et1);
				
			}
			
			ctimer_restart(&ct1);
			
		}
    }

  	PROCESS_END();		//End of thread
}
 
 
 
 
 
 
 
 
