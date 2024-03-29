#include <px4_config.h>
#include <px4_tasks.h>

#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <px4_time.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_baro.h>
//#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/tune_control.h>


#include <stdbool.h>
#include <stdio.h>

#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>

#include <stm32.h>
#include "board_config.h"
#include <stm32_uart.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>

#include <systemlib/px4_macros.h>

#include <px4_init.h>
#include <drivers/boards/common/board_dma_alloc.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	//PX4_INFO("Hello Sky!");
	/* subscribe to sensor_combined topic */
	
	int sensor_sub_fd  = orb_subscribe(ORB_ID(sensor_combined));
	
	/*criados por mim*/
	int sensor_sub_baro = orb_subscribe(ORB_ID(sensor_baro));
	int veihcule_sub_fd = orb_subscribe(ORB_ID(vehicle_air_data));
	
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 100);
	/*setando taxa de atualização de dados*/
	orb_set_interval(sensor_sub_baro, 100);
	
	/* advertise attitude topic */
	//struct tune_control_s att;
	//memset(&att, 0, sizeof(att));
	//orb_advert_t att_pub = orb_advertise(ORB_ID(tune_control), &att);
	/* one could wait for multiple topics with this technique, just using one here */
	
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		{ .fd = sensor_sub_baro,   .events = POLLIN },
		{ .fd = veihcule_sub_fd,   .events = POLLIN },
		
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};
	double  altRelative = 0;
	int error_counter = 0;
	int cont = 0 ;
	//int i = 10000;
	
	/*configure gpioOutput*/
	stm32_configgpio(GPIO_GPIO0_OUTPUT);
	stm32_configgpio(GPIO_GPIO1_OUTPUT);
	stm32_configgpio(GPIO_GPIO2_OUTPUT);
	stm32_configgpio(GPIO_GPIO3_OUTPUT);
	stm32_configgpio(GPIO_GPIO4_OUTPUT);
	stm32_configgpio(GPIO_GPIO5_OUTPUT);
	/*set for high ou low gpio*/
	stm32_gpiowrite(GPIO_GPIO0_OUTPUT,1);
	stm32_gpiowrite(GPIO_GPIO1_OUTPUT,1);
	stm32_gpiowrite(GPIO_GPIO2_OUTPUT,1);
	stm32_gpiowrite(GPIO_GPIO2_OUTPUT,1);
	stm32_gpiowrite(GPIO_GPIO3_OUTPUT,1);
	stm32_gpiowrite(GPIO_GPIO4_OUTPUT,1);
	stm32_gpiowrite(GPIO_GPIO5_OUTPUT,0);
	/*buffers determine variation altitude negative or positive*/
	double a1 = 0;
	double a2 = 0;
	//double a3 = 0;
	/*state 0 = idle ; state 1 = running */
	int state = 0;

	while(true){
		//a3 = a2;
		a2 = a1;
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 100);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				struct sensor_baro_s rawbaro;
				// data for altimeter in meters
				struct vehicle_air_data_s rawVeicule;   
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
				orb_copy(ORB_ID(sensor_baro), sensor_sub_baro, &rawbaro);
				orb_copy(ORB_ID(vehicle_air_data), veihcule_sub_fd, &rawVeicule);
				
				PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
					 (double)raw.accelerometer_m_s2[0],
					 (double)raw.accelerometer_m_s2[1],
					 (double)raw.accelerometer_m_s2[2]);
			        PX4_INFO("Barometer:\t%8.4f",
					 (double)rawbaro.pressure);
				PX4_INFO("Attitude: \t%8.4f",
			        	 (double)rawVeicule.baro_alt_meter);

				/* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/
				
				/* obtained data for the first file descriptor */
				
				/* copy sensors raw data into local buffer */
				   /*update the altitude actual*/
				   
			 	    a1 = rawVeicule.baro_alt_meter;
			 	    /*verify the init of airdrop*/
			 	  if(cont == 0 ){
			 	      altRelative = a1;
			 	    cont ++;
			 	  }
			 	  if((a1 - altRelative) > 2.0){
			 	    	state = 1;
			 	    }
			 	    if(state == 1){
			 	    	if(a1 < a2 ){
			 	    	   stm32_gpiowrite(GPIO_GPIO5_OUTPUT,1);
			 	    	   PX4_INFO("Apogeu was : \t%8.4f meters",
			        	 a2);

			 	    	}
			 	    }
				   // stm32_gpiowrite(GPIO_GPIO5_OUTPUT,0);
				   //px4_sleep(1);
				   //PX4_INFO("aqui");
				     
				   //px4_sleep(1);
				  // px4_sleep(2);
				   // orb_publish(ORB_ID(tune_control), att_pub, &att);
				 /* set att and publish this information for other apps
				 the following does not have any meaning, it's just an example
				*/

				//orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
			
			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}
}
	PX4_INFO("exiting");
	return 0;
}
