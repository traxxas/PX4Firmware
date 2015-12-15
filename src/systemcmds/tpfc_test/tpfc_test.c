/****************************************************************************
*
* Copyright (c) 2014 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
* used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

/**
* @file tpfc_test.c
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <version/version.h>
#include <systemlib/err.h>
#include <stm32_bkp.h>


static void usage(const char *reason)
{
	if (reason != NULL) {
		printf("%s\n", reason);
	}

	errx(1,
	     "usage: tpfc_test <send|display_ekf|display_err|display_timings|jerk|clearsm|newstop|newbaro|rtl_brake|rtl_brake_duration|rtl_brake_velocity [num]\n  num defaults to 0, which is OFF\n\n");
}

extern int tpfc_send_ekf_data;

extern int tpfc_new_loiter_stop_algorithm;
extern int tpfc_baro_new_algorithm;

extern uint32_t tpfc_rtl_brake;
extern uint32_t tpfc_rtl_brake_duration_ms;
extern uint32_t tpfc_rtl_brake_velocity_cms;


extern int tpfc_display_ekf_data;
extern int tpfc_display_error_data;
// extern int g_print_serial_msg;
	    
extern int tpfc_read_calls;  
extern int tpfc_read_retries;
extern uint8_t tpfc_read_last_clean_page;
extern int tpfc_read_good;  
extern int tpfc_read_bad;  

extern int tpfc_send_calls;
extern int tpfc_send_retries;
extern int tpfc_send_with_zeroes;
extern uint8_t tpfc_send_last_clean_page;
extern int tpfc_send_error_dma;
extern int tpfc_send_error_crc;
extern int tpfc_send_error_timeout;
extern int tpfc_send_error_overruns;
extern int tpfc_send_error_noise;
extern int tpfc_send_error_framing;
extern int tpfc_send_error_bad_idle;
extern int tpfc_send_error_waiting_idle;
extern uint32_t tpfc_last_dma_sr;
extern uint32_t tpfc_last_ok_sr;
extern unsigned tpfc_last_rx_status;
extern unsigned tpfc_last_ok_rx_status;
extern int tpfc_interrupt_count;

extern bool tpfc_print_loop_timings;

extern float   tpfc_jerk_ratio;



__EXPORT int tpfc_test_main(int argc, char *argv[]);

int tpfc_test_main(int argc, char *argv[])
{

  if (argc != 2 && argc != 3)  {
    usage(NULL);
  }
  
  int setting = 0;
  
  if (argv[1])  {
    if (argv[2]) {
      setting = atoi(argv[2]);
    }
    
    if (strcmp(argv[1], "send") == 0) {
	tpfc_send_ekf_data = setting;
      }
      else if (strcmp(argv[1], "display_ekf") == 0) {
	tpfc_display_ekf_data = setting;
      }
      else if (strcmp(argv[1], "display_err") == 0) {
	tpfc_display_error_data = setting;
      }
      else if (strcmp(argv[1], "display_timings") == 0) {
	tpfc_print_loop_timings = true;
      }
      else if (strcmp(argv[1], "show") == 0) {
#if 0
	printf("calls r:%d, w:%d  retries r:%d  w:%d  lcp r:0x%02x w:0x%02x  reads good:%d bad:%d\n    errors, szd:%d, dma:%d, crc:%d, timeout:%d, ovr:%d, ne:%d, fe:%d, bi:%d wi:%d\n",
	       tpfc_read_calls, tpfc_send_calls, tpfc_read_retries, tpfc_send_retries, tpfc_read_last_clean_page, tpfc_send_last_clean_page, tpfc_read_good, tpfc_read_bad,
	       tpfc_send_with_zeroes,
	       tpfc_send_error_dma, tpfc_send_error_crc, tpfc_send_error_timeout,
	       tpfc_send_error_overruns, tpfc_send_error_noise, tpfc_send_error_framing, tpfc_send_error_bad_idle, tpfc_send_error_waiting_idle);


	printf("  interrupt cnt:%d  last sr:%d  last ok sr:%d  last rx status:%d  last ok status:%d\n",
	       tpfc_interrupt_count, tpfc_last_dma_sr, tpfc_last_ok_sr, tpfc_last_rx_status, tpfc_last_ok_rx_status);
#endif
	//	printf("Z jerk ratio: %2.2f\n", tpfc_jerk_ratio);

	printf("SM erase all: %d, FRAM restores: %d\n",
	       (*(uint32_t *) STM32_BKP_DR9),
	       (*(uint32_t *) STM32_BKP_DR10));

	printf("New loiter stop: %d, new baro:%d  RTL brake:%d, duration:%d, velocity:%d\n", 
	       tpfc_new_loiter_stop_algorithm,
	       tpfc_baro_new_algorithm,
	       tpfc_rtl_brake,
	       tpfc_rtl_brake_duration_ms,
	       tpfc_rtl_brake_velocity_cms);
      }
      else if (strcmp(argv[1], "clearsm") == 0) {
	(*(uint32_t *) STM32_BKP_DR9) = 0;
	(*(uint32_t *) STM32_BKP_DR10) = 0;
      }
      else if (strcmp(argv[1], "jerk") == 0) {
	tpfc_jerk_ratio = atof(argv[2]);
      }
      else if (strcmp(argv[1], "newstop") == 0) {
	tpfc_new_loiter_stop_algorithm = setting;
      }
      else if (strcmp(argv[1], "newbaro") == 0) {
	tpfc_baro_new_algorithm = setting;
      }
      else if (strcmp(argv[1], "rtl_brake") == 0) {
	tpfc_rtl_brake = setting;
      }
      else if (strcmp(argv[1], "rtl_brake_duration") == 0) {
	tpfc_rtl_brake_duration_ms = setting;
      }
      else if (strcmp(argv[1], "rtl_brake_velocity") == 0) {
	tpfc_rtl_brake_velocity_cms = setting;
      }
  }
  
  //  printf("tpfc:test: num is %d\n", setting);
  //  printf("sizoef IOPacket is %d\n", sizeof(struct IOPacket));


  // g_print_serial_msg = setting;
  return 0;
}
