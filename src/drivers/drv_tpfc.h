/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 * @file drv_tpfc.h
 *
 * Third-party flight controller driver interfaces to communicate 
 * filtered inertial navigation data to flight control thru the PX4IO
 * serial channel.
 *
 */

#ifndef _DRV_TPFC_H
#define _DRV_TPFC_H

#include <stdint.h>
#include <sys/ioctl.h>


/*
 * ioctl() definitions
 */
#define _TPFC_IOC_BASE		(0x2f00)
#define _TPFC_IOC(_n)		(_IOC(_TPFC_IOC_BASE, _n))

#define TPFC_IOC_EKF_SET		_TPFC_IOC(0)
#define TPFC_IOC_SP_SET	        	_TPFC_IOC(1)
#define TPFC_IOC_MODE_GET		_TPFC_IOC(2)
#define TPFC_IOC_INPUT_REQ_GET		_TPFC_IOC(3)
#define TPFC_IOC_FC_FW_VERSION_GET  	_TPFC_IOC(4)
#define TPFC_IOC_ESC_FW_VERSION_GET 	_TPFC_IOC(5)
#define TPFC_IOC_MAG_OFFSETS_GET 	_TPFC_IOC(6)
#define TPFC_IOC_MAG_OFFSETS_SET 	_TPFC_IOC(7)
#define TPFC_IOC_ACCEL_OFFSETS_GET 	_TPFC_IOC(8)
#define TPFC_IOC_ACCEL_OFFSETS_SET 	_TPFC_IOC(9)
#define TPFC_IOC_ACCEL_SCALE_GET 	_TPFC_IOC(10)
#define TPFC_IOC_ACCEL_SCALE_SET 	_TPFC_IOC(11)
#define TPFC_IOC_TRIM_OFFSETS_GET 	_TPFC_IOC(12)
#define TPFC_IOC_TRIM_OFFSETS_SET 	_TPFC_IOC(13)
#define TPFC_IOC_FCU_LOG_GET 		_TPFC_IOC(14)
#define TPFC_IOC_FCU_PARAM_SET		_TPFC_IOC(15)
#define TPFC_IOC_FCU_PARAM_GET		_TPFC_IOC(16)
#define TPFC_IOC_FCU_BATTERY            _TPFC_IOC(17)
#define TPFC_IOC_FCU_STATUS_LED         _TPFC_IOC(18)

typedef struct {
  float x;
  float y;
  float z;
} TpfcFloatVector;


//#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
# define PX4IO_DEVICE_PATH	"/dev/px4io"
//#endif

#endif /* _DRV_TPFC_H */

