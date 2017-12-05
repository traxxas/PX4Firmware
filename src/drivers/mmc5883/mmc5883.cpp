/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file mmc5883.cpp
 *
 * Driver for the MMC5883MA magnetometer connected via I2C. Based on the HMC5883 
 * driver.
 */

#include <px4_config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <board_config.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_device.h>

#include <uORB/uORB.h>

#include <float.h>
#include <getopt.h>
#include <lib/conversion/rotation.h>

#include "mmc5883.h"

/*
 * MMC5883 internal constants and data structures.
 */

/* Measurement rate is 200Hz */
#define MMC5883_CONVERSION_INTERVAL  (1000000/200)  /* microseconds */
#define MMC5883_SET_RESET_DELAY      1200           /* microseconds */

#define MMC5883_TEMP_CHECK_INTERVAL  1000           /* mag measures per temp measure */

#define MMC5883_COUNTS_PER_GAUSS     4096           /* Constant gain */
#define MMC5883_NULL_FIELD_OUTPUT    32768          /* Counts at 0 Gauss */

#define MMC5883_DEGREES_PER_COUNT    0.78f          /* Deg C per count for temperature */
#define MMC5883_TEMP_AT_ZERO_COUNTS  -75            /* 0 counts is -75 deg C */

/* Register addresses */
#define REG_ADDR_DATA_OUT_X_LSB		0x00
#define REG_ADDR_DATA_OUT_X_MSB		0x01
#define REG_ADDR_DATA_OUT_Y_LSB		0x02
#define REG_ADDR_DATA_OUT_Y_MSB		0x03
#define REG_ADDR_DATA_OUT_Z_LSB		0x04
#define REG_ADDR_DATA_OUT_Z_MSB		0x05
#define REG_ADDR_TEMPERATURE		0x06
#define REG_ADDR_STATUS			0x07

#define REG_ADDR_CONTROL_0		0x08
#define REG_ADDR_CONTROL_1		0x09
#define REG_ADDR_CONTROL_2		0x0A

/* Register Masks */
#define REG_MASK_STATUS_M_DONE                  0x01
#define REG_MASK_STATUS_T_DONE                  0x02
#define REG_MASK_STATUS_PUMP_ON                 0x08

#define REG_MASK_CONTROL_0_TM_M                 0x01
#define REG_MASK_CONTROL_0_TM_T                 0x02
#define REG_MASK_CONTROL_0_SET                  0x08
#define REG_MASK_CONTROL_0_RESET                0x10
#define REG_MASK_CONTROL_0_RESERVED             0x20

#define REG_MASK_CONTROL_1_BANDWIDTH_0          0x01
#define REG_MASK_CONTROL_1_BANDWIDTH_1          0x02
#define REG_MASK_CONTROL_1_SW_RESET             0x80


enum MMC5883_BUS {
	MMC5883_BUS_ALL = 0,
	MMC5883_BUS_I2C_INTERNAL,
	MMC5883_BUS_I2C_EXTERNAL
};

enum MMC5883_MEASURE_STATE {
  MMC5883_MEASURE_TEMP,    // next poll will measure temperature
  MMC5883_MEASURE_FIELD,   // next poll will measure mag field
  MMC5883_COLLECT          // read and store the measurements
};


/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class MMC5883 : public device::CDev
{
public:
	MMC5883(device::Device *interface, const char *path, enum Rotation rotation);
	virtual ~MMC5883();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	Device			*_interface;

private:
        work_s			_hp_work;
	unsigned		_measure_ticks;

	ringbuffer::RingBuffer	*_reports;
	mag_scale		_scale;
	float 			_range_scale;
	int			_class_instance;
	int			_orb_class_instance;
        MMC5883_MEASURE_STATE   _measure_state;
        uint16_t                _check_temp_counter;
        bool                    _temp_compensation;

	orb_advert_t		_mag_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;
	perf_counter_t		_field_read_errors;
	perf_counter_t		_conf_errors;

	bool			_calibrated;		/**< the calibration is valid */

	enum Rotation		_rotation;

	struct mag_report	_last_report;           /**< used for info() */


	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop();

	/**
	 * Reset the device
	 */
	int			reset();

	/**
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *	 will however reflect the uncalibrated sensor state until
	 *	 the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test strap, 0 to disable
	 */
	int			calibrate(struct file *filp, unsigned enable);

	/**
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *	 will however reflect the uncalibrated sensor state until
	 *	 the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test positive strap, -1 to enable
	 *        negative strap, 0 to set to normal mode
	 */
	int			set_excitement(unsigned long enable);



	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			measurement_cycle();

	/**
	 * Static trampoline from the high priority workq context
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		hp_workq_callback(void *arg);

	/**
	 * Write a register.
	 *
	 * @param reg		The register to write.
	 * @param val		The value to write.
	 * @return		OK on write success.
	 */
	int			write_reg(uint8_t reg, uint8_t val);

	/**
	 * Read a register.
	 *
	 * @param reg		The register to read.
	 * @param val		The value read.
	 * @return		OK on read success.
	 */
	int			read_reg(uint8_t reg, uint8_t &val);

       /**
	 * Issue a measurement command.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			measure(MMC5883_MEASURE_STATE type);

	/**
	 * Collect the result of the most recent measurement, store it 
         * in the provided mag_report structure.
	 */
        int                     collect(struct mag_report &new_report);

       /**
	 * Collect the result of the most recent measurement, and publish it 
         * to the ORB topic and the report queue.
	 */
        int                     collect_and_publish();

	/**
	 * Convert a big-endian signed 16-bit value to a float.
	 *
	 * @param in		A signed 16-bit big-endian value.
	 * @return		The floating-point representation of the value.
	 */
	float			meas_to_float(uint8_t in[2]);

	/**
	 * Check the current calibration and update device status
	 *
	 * @return 0 if calibration is ok, 1 else
	 */
	 int 			check_calibration();

	 /**
	 * Check the current scale calibration
	 *
	 * @return 0 if scale calibration is ok, 1 else
	 */
	 int 			check_scale();

	 /**
	 * Check the current offset calibration
	 *
	 * @return 0 if offset calibration is ok, 1 else
	 */
	 int 			check_offset();

	/* this class has pointer data members, do not allow copying it */
	MMC5883(const MMC5883&);
	MMC5883 operator=(const MMC5883&);
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int mmc5883_main(int argc, char *argv[]);


MMC5883::MMC5883(device::Device *interface, const char *path, enum Rotation rotation) :
	CDev("MMC5883", path),
	_interface(interface),
	_hp_work{},
	_measure_ticks(0),
	_reports(nullptr),
	_scale{},
	_range_scale(1.0f/MMC5883_COUNTS_PER_GAUSS), /* default range scale, counts to gauss */
	_class_instance(-1),
	_orb_class_instance(-1),
	_measure_state(MMC5883_MEASURE_TEMP),
	_check_temp_counter(MMC5883_TEMP_CHECK_INTERVAL),
	_temp_compensation(false),
	_mag_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "mmc5883_read")),
	_comms_errors(perf_alloc(PC_COUNT, "mmc5883_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "mmc5883_buffer_overflows")),
	_field_read_errors(perf_alloc(PC_COUNT, "mmc5883_field_read_errors")),
	_conf_errors(perf_alloc(PC_COUNT, "mmc5883_conf_errors")),
	_calibrated(false),
	_rotation(rotation),
	_last_report{0}
{
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_MMC5883;

	// enable debug() calls
	_debug_enabled = false;

	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_hp_work, 0, sizeof(_hp_work));
}

MMC5883::~MMC5883()
{
	/* make sure we are truly inactive */
	stop();

	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _class_instance);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);
	perf_free(_field_read_errors);
	perf_free(_conf_errors);
}

int
MMC5883::init()
{
	int ret = ERROR;

	ret = CDev::init();

	if (ret != OK) {
		debug("CDev init failed");
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));

	if (_reports == nullptr) {
		goto out;
	}

	/* reset the device configuration */
	reset();

	_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	ret = OK;

 out:
	return ret;
}




ssize_t
MMC5883::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct mag_report);
	struct mag_report *mag_buf = reinterpret_cast<struct mag_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {
		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(mag_buf)) {
				ret += sizeof(struct mag_report);
				mag_buf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	/* XXX really it'd be nice to lock against other readers here */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure(MMC5883_MEASURE_FIELD)) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(MMC5883_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect_and_publish()) {
			ret = -EIO;
			break;
		}

		if (_reports->get(mag_buf)) {
			ret = sizeof(struct mag_report);
		}
	} while (0);
	
	return ret;
}

int
MMC5883::ioctl(struct file *filp, int cmd, unsigned long arg)
{
  unsigned dummy = arg;

  switch (cmd) {
    case SENSORIOCSPOLLRATE: {
      switch (arg) {

	/* switching to manual polling */
	case SENSOR_POLLRATE_MANUAL:
	  stop();
	  _measure_ticks = 0;
	  return OK;

	  /* external signalling (DRDY) not supported */
	case SENSOR_POLLRATE_EXTERNAL:

	  /* zero would be bad */
	case 0:
	  return -EINVAL;

	  /* set default/max polling rate */
	case SENSOR_POLLRATE_MAX:
	case SENSOR_POLLRATE_DEFAULT: {
	  /* do we need to start internal polling? */
	  bool want_start = (_measure_ticks == 0);

	  /* set interval for next measurement to minimum legal value */
	  _measure_ticks = USEC2TICK(MMC5883_CONVERSION_INTERVAL);

	  int ret = write_reg(REG_ADDR_CONTROL_1, REG_MASK_CONTROL_1_BANDWIDTH_0);
	  
	  /* if we need to start the poll state machine, do it */
	  if (want_start) {
	    start();
	  }

	  return ret;
	}

	  /* adjust to a legal polling interval in Hz */
	default: {
	  /* do we need to start internal polling? */
	  bool want_start = (_measure_ticks == 0);

	  /* convert hz to tick interval via microseconds */
	  unsigned ticks = USEC2TICK(1000000 / arg);

	  /* check against maximum rate */
	  if (ticks < USEC2TICK(MMC5883_CONVERSION_INTERVAL)) {
	    return -EINVAL;
	  }

	  /* update interval for next measurement */
	  _measure_ticks = ticks;

	  /* if we need to start the poll state machine, do it */
	  if (want_start) {
	    start();
	  }

	  return OK;
	}
      }
    }

    case SENSORIOCGPOLLRATE:
      if (_measure_ticks == 0) {
	return SENSOR_POLLRATE_MANUAL;
      }

      return 1000000/TICK2USEC(_measure_ticks);

    case SENSORIOCSQUEUEDEPTH: {
      /* lower bound is mandatory, upper bound is a sanity check */
      if ((arg < 1) || (arg > 100)) {
	return -EINVAL;
      }

      irqstate_t flags = irqsave();

      if (!_reports->resize(arg)) {
	irqrestore(flags);
	return -ENOMEM;
      }

      irqrestore(flags);

      return OK;
    }

    case SENSORIOCGQUEUEDEPTH:
      return _reports->size();

    case SENSORIOCRESET:
      return reset();

    case MAGIOCSSAMPLERATE:
      /* same as pollrate because device is in single measurement mode*/
      return ioctl(filp, SENSORIOCSPOLLRATE, arg);

    case MAGIOCGSAMPLERATE:
      /* same as pollrate because device is in single measurement mode*/
      return 1000000/TICK2USEC(_measure_ticks);

    case MAGIOCSRANGE:
      return OK;  /* Gain is constant for this part */

    case MAGIOCGRANGE:
      return MMC5883_COUNTS_PER_GAUSS;

    case MAGIOCSLOWPASS:
    case MAGIOCGLOWPASS:
      /* not supported, no internal filtering */
      return -EINVAL;

    case MAGIOCSSCALE:
      /* set new scale factors */
      memcpy(&_scale, (mag_scale *)arg, sizeof(_scale));
      /* check calibration, but not actually return an error */
      (void)check_calibration();
      return 0;

    case MAGIOCGSCALE:
      /* copy out scale factors */
      memcpy((mag_scale *)arg, &_scale, sizeof(_scale));
      return 0;

    case MAGIOCCALIBRATE:
      return calibrate(filp, arg);

    case MAGIOCEXSTRAP:
      return set_excitement(arg);

    case MAGIOCSELFTEST:
      return check_calibration();

    case MAGIOCGEXTERNAL:
      debug("MAGIOCGEXTERNAL in main driver");
      return _interface->ioctl(cmd, dummy);

    case MAGIOCSTEMPCOMP:
      if (arg) {
	_temp_compensation = true;
      }
      else {
	_temp_compensation = false;
      }
      return 0;

    case DEVIOCGDEVICEID:
      return _interface->ioctl(cmd, dummy);

    default:
      /* give it to the superclass */
      return CDev::ioctl(filp, cmd, arg);
  }
}

void
MMC5883::start()
{
  /* reset the report ring and state machine */
  _measure_state = MMC5883_MEASURE_TEMP;
  _check_temp_counter = MMC5883_TEMP_CHECK_INTERVAL;
  _reports->flush();

  /* schedule callbacks to start things */
  work_queue(HPWORK, &_hp_work, (worker_t)&MMC5883::hp_workq_callback, this, USEC2TICK(1000));
}


void
MMC5883::stop()
{
  work_cancel(HPWORK, &_hp_work);
}

int
MMC5883::reset()
{
  /* FIXME: The HMC driver just set the gain here.  Anything to do
     here for MMC?  Note this method is called before the device is
     discovered so can't send anything on I2C.
  */
  return OK;
}

void
MMC5883::hp_workq_callback(void *arg)
{
	MMC5883 *dev = (MMC5883 *)arg;

	dev->measurement_cycle();
}

void
MMC5883::measurement_cycle()
{
  /* collection phase? */
  if (_measure_state == MMC5883_COLLECT) {

    /* perform collection */
    if (OK != collect_and_publish()) {
      debug("collection error");

      /* restart the measurement state machine */
      start();
      return;
    }

    if (_check_temp_counter == 0) {
      _measure_state = MMC5883_MEASURE_TEMP;
    }
    else {
      _check_temp_counter--;
      _measure_state = MMC5883_MEASURE_FIELD;
    }

    /* 
     * Is there a collect->measure gap?
     */
    if (_measure_ticks > USEC2TICK(MMC5883_CONVERSION_INTERVAL)) {

      /* schedule a fresh cycle call when we are ready to measure again */

      work_queue(HPWORK,
		 &_hp_work,
		 (worker_t)&MMC5883::hp_workq_callback,
		 this,
		 _measure_ticks - USEC2TICK(MMC5883_CONVERSION_INTERVAL));

      return;
    }
  }

  /* measurement phase */
  if (OK != measure(_measure_state)) {
    debug("measure error");
  }

  // If we measured the temperature, reset the counter.
  //
  if (_measure_state == MMC5883_MEASURE_TEMP) {
    _check_temp_counter = MMC5883_TEMP_CHECK_INTERVAL;
  }

  // Next stage is collect.
  //
  _measure_state = MMC5883_COLLECT;

  /* schedule a fresh cycle call when the measurement is done */
  work_queue(HPWORK,
	     &_hp_work,
	     (worker_t)&MMC5883::hp_workq_callback,
	     this,
	     USEC2TICK(MMC5883_CONVERSION_INTERVAL));
}

int
MMC5883::measure(MMC5883_MEASURE_STATE type)
{
  int ret;
  uint8_t regValue = REG_MASK_CONTROL_0_TM_M;  // assume measuring mag field

  // What do we need to measure?
  //
  if (type == MMC5883_MEASURE_TEMP) {
    regValue = REG_MASK_CONTROL_0_TM_T;
  }

  /*
   * Send the command to begin a measurement.
   */
  ret = write_reg(REG_ADDR_CONTROL_0, regValue);
  if (OK != ret) {
    perf_count(_comms_errors);
  }
  
  return ret;
}


int
MMC5883::collect(struct mag_report &new_report)
{
#pragma pack(push, 1)
  struct { /* status register and data as read back from the device */
    uint16_t	x;
    uint16_t	y;
    uint16_t	z;
    uint8_t     temperature;
    uint8_t     status;
  } mmc_report;
#pragma pack(pop)

  struct {
    int16_t		x, y, z;
  } report;

  int	ret;
  //  uint8_t check_counter;

  bool sensor_is_onboard = false;

  float xraw_f;
  float yraw_f;
  float zraw_f;

  memset(&new_report, 0, sizeof(new_report));
  
  /* this should be fairly close to the end of the measurement, so the best approximation of the time */
  new_report.timestamp = hrt_absolute_time();
  new_report.error_count = perf_event_count(_comms_errors);

  /* get measurements from the device.  We are reading data plus
     the status register that will let us know if the data is
     valid. Easier than doing incremental reads. 
  */
  ret = _interface->read(REG_ADDR_DATA_OUT_X_LSB, (uint8_t *)&mmc_report, sizeof(mmc_report));

  bool validFieldMeasurement = ((mmc_report.status & REG_MASK_STATUS_M_DONE) == REG_MASK_STATUS_M_DONE);
  bool validTempMeasurement = ((mmc_report.status & REG_MASK_STATUS_T_DONE) == REG_MASK_STATUS_T_DONE);

  if (ret != OK) {
    perf_count(_comms_errors);
    debug("data/status read error");
    goto out;
  }

  // Clear the measurement indicator bits.
  write_reg(REG_ADDR_STATUS, (REG_MASK_STATUS_M_DONE | REG_MASK_STATUS_T_DONE));

  /* Do did we get a valid measurement? */
  if (!validFieldMeasurement && !validTempMeasurement) {
    ret = 1;
    perf_count(_field_read_errors);
    goto out;
  }

  report.x = mmc_report.x - MMC5883_NULL_FIELD_OUTPUT;
  report.y = mmc_report.y - MMC5883_NULL_FIELD_OUTPUT;
  report.z = mmc_report.z - MMC5883_NULL_FIELD_OUTPUT;

  /* Check temperature measurement */
  if (validTempMeasurement) {
    // Linear conversion from counts to degree, y=mx+b
    // Note: datasheet gives a rate (m) and a range that do not match.
    new_report.temperature = mmc_report.temperature * MMC5883_DEGREES_PER_COUNT + MMC5883_TEMP_AT_ZERO_COUNTS;
  } else {
    new_report.temperature = _last_report.temperature;
  }


  /*
   * RAW outputs
   *
   * to align the sensor axes with the board, x and y need to be flipped
   * and y needs to be negated.
   */
  new_report.x_raw = report.y;
  new_report.y_raw = -report.x;
  new_report.z_raw = report.z;

  /* scale values for output */

  // XXX revisit for SPI part, might require a bus type IOCTL
  unsigned dummy;
  sensor_is_onboard = !_interface->ioctl(MAGIOCGEXTERNAL, dummy);

  if (sensor_is_onboard) {
    // convert onboard so it matches offboard for the
    // scaling below
    report.y = -report.y;
    report.x = -report.x;
  }

  /* the standard external mag by 3DR has x pointing to the
   * right, y pointing backwards, and z down, therefore switch x
   * and y and invert y */
  xraw_f = -report.y;
  yraw_f = report.x;
  zraw_f = -report.z;

  // apply user specified rotation
  rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

  new_report.x = ((xraw_f * _range_scale) - _scale.x_offset) * _scale.x_scale;
  new_report.y = ((yraw_f * _range_scale) - _scale.y_offset) * _scale.y_scale;
  new_report.z = ((zraw_f * _range_scale) - _scale.z_offset) * _scale.z_scale;

  ret = OK;

 out:
  
  return ret;
}


int
MMC5883::collect_and_publish()
{
  struct mag_report new_report;

  perf_begin(_sample_perf);

  int ret = collect(new_report);

  if (ret == OK) {

    unsigned dummy;
    bool sensor_is_onboard = !_interface->ioctl(MAGIOCGEXTERNAL, dummy);

    // Is temperature compensation enabled?
    if (_temp_compensation) {
      //
      // NOTE: Not implemented.  If this is needed the algorithm would
      // be to recalibrate when the temperature changes by a given
      // threshold.  Psuedocode is below....
      //
      //      if (abs(new_report.temperature - _last_report.temperature) > MMC5883_RECALIB_DELTA_T) {
      //        if (sensor_is_onboard) {
      //          busid = I2C internal
      //        } else {
      //          busid = I2C external;
      //        }
      //        calibrate(busid);
    }

    _last_report = new_report;
    
    if (!(_pub_blocked)) {

      if (_mag_topic != nullptr) {
	/* publish it */
	orb_publish(ORB_ID(sensor_mag), _mag_topic, &new_report);
      } else {
	_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &new_report,
					 &_orb_class_instance, (sensor_is_onboard) ? ORB_PRIO_HIGH : ORB_PRIO_MAX);

	if (_mag_topic == nullptr) {
	  debug("ADVERT FAIL");
	}
      }
    }

    /* post the report to the ring */
    if (_reports->force(&new_report)) {
      perf_count(_buffer_overflows);
    }

    /* notify anyone waiting for data */
    poll_notify(POLLIN);
  }

  perf_end(_sample_perf);

  return ret;
}


int MMC5883::calibrate(struct file *filp, unsigned enable)
{
  struct mag_report negativeBiasSample;
  struct mag_report positiveBiasSample;
  
  int ret = 1;

  struct mag_scale mscale_previous = {
    0.0f,
    1.0f,
    0.0f,
    1.0f,
    0.0f,
    1.0f,
  };

  struct mag_scale mscale_null = {
    0.0f,
    1.0f,
    0.0f,
    1.0f,
    0.0f,
    1.0f,
  };

  // Go into manual measure mode.
  stop();

  // Retrieve existing calibration and set to defaults
  if (OK != ioctl(filp, MAGIOCGSCALE, (long unsigned int)&mscale_previous)) {
    warn("FAILED: MAGIOCGSCALE 1");
    ret = 1;
    goto out;
  }

  if (OK != ioctl(filp, MAGIOCSSCALE, (long unsigned int)&mscale_null)) {
    warn("FAILED: MAGIOCSSCALE 1");
    ret = 1;
    goto out;
  }

  
  // Apply negative bias
  if (OK != ioctl(filp, MAGIOCEXSTRAP, -1)) {
    warnx("FAILED: MAGIOCEXSTRAP -1");
    ret = 1;
    goto out;
  }

  // Allow bias to take effect
  usleep(MMC5883_SET_RESET_DELAY);
  
  // Discard a few samples 
  for (uint8_t i = 0; i < 2; ++i) {
    if (OK != measure(MMC5883_MEASURE_FIELD)) {
      warnx("negative bias: measure() failed");
      ret = 1;
      goto out;
    }
    
    // Wait for data...
    usleep(MMC5883_CONVERSION_INTERVAL);

    if (OK != collect(negativeBiasSample)) {
      warnx("negative bias: collect() failed");
      ret = 1;
      goto out;
    }
  }


  // Positive bias
  if (OK != ioctl(filp, MAGIOCEXSTRAP, 1)) {
    warnx("FAILED: MAGIOCEXSTRAP 1");
    ret = 1;
    goto out;
  }

  // Allow bias to take effect
  usleep(MMC5883_SET_RESET_DELAY);
  
  // Discard a few samples 
  for (uint8_t i = 0; i < 2; ++i) {
    if (OK != measure(MMC5883_MEASURE_FIELD)) {
      warnx("positive bias: measure() failed");
      ret = 1;
      goto out;
    }
    
    // Wait for data...
    usleep(MMC5883_CONVERSION_INTERVAL);

    if (OK != collect(positiveBiasSample)) {
      warnx("positive bias: collect() failed");
      ret = 1;
      goto out;
    }
  }

  // FIXME: remove printfs
  //  printf("negative output  (%.3f %.3f %.3f)\n", (double)negativeBiasSample.x, (double)negativeBiasSample.y, (double)negativeBiasSample.z);
  //  printf("positive output  (%.3f %.3f %.3f)\n", (double)positiveBiasSample.x, (double)positiveBiasSample.y, (double)positiveBiasSample.z);


  // Validate the measurements, they should be different signs, ie,
  // product is negative.
  //
  if ((negativeBiasSample.x*positiveBiasSample.x < 0) &&
      (negativeBiasSample.y*positiveBiasSample.y < 0) &&
      (negativeBiasSample.z*positiveBiasSample.z < 0)) {
    
    // Compute offset
    //
    mscale_previous.x_offset = (negativeBiasSample.x + positiveBiasSample.x)/2;
    mscale_previous.y_offset = (negativeBiasSample.y + positiveBiasSample.y)/2;
    mscale_previous.z_offset = (negativeBiasSample.z + positiveBiasSample.z)/2;

  printf("computed calibration  (%.3f %.3f %.3f)\n", (double)mscale_previous.x_offset, (double)mscale_previous.y_offset, (double)mscale_previous.z_offset);
  }
  else {
    warnx("Calibration samples invalid!");
    ret = 1;
    goto out;
  }

  ret = OK;

 out:

  /* Install the new scale */
  if (OK != ioctl(filp, MAGIOCSSCALE, (long unsigned int)&mscale_previous)) {
    warn("FAILED: MAGIOCSSCALE 2");
  }

  // Restart measurement
  start();

  return ret;
}


int MMC5883::check_scale()
{
  bool scale_valid;

  // The scale is not set on this part, so it should be all 1's.
  //
	

  if ((-FLT_EPSILON + 1.0f < _scale.x_scale && _scale.x_scale < FLT_EPSILON + 1.0f) &&
      (-FLT_EPSILON + 1.0f < _scale.y_scale && _scale.y_scale < FLT_EPSILON + 1.0f) &&
      (-FLT_EPSILON + 1.0f < _scale.z_scale && _scale.z_scale < FLT_EPSILON + 1.0f)) {
    /* scale is one */
    scale_valid = true;
  } else {
    scale_valid = false;
  }

  /* return 0 if calibrated, 1 else */
  return !scale_valid;
}

int MMC5883::check_offset()
{
  bool offset_valid;

  if ((-2.0f * FLT_EPSILON < _scale.x_offset && _scale.x_offset < 2.0f * FLT_EPSILON) &&
      (-2.0f * FLT_EPSILON < _scale.y_offset && _scale.y_offset < 2.0f * FLT_EPSILON) &&
      (-2.0f * FLT_EPSILON < _scale.z_offset && _scale.z_offset < 2.0f * FLT_EPSILON)) {
    /* offsets are zero */
    offset_valid = false;
  } else {
    offset_valid = true;
  }

  /* return 0 if calibrated, 1 else */
  return !offset_valid;
}

int MMC5883::check_calibration()
{
  bool offset_valid = (check_offset() == OK);
  bool scale_valid  = (check_scale() == OK);

  if (_calibrated != (offset_valid && scale_valid)) {
    warnx("mag cal status changed %s%s", (scale_valid) ? "" : "scale invalid ",
	  (offset_valid) ? "" : "offset invalid");
    _calibrated = (offset_valid && scale_valid);
  }

  /* return 0 if calibrated, 1 else */
  return (!_calibrated);
}


int MMC5883::set_excitement(unsigned long enable)
{
  uint8_t control_reg_0 = 0;
  int ret = OK;


  if (((int)enable) < 0) {
    // Excite coil in negative direction.
    control_reg_0 |= REG_MASK_CONTROL_0_RESET;

  } else if (enable > 0) {
    // Excite coil in positive direction.
    control_reg_0 |= REG_MASK_CONTROL_0_SET;
  }

  if (control_reg_0 != 0) {
    ret = write_reg(REG_ADDR_CONTROL_0, control_reg_0);
  }

  if (OK != ret) {
    perf_count(_comms_errors);
  }

  return ret;
}



int
MMC5883::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t buf = val;
	return _interface->write(reg, &buf, 1);
}

int
MMC5883::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t buf = val;
	int ret = _interface->read(reg, &buf, 1);
	val = buf;
	return ret;
}

float
MMC5883::meas_to_float(uint8_t in[2])
{
	union {
		uint8_t	b[2];
		int16_t	w;
	} u;

	u.b[0] = in[1];
	u.b[1] = in[0];

	return (float) u.w;
}

void
MMC5883::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_field_read_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("output  (%.2f %.2f %.2f)\n", (double)_last_report.x, (double)_last_report.y, (double)_last_report.z);
	printf("offsets (%.3f %.3f %.3f)\n", (double)_scale.x_offset, (double)_scale.y_offset, (double)_scale.z_offset);
	printf("scaling (%.2f %.2f %.2f) 1/range_scale %.2f\n",
	       (double)_scale.x_scale, (double)_scale.y_scale, (double)_scale.z_scale,
	       (double)(1.0f/_range_scale));
	printf("temperature %.2f\n", (double)_last_report.temperature);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace mmc5883
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

/*
  list of supported bus configurations
 */
struct mmc5883_bus_option {
	enum MMC5883_BUS busid;
	const char *devpath;
	MMC5883_constructor interface_constructor;
	uint8_t busnum;
	MMC5883	*dev;
} bus_options[] = {
	{ MMC5883_BUS_I2C_EXTERNAL, "/dev/mmc5883_ext", &MMC5883_I2C_interface, PX4_I2C_BUS_EXPANSION, NULL },
#ifdef PX4_I2C_BUS_ONBOARD
	{ MMC5883_BUS_I2C_INTERNAL, "/dev/hmc5883_int", &MMC5883_I2C_interface, PX4_I2C_BUS_ONBOARD, NULL },
#endif
#ifdef PX4_SPIDEV_MMC
#endif
};
#define NUM_BUS_OPTIONS (sizeof(bus_options)/sizeof(bus_options[0]))

void	start(enum MMC5883_BUS busid, enum Rotation rotation);
bool	start_bus(struct mmc5883_bus_option &bus, enum Rotation rotation);
struct mmc5883_bus_option &find_bus(enum MMC5883_BUS busid);
void	test(enum MMC5883_BUS busid);
void	reset(enum MMC5883_BUS busid);
int	info(enum MMC5883_BUS busid);
int	calibrate(enum MMC5883_BUS busid);
int	temp_enable(MMC5883_BUS busid, bool enable);
void	usage();

/**
 * start driver for a specific bus option
 */
bool
start_bus(struct mmc5883_bus_option &bus, enum Rotation rotation)
{
	if (bus.dev != nullptr) {
		errx(1, "bus option already started");
	}

	device::Device *interface = bus.interface_constructor(bus.busnum);

	if (interface->init() != OK) {
		delete interface;
		warnx("no device on bus %u", (unsigned)bus.busid);
		return false;
	}

	bus.dev = new MMC5883(interface, bus.devpath, rotation);

	if (bus.dev != nullptr && OK != bus.dev->init()) {
		delete bus.dev;
		bus.dev = NULL;
		return false;
	}

	int fd = open(bus.devpath, O_RDONLY);

	if (fd < 0) {
		return false;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		close(fd);
		errx(1, "Failed to setup poll rate");
	}

	close(fd);

	return true;
}


/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
void
start(enum MMC5883_BUS busid, enum Rotation rotation)
{
	bool started = false;

	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if (busid == MMC5883_BUS_ALL && bus_options[i].dev != NULL) {
			// this device is already started
			continue;
		}

		if (busid != MMC5883_BUS_ALL && bus_options[i].busid != busid) {
			// not the one that is asked for
			continue;
		}
		started |= start_bus(bus_options[i], rotation);
	}

	if (!started) {
		errx(1, "driver start failed");
	}
}

/**
 * find a bus structure for a busid
 */
struct mmc5883_bus_option &find_bus(enum MMC5883_BUS busid)
{
	for (unsigned i = 0; i < NUM_BUS_OPTIONS; i++) {
		if ((busid == MMC5883_BUS_ALL ||
		     busid == bus_options[i].busid) && bus_options[i].dev != NULL) {
			return bus_options[i];
		}
	}

	errx(1, "bus %u not started", (unsigned)busid);
}


/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(enum MMC5883_BUS busid)
{
	struct mmc5883_bus_option &bus = find_bus(busid);
	struct mag_report report;
	ssize_t sz;
	int ret;
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'mmc5883 start')", path);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("measurement: %.6f  %.6f  %.6f", (double)report.x, (double)report.y, (double)report.z);
	warnx("time:        %lld", report.timestamp);

	/* check if mag is onboard or external */
	if ((ret = ioctl(fd, MAGIOCGEXTERNAL, 0)) < 0) {
		errx(1, "failed to get if mag is onboard or external");
	}

	warnx("device active: %s", ret ? "external" : "onboard");

	/* set the queue depth */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10)) {
		errx(1, "failed to set queue depth");
	}

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		warnx("periodic read %u", i);
		warnx("measurement: %.6f  %.6f  %.6f", (double)report.x, (double)report.y, (double)report.z);
		warnx("time:        %lld", report.timestamp);
	}

	errx(0, "PASS");
}


/**
 * Automatic scale calibration.
 *
 */
int calibrate(enum MMC5883_BUS busid)
{
	int ret;
	struct mmc5883_bus_option &bus = find_bus(busid);
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'mmc5883 start' if the driver is not running", path);
	}

	if (OK != (ret = ioctl(fd, MAGIOCCALIBRATE, fd))) {
		warnx("failed to enable sensor calibration mode");
	}

	close(fd);

	return ret;
}

/**
 * Reset the driver.
 */
void
reset(enum MMC5883_BUS busid)
{
	struct mmc5883_bus_option &bus = find_bus(busid);
	const char *path = bus.devpath;

	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}


/**
 * enable/disable temperature compensation
 */
int
temp_enable(enum MMC5883_BUS busid, bool enable)
{
  struct mmc5883_bus_option &bus = find_bus(busid);
  const char *path = bus.devpath;

  int fd = open(path, O_RDONLY);

  if (fd < 0) {
    err(1, "failed ");
  }

  if (ioctl(fd, MAGIOCSTEMPCOMP, (unsigned)enable) < 0) {
    err(1, "set temperature compensation failed");
  }

  close(fd);
  return 0;
}

/**
 * Print a little info about the driver.
 */
int
info(enum MMC5883_BUS busid)
{
	struct mmc5883_bus_option &bus = find_bus(busid);

	warnx("running on bus: %u (%s)\n", (unsigned)bus.busid, bus.devpath);
	bus.dev->print_info();
	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'reset', 'info', 'calibrate'");
	warnx("options:");
	warnx("    -R rotation");
	warnx("    -C calibrate on start");
	warnx("    -X only external bus");
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_MMC)
	warnx("    -I only internal bus");
#endif
}

} // namespace

int
mmc5883_main(int argc, char *argv[])
{
	int ch;
	enum MMC5883_BUS busid = MMC5883_BUS_ALL;
	enum Rotation rotation = ROTATION_NONE;
        bool calibrate = false;
	bool temp_compensation = false;

	while ((ch = getopt(argc, argv, "XIR:CT")) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;
#if (PX4_I2C_BUS_ONBOARD || PX4_SPIDEV_MMC)
		case 'I':
			busid = MMC5883_BUS_I2C_INTERNAL;
			break;
#endif
		case 'X':
			busid = MMC5883_BUS_I2C_EXTERNAL;
			break;
		case 'C':
			calibrate = true;
			break;
		case 'T':
			temp_compensation = true;
			break;
		default:
			mmc5883::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		mmc5883::start(busid, rotation);

		if (calibrate && mmc5883::calibrate(busid) != 0) {
			errx(1, "calibration failed");
		}

		if (temp_compensation) {
		  // we consider failing to setup temperature
		  // compensation as non-fatal
		  mmc5883::temp_enable(busid, true);
		}
		exit(0);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		mmc5883::test(busid);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		mmc5883::reset(busid);
	}

	/*
	 * enable/disable temperature compensation
	 */
	if (!strcmp(verb, "tempoff")) {
		mmc5883::temp_enable(busid, false);
	}

	if (!strcmp(verb, "tempon")) {
		mmc5883::temp_enable(busid, true);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
		mmc5883::info(busid);
	}

	/*
	 * Autocalibrate the scaling
	 */
	if (!strcmp(verb, "calibrate")) {
		if (mmc5883::calibrate(busid) == 0) {
			errx(0, "calibration successful");

		} else {
			errx(1, "calibration failed");
		}
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' 'calibrate', 'tempoff', 'tempon' or 'info'");
}
