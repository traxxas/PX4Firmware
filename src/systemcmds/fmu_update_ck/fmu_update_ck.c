/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file fmu_update_ck.c
 *
 * FMU update check tool.
 */
#include <px4_config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

#include <stm32.h>

#include "systemlib/systemlib.h"
#include "systemlib/err.h"

#define FMU_FILE_SIZE_LIMIT	0x000FBFFF    /* 1Mb - 0x4000 for bootloader */
#define FMU_APP_BASE            (STM32_FLASH_BASE + 0x4000)
#define FMU_SRAM_BASE           0x20000000
#define READ_BUFF_SIZE          1024
#define FIRMWARE_FILENAME_PREFIX    "TRX-FMU-V2"
#define FIRMWARE_FILENAME_SUFFIX    ".bin"
#define FIRMWARE_FILENAME_SIZE      64  /* max num of characters in filename */
#define BL_TOUCH_FILE_NAME          "UPGRADE"


__EXPORT int fmu_update_ck_main(int argc, char *argv[]);


int
fmu_update_ck_main(int argc, char *argv[])
{
  if (argc != 2)
    errx(1, "missing firmware directory");

  char bl_touch_filename[2*FIRMWARE_FILENAME_SIZE];

  strcpy(bl_touch_filename, argv[1]);
  strcat(bl_touch_filename, "/");
  strcat(bl_touch_filename, BL_TOUCH_FILE_NAME);
  strcat(bl_touch_filename, FIRMWARE_FILENAME_SUFFIX);

  struct stat s;

  if (stat(bl_touch_filename, &s) == 0) {
    /* Remove old touch file if it exists... */
    unlink(bl_touch_filename);
  }

  int upgrade_needed = 0;

  char firmware_candidate[FIRMWARE_FILENAME_SIZE];

  strcpy(firmware_candidate, FIRMWARE_FILENAME_PREFIX);
  strcat(firmware_candidate, FIRMWARE_FILENAME_SUFFIX);

  char full_path[2*FIRMWARE_FILENAME_SIZE];

  strcpy(full_path, argv[1]);
  strcat(full_path, "/");
  strcat(full_path, firmware_candidate);

  if (stat(full_path, &s) != 0 || !S_ISREG(s.st_mode)) {
    /* Not really an error, this is the expected startup path */
    errx(1, "firmware image %s not present", full_path);
  }

  /* sanity-check file size */
  if (s.st_size > FMU_FILE_SIZE_LIMIT) {
    errx(1, "%s: file too large (limit: %u, actual: %d)", full_path, FMU_FILE_SIZE_LIMIT, s.st_size);
  }

  int fd = open(full_path, O_RDONLY);

  if (fd < 0) {
    err(1, "open %s failed", full_path);
  }
	
  uint8_t *buf = malloc(READ_BUFF_SIZE);

  if (buf == NULL) {
    close (fd);
    errx(1, "failed to allocate %u bytes for firmware buffer", READ_BUFF_SIZE);
  }

  if (read(fd, buf, READ_BUFF_SIZE) != READ_BUFF_SIZE) {
    close (fd);
    free(buf);
    err(1, "firmware read error");
  }

  uint32_t *hdr = (uint32_t *)buf;

  if ((hdr[0] < FMU_SRAM_BASE) ||		    /* stack not below RAM */
      (hdr[0] > (FMU_SRAM_BASE + (192 * 1024))) ||  /* stack not above RAM */
      (hdr[1] < FMU_APP_BASE) ||	          /* entrypoint not below flash  or inside bootloader area */
      ((hdr[1] - STM32_FLASH_BASE) > FMU_FILE_SIZE_LIMIT)) {  /* entrypoint not outside flash */

    close (fd);
    free(buf);
    errx(1, "File %s not a bootloader image", full_path);
  }

  ssize_t bytes_read = READ_BUFF_SIZE;
  unsigned char* flash_addr = (unsigned char*) FMU_APP_BASE;

  do {
    if (memcmp(flash_addr, buf, bytes_read) != 0) {
      upgrade_needed = 1;
      break;
    }

    /* read next block */
    flash_addr += bytes_read;
    bytes_read = read(fd, buf, READ_BUFF_SIZE);

  } while (bytes_read > 0);

  
  /* warnx("Comparison done, flash offset: %x  Upgrade needed: %s\n",
     (unsigned int)flash_addr, (upgrade_needed?"Y":"N")); */


  if (upgrade_needed) {
    if (lseek(fd, 0, SEEK_SET) != 0) {
      errx(1, "lseek failed\n");
    }

    /* The application creates this file for the bootloader to
       use. Its presence tells the bootloader it needs to upgrade.
     */
    int touch_fd = open(bl_touch_filename, O_CREAT|O_WRONLY|O_TRUNC);

    if (touch_fd < 0) {
      errx(1, "Failed to open touch file %s\n", bl_touch_filename);
    }
      
    bytes_read = read(fd, buf, READ_BUFF_SIZE);

    while (bytes_read > 0) {
      write(touch_fd, buf, bytes_read);
      bytes_read = read(fd, buf, READ_BUFF_SIZE);
    }
    close(touch_fd);
  }

  close(fd);
  free(buf);
  
  /* non-zero indicates no upgrade needed, 0 indicates upgrade is needed */
  exit(!upgrade_needed);
}
