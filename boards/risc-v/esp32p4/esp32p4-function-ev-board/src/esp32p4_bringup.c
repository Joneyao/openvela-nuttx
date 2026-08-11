/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4_bringup.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/debug.h>
#include <fcntl.h>
#include <stdint.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <sched.h>

#include "espressif/esp_usbserial.h"

#include <nuttx/fs/fs.h>

#include "esp_board_ledc.h"
#include "esp_board_spiflash.h"
#include "esp_board_i2c.h"
#include "esp_board_bmp180.h"

#include "espressif/esp_start.h"

#ifdef CONFIG_WATCHDOG
#  include "espressif/esp_wdt.h"
#endif

#ifdef CONFIG_TIMER
#  include "espressif/esp_gptimer.h"
#endif

#ifdef CONFIG_ONESHOT
#  include "espressif/esp_oneshot.h"
#endif

#ifdef CONFIG_RTC_DRIVER
#  include "espressif/esp_rtc.h"
#endif

#ifdef CONFIG_DEV_GPIO
#  include "espressif/esp_gpio.h"
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_INPUT_GT9XX
#  include <nuttx/input/gt9xx.h>
#endif

#ifdef CONFIG_ESP32P4_JPEG_ENCODER
#  include "esp_jpeg_enc.h"
#endif

#ifdef CONFIG_ESPRESSIF_EFUSE
#  include "espressif/esp_efuse.h"
#endif

#ifdef CONFIG_ESP_RMT
#  include "esp_board_rmt.h"
#endif

#ifdef CONFIG_ESPRESSIF_I2S
#  include "esp_board_i2s.h"
#endif

#ifdef CONFIG_ESPRESSIF_SPI
#  include "espressif/esp_spi.h"
#  include "esp_board_spidev.h"
#  ifdef CONFIG_ESPRESSIF_SPI_BITBANG
#    include "espressif/esp_spi_bitbang.h"
#  endif
#endif

#ifdef CONFIG_SPI_SLAVE_DRIVER
#  include "espressif/esp_spi.h"
#  include "esp_board_spislavedev.h"
#endif

#ifdef CONFIG_ESPRESSIF_TEMP
#  include "espressif/esp_temperature_sensor.h"
#endif

#ifdef CONFIG_ESP_MCPWM
#  include "esp_board_mcpwm.h"
#endif

#ifdef CONFIG_ESP_PCNT
#  include "espressif/esp_pcnt.h"
#  include "esp_board_pcnt.h"
#endif

#ifdef CONFIG_ESPRESSIF_ADC
#  include "esp_board_adc.h"
#endif

#ifdef CONFIG_PM
#  include "espressif/esp_pm.h"
#endif

#ifdef CONFIG_SYSTEM_NXDIAG_ESPRESSIF_CHIP_WO_TOOL
#  include "espressif/esp_nxdiag.h"
#endif

#ifdef CONFIG_ESP_SDM
#  include "espressif/esp_sdm.h"
#endif

#ifdef CONFIG_COMP
#  include "espressif/esp_ana_cmpr.h"
#endif

#ifdef CONFIG_ESPRESSIF_USE_LP_CORE
#  include "espressif/esp_ulp.h"
#  ifdef CONFIG_ESPRESSIF_ULP_USE_TEST_BIN
#    include "ulp/ulp_code.h"
#  endif
#  ifdef CONFIG_ESPRESSIF_LP_MAILBOX
#    include "espressif/esp_lp_mailbox.h"
#  endif
#endif

#ifdef CONFIG_VIDEO_FB
#  include "esp_board_fb.h"
#endif
#ifdef CONFIG_ESP32P4_MIPI_DSI
#  include "esp_mipi_dsi.h"
#endif

#include "esp32p4-function-ev-board.h"

#ifdef CONFIG_ESP32P4_CAMERA
#  include "esp_camera.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <nuttx/net/ioctl.h>

#ifdef CONFIG_EXAMPLES_CAMPILOT
extern int campilot_main(int argc, char *argv[]);

static int configure_net(void)
{
  struct sockaddr_in addr;
  struct ifreq ifr;
  int fd;
  int ret;

  fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0)
    {
      printf("[campilot] socket failed: %d\n", errno);
      return -1;
    }

  memset(&ifr, 0, sizeof(ifr));
  strlcpy(ifr.ifr_name, "eth0", IFNAMSIZ);

  /* Set netmask first */
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = inet_addr("255.255.248.0");
  memcpy(&ifr.ifr_addr, &addr, sizeof(addr));
  ret = ioctl(fd, SIOCSIFNETMASK, (unsigned long)&ifr);
  printf("[campilot] SIOCSIFNETMASK: ret=%d errno=%d\n", ret, errno);

  /* Set IP */
  addr.sin_addr.s_addr = inet_addr("10.192.228.200");
  memcpy(&ifr.ifr_addr, &addr, sizeof(addr));
  ret = ioctl(fd, SIOCSIFADDR, (unsigned long)&ifr);
  printf("[campilot] SIOCSIFADDR: ret=%d errno=%d\n", ret, errno);

  /* Set gateway */
  addr.sin_addr.s_addr = inet_addr("10.192.224.1");
  memcpy(&ifr.ifr_addr, &addr, sizeof(addr));
  ret = ioctl(fd, SIOCSIFDSTADDR, (unsigned long)&ifr);
  printf("[campilot] SIOCSIFDSTADDR: ret=%d errno=%d\n", ret, errno);

  /* Read back IP */
  ret = ioctl(fd, SIOCGIFADDR, (unsigned long)&ifr);
  if (ret >= 0)
    {
      struct sockaddr_in *sin = (struct sockaddr_in *)&ifr.ifr_addr;
      printf("[campilot] SIOCGIFADDR verify: %s\n", inet_ntoa(sin->sin_addr));
    }
  else
    {
      printf("[campilot] SIOCGIFADDR failed: ret=%d errno=%d\n", ret, errno);
    }

  /* Read back netmask */
  ret = ioctl(fd, SIOCGIFNETMASK, (unsigned long)&ifr);
  if (ret >= 0)
    {
      struct sockaddr_in *sin = (struct sockaddr_in *)&ifr.ifr_addr;
      printf("[campilot] SIOCGIFNETMASK verify: %s\n", inet_ntoa(sin->sin_addr));
    }

  close(fd);
  printf("[campilot] network configured\n");
  return 0;
}

static int campilot_auto(int argc, char *argv[])
{
  int ret;

  printf("[campilot] auto-test starting...\n");

  /* Wait a bit for netinit thread to finish */
  usleep(500000);

  ret = configure_net();
  if (ret < 0)
    printf("[campilot] net config failed\n");

  printf("[campilot] === text hello ===\n");

  char *cargs[] = {"campilot", "text", "hello", NULL};
  campilot_main(3, cargs);

  printf("[campilot] auto-test done\n");
  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_bringup
 *
 * Description:
 *   Perform architecture-specific initialization.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp_bringup(void)
{
  int ret = OK;

  /* Boot delay before the RAM marker dump.  The USB-Serial-JTAG console
   * re-enumerates on a hard reset, so a fast boot prints the M: dump before
   * a monitor can attach.  Hold ~8s here so the post-hang marker dump is
   * reliably captured.  up_udelay() is calibrated via BOARD_LOOPSPERMSEC; if
   * the calibration is off by a few x the window is still several seconds. */

  up_udelay(8000000);  /* ~8 s */

  /* RAM debug marker dump.  Reads the app-only marker address (0x5010ffe0):
   * boot's own dbg_putc() markers go to 0x5010fff0 and would clobber the
   * value before this dump runs, so the two are kept separate.  The app
   * marker survives a warm reset and tells us the last point the previous
   * app run reached, independent of the lossy USB console.
   */

  {
    static const char hex[] = "0123456789abcdef";
    uint32_t m = *((volatile uint32_t *)0x5010ffe0);
    uint32_t d = *((volatile uint32_t *)0x5010fff0);
    uint32_t c = *((volatile uint32_t *)0x5010ffe4);
    uint32_t mc = *((volatile uint32_t *)0x5010ffe8);
    uint32_t iq = *((volatile uint32_t *)0x5010ffec);
    uint32_t snd = *((volatile uint32_t *)0x5010ffd0);  /* esp_send last char */
    uint32_t usw = *((volatile uint32_t *)0x5010ffd4);  /* esp_usbserial_write last char */
    uint32_t shr = *((volatile uint32_t *)0x5010ffd8);  /* shared_intr_isr run count */
    uint32_t src = *((volatile uint32_t *)0x5010ffcc);  /* last dispatched shared source */
    uint32_t intno = *((volatile uint32_t *)0x5010ffdc);  /* shared vector's CPU int */
    uint32_t flgs = *((volatile uint32_t *)0x5010ffbc);  /* shared vd->flags */
    uint32_t smask = *((volatile uint32_t *)0x5010ffc0); /* first sh_vec statusmask */
    uint32_t sreg = *((volatile uint32_t *)0x5010ffc4);  /* first sh_vec *statusreg value */
    uint32_t fsrc = *((volatile uint32_t *)0x5010ffc8);  /* first sh_vec source */
    uint32_t mf1 = *((volatile uint32_t *)0x5010ff90);  /* int_map 1st source routed to intno */
    uint32_t mf2 = *((volatile uint32_t *)0x5010ff94);  /* int_map 2nd source routed to intno */
    uint32_t mcnt = *((volatile uint32_t *)0x5010ff98); /* int_map count routed to intno */
    uint32_t rjpeg = *((volatile uint32_t *)0x5010ff80); /* JPEG int_map value */
    uint32_t rdma = *((volatile uint32_t *)0x5010ff84);  /* DMA2D_IN_CH0 int_map value */
    uint32_t r5 = *((volatile uint32_t *)0x5010ff88);    /* source-5 int_map value */
    uint32_t rc5 = *((volatile uint32_t *)0x5010ff8c);   /* CLIC int5 reg */
    uint32_t rc21 = *((volatile uint32_t *)0x5010ff70);  /* CLIC int21 reg (jpeg route) */
    uint32_t rc18 = *((volatile uint32_t *)0x5010ff74);  /* CLIC int18 reg (boot cpuint2) */
    uint32_t rc19 = *((volatile uint32_t *)0x5010ff78);  /* CLIC int19 reg (boot cpuint3) */
    uint32_t rc20 = *((volatile uint32_t *)0x5010ff7c);  /* CLIC int20 reg (boot cpuint4) */
    dbg_putc('M');
    dbg_putc(hex[(m >> 28) & 0xF]);
    dbg_putc(hex[(m >> 24) & 0xF]);
    dbg_putc(hex[(m >> 20) & 0xF]);
    dbg_putc(hex[(m >> 16) & 0xF]);
    dbg_putc(hex[(m >> 12) & 0xF]);
    dbg_putc(hex[(m >>  8) & 0xF]);
    dbg_putc(hex[(m >>  4) & 0xF]);
    dbg_putc(hex[m & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(d >> 28) & 0xF]);
    dbg_putc(hex[(d >> 24) & 0xF]);
    dbg_putc(hex[(d >> 20) & 0xF]);
    dbg_putc(hex[(d >> 16) & 0xF]);
    dbg_putc(hex[(d >> 12) & 0xF]);
    dbg_putc(hex[(d >>  8) & 0xF]);
    dbg_putc(hex[(d >>  4) & 0xF]);
    dbg_putc(hex[d & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(c >> 28) & 0xF]);
    dbg_putc(hex[(c >> 24) & 0xF]);
    dbg_putc(hex[(c >> 20) & 0xF]);
    dbg_putc(hex[(c >> 16) & 0xF]);
    dbg_putc(hex[(c >> 12) & 0xF]);
    dbg_putc(hex[(c >>  8) & 0xF]);
    dbg_putc(hex[(c >>  4) & 0xF]);
    dbg_putc(hex[c & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(mc >> 28) & 0xF]);
    dbg_putc(hex[(mc >> 24) & 0xF]);
    dbg_putc(hex[(mc >> 20) & 0xF]);
    dbg_putc(hex[(mc >> 16) & 0xF]);
    dbg_putc(hex[(mc >> 12) & 0xF]);
    dbg_putc(hex[(mc >>  8) & 0xF]);
    dbg_putc(hex[(mc >>  4) & 0xF]);
    dbg_putc(hex[mc & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(iq >> 28) & 0xF]);
    dbg_putc(hex[(iq >> 24) & 0xF]);
    dbg_putc(hex[(iq >> 20) & 0xF]);
    dbg_putc(hex[(iq >> 16) & 0xF]);
    dbg_putc(hex[(iq >> 12) & 0xF]);
    dbg_putc(hex[(iq >>  8) & 0xF]);
    dbg_putc(hex[(iq >>  4) & 0xF]);
    dbg_putc(hex[iq & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(snd >> 4) & 0xF]);
    dbg_putc(hex[snd & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(usw >> 4) & 0xF]);
    dbg_putc(hex[usw & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(shr >> 28) & 0xF]);
    dbg_putc(hex[(shr >> 24) & 0xF]);
    dbg_putc(hex[(shr >> 20) & 0xF]);
    dbg_putc(hex[(shr >> 16) & 0xF]);
    dbg_putc(hex[(shr >> 12) & 0xF]);
    dbg_putc(hex[(shr >>  8) & 0xF]);
    dbg_putc(hex[(shr >>  4) & 0xF]);
    dbg_putc(hex[shr & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(src >> 28) & 0xF]);
    dbg_putc(hex[(src >> 24) & 0xF]);
    dbg_putc(hex[(src >> 20) & 0xF]);
    dbg_putc(hex[(src >> 16) & 0xF]);
    dbg_putc(hex[(src >> 12) & 0xF]);
    dbg_putc(hex[(src >>  8) & 0xF]);
    dbg_putc(hex[(src >>  4) & 0xF]);
    dbg_putc(hex[src & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(intno >> 28) & 0xF]);
    dbg_putc(hex[(intno >> 24) & 0xF]);
    dbg_putc(hex[(intno >> 20) & 0xF]);
    dbg_putc(hex[(intno >> 16) & 0xF]);
    dbg_putc(hex[(intno >> 12) & 0xF]);
    dbg_putc(hex[(intno >>  8) & 0xF]);
    dbg_putc(hex[(intno >>  4) & 0xF]);
    dbg_putc(hex[intno & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(flgs >> 28) & 0xF]);
    dbg_putc(hex[(flgs >> 24) & 0xF]);
    dbg_putc(hex[(flgs >> 20) & 0xF]);
    dbg_putc(hex[(flgs >> 16) & 0xF]);
    dbg_putc(hex[(flgs >> 12) & 0xF]);
    dbg_putc(hex[(flgs >>  8) & 0xF]);
    dbg_putc(hex[(flgs >>  4) & 0xF]);
    dbg_putc(hex[flgs & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(smask >> 28) & 0xF]);
    dbg_putc(hex[(smask >> 24) & 0xF]);
    dbg_putc(hex[(smask >> 20) & 0xF]);
    dbg_putc(hex[(smask >> 16) & 0xF]);
    dbg_putc(hex[(smask >> 12) & 0xF]);
    dbg_putc(hex[(smask >>  8) & 0xF]);
    dbg_putc(hex[(smask >>  4) & 0xF]);
    dbg_putc(hex[smask & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(sreg >> 28) & 0xF]);
    dbg_putc(hex[(sreg >> 24) & 0xF]);
    dbg_putc(hex[(sreg >> 20) & 0xF]);
    dbg_putc(hex[(sreg >> 16) & 0xF]);
    dbg_putc(hex[(sreg >> 12) & 0xF]);
    dbg_putc(hex[(sreg >>  8) & 0xF]);
    dbg_putc(hex[(sreg >>  4) & 0xF]);
    dbg_putc(hex[sreg & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(fsrc >> 28) & 0xF]);
    dbg_putc(hex[(fsrc >> 24) & 0xF]);
    dbg_putc(hex[(fsrc >> 20) & 0xF]);
    dbg_putc(hex[(fsrc >> 16) & 0xF]);
    dbg_putc(hex[(fsrc >> 12) & 0xF]);
    dbg_putc(hex[(fsrc >>  8) & 0xF]);
    dbg_putc(hex[(fsrc >>  4) & 0xF]);
    dbg_putc(hex[fsrc & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(mf1 >> 28) & 0xF]);
    dbg_putc(hex[(mf1 >> 24) & 0xF]);
    dbg_putc(hex[(mf1 >> 20) & 0xF]);
    dbg_putc(hex[(mf1 >> 16) & 0xF]);
    dbg_putc(hex[(mf1 >> 12) & 0xF]);
    dbg_putc(hex[(mf1 >>  8) & 0xF]);
    dbg_putc(hex[(mf1 >>  4) & 0xF]);
    dbg_putc(hex[mf1 & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(mf2 >> 28) & 0xF]);
    dbg_putc(hex[(mf2 >> 24) & 0xF]);
    dbg_putc(hex[(mf2 >> 20) & 0xF]);
    dbg_putc(hex[(mf2 >> 16) & 0xF]);
    dbg_putc(hex[(mf2 >> 12) & 0xF]);
    dbg_putc(hex[(mf2 >>  8) & 0xF]);
    dbg_putc(hex[(mf2 >>  4) & 0xF]);
    dbg_putc(hex[mf2 & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(mcnt >> 28) & 0xF]);
    dbg_putc(hex[(mcnt >> 24) & 0xF]);
    dbg_putc(hex[(mcnt >> 20) & 0xF]);
    dbg_putc(hex[(mcnt >> 16) & 0xF]);
    dbg_putc(hex[(mcnt >> 12) & 0xF]);
    dbg_putc(hex[(mcnt >>  8) & 0xF]);
    dbg_putc(hex[(mcnt >>  4) & 0xF]);
    dbg_putc(hex[mcnt & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(rjpeg >>  4) & 0xF]);
    dbg_putc(hex[rjpeg & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(rdma >>  4) & 0xF]);
    dbg_putc(hex[rdma & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(r5 >>  4) & 0xF]);
    dbg_putc(hex[r5 & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(rc5 >>  4) & 0xF]);
    dbg_putc(hex[rc5 & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(rc21 >> 28) & 0xF]);
    dbg_putc(hex[(rc21 >> 24) & 0xF]);
    dbg_putc(hex[(rc21 >> 20) & 0xF]);
    dbg_putc(hex[(rc21 >> 16) & 0xF]);
    dbg_putc(hex[(rc21 >> 12) & 0xF]);
    dbg_putc(hex[(rc21 >>  8) & 0xF]);
    dbg_putc(hex[(rc21 >>  4) & 0xF]);
    dbg_putc(hex[rc21 & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(rc18 >> 28) & 0xF]);
    dbg_putc(hex[(rc18 >> 24) & 0xF]);
    dbg_putc(hex[(rc18 >> 20) & 0xF]);
    dbg_putc(hex[(rc18 >> 16) & 0xF]);
    dbg_putc(hex[(rc18 >> 12) & 0xF]);
    dbg_putc(hex[(rc18 >>  8) & 0xF]);
    dbg_putc(hex[(rc18 >>  4) & 0xF]);
    dbg_putc(hex[rc18 & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(rc19 >> 28) & 0xF]);
    dbg_putc(hex[(rc19 >> 24) & 0xF]);
    dbg_putc(hex[(rc19 >> 20) & 0xF]);
    dbg_putc(hex[(rc19 >> 16) & 0xF]);
    dbg_putc(hex[(rc19 >> 12) & 0xF]);
    dbg_putc(hex[(rc19 >>  8) & 0xF]);
    dbg_putc(hex[(rc19 >>  4) & 0xF]);
    dbg_putc(hex[rc19 & 0xF]);
    dbg_putc(':');
    dbg_putc(hex[(rc20 >> 28) & 0xF]);
    dbg_putc(hex[(rc20 >> 24) & 0xF]);
    dbg_putc(hex[(rc20 >> 20) & 0xF]);
    dbg_putc(hex[(rc20 >> 16) & 0xF]);
    dbg_putc(hex[(rc20 >> 12) & 0xF]);
    dbg_putc(hex[(rc20 >>  8) & 0xF]);
    dbg_putc(hex[(rc20 >>  4) & 0xF]);
    dbg_putc(hex[rc20 & 0xF]);
    /* JPEG bytesused recorded by jpegnc after a successful DQBUF
     * (0x5010ffc4).  Zero means the encode never produced output.
     * 0x5010ffc8 = open() result (fd, or -errno), 0x5010ffcc = write() ret,
     * 0x5010ffb0 = first 4 bytes of the JPEG bitstream (SOI magic FF D8). */
    {
      uint32_t jsz  = *((volatile uint32_t *)0x5010ffc4);
      uint32_t jfd  = *((volatile uint32_t *)0x5010ffc8);
      uint32_t jwr  = *((volatile uint32_t *)0x5010ffcc);
      uint32_t jmag = *((volatile uint32_t *)0x5010ffb0);
      const uint32_t w4[4] = { jsz, jfd, jwr, jmag };
      int j;
      for (j = 0; j < 4; j++)
        {
          uint32_t v = w4[j];
          dbg_putc(':');
          dbg_putc(hex[(v >> 28) & 0xF]); dbg_putc(hex[(v >> 24) & 0xF]);
          dbg_putc(hex[(v >> 20) & 0xF]); dbg_putc(hex[(v >> 16) & 0xF]);
          dbg_putc(hex[(v >> 12) & 0xF]); dbg_putc(hex[(v >>  8) & 0xF]);
          dbg_putc(hex[(v >>  4) & 0xF]); dbg_putc(hex[v & 0xF]);
        }
    }

    /* Console xmit state at the last dispatch (C: field).  Latch written by
     * riscv_dispatch_irq on EVERY dispatch (tick keeps firing during the
     * hang, so the last write = state when the console stopped draining).
     * 0x5010ff9c = jpegnc flush stage marker (jpegnc writes: 0x5052 before
     * final printf / 0x4646 printf returned / 0x4642 fflush returned),
     * 0x5010ffa0 = xmit.head<<16 | xmit.tail (head==tail => buffer empty,
     * head/tail ~size apart => full), 0x5010ffa4 = xmitsem.semcount,
     * 0x5010ffa8 = USJ int_ena (IN_EMPTY=bit3), 0x5010ffac = USJ int_st,
     * 0x5010ffb4 = USJ ep1_conf (serial_in_ep_data_free). */
    {
      uint32_t jbk   = *((volatile uint32_t *)0x5010ff9c);
      uint32_t cbuf  = *((volatile uint32_t *)0x5010ffa0);
      uint32_t csem  = *((volatile uint32_t *)0x5010ffa4);
      uint32_t cena  = *((volatile uint32_t *)0x5010ffa8);
      uint32_t cst   = *((volatile uint32_t *)0x5010ffac);
      uint32_t cfifo = *((volatile uint32_t *)0x5010ffb4);
      const uint32_t w6[6] = { jbk, cbuf, csem, cena, cst, cfifo };
      int j;
      dbg_putc('C');
      for (j = 0; j < 6; j++)
        {
          uint32_t v = w6[j];
          dbg_putc(':');
          dbg_putc(hex[(v >> 28) & 0xF]); dbg_putc(hex[(v >> 24) & 0xF]);
          dbg_putc(hex[(v >> 20) & 0xF]); dbg_putc(hex[(v >> 16) & 0xF]);
          dbg_putc(hex[(v >> 12) & 0xF]); dbg_putc(hex[(v >>  8) & 0xF]);
          dbg_putc(hex[(v >>  4) & 0xF]); dbg_putc(hex[v & 0xF]);
        }
    }

    /* Console xmit.lock stage markers (L: field), written by serial.c on
     * every uart_write()/uart_tcdrain() pass so the last value shows where
     * the thread that owns xmit.lock got stuck:
     * 0x5010ff44 = "WRLK" uart_write lock acquired
     * 0x5010ff48 = "WREN" uart_write txint re-enabled (end of fill loop)
     * 0x5010ff4c = "WRUN" uart_write about to unlock
     * 0x5010ff50 = "TDLK" uart_tcdrain lock acquired
     * 0x5010ff54 = "TDUN" uart_tcdrain about to unlock */
    {
      const uint32_t w7[7] =
        {
          *((volatile uint32_t *)0x5010ff44),
          *((volatile uint32_t *)0x5010ff48),
          *((volatile uint32_t *)0x5010ff4c),
          *((volatile uint32_t *)0x5010ff50),
          *((volatile uint32_t *)0x5010ff54),
          *((volatile uint32_t *)0x5010ff58),   /* esp_send TX drop counter */
          *((volatile uint32_t *)0x5010ff5c)    /* last dropped char + int_st */
        };
      int j;
      dbg_putc('L');
      for (j = 0; j < 7; j++)
        {
          uint32_t v = w7[j];
          dbg_putc(':');
          dbg_putc(hex[(v >> 28) & 0xF]); dbg_putc(hex[(v >> 24) & 0xF]);
          dbg_putc(hex[(v >> 20) & 0xF]); dbg_putc(hex[(v >> 16) & 0xF]);
          dbg_putc(hex[(v >> 12) & 0xF]); dbg_putc(hex[(v >>  8) & 0xF]);
          dbg_putc(hex[(v >>  4) & 0xF]); dbg_putc(hex[v & 0xF]);
        }
    }

    /* xmitsem wakeup probes (W: field), written by serial.c:
     * 0x5010ff60 = uart_datasent call count (ISR drained + attempted wakeup)
     * 0x5010ff64 = xmitsem.semcount at last uart_datasent (bit31 = the
     *              nxsem_get_value inside the probe failed, mirroring the
     *              uart_wakeup early-return path that would drop the post)
     * 0x5010ff68 = #times uart_putxmitchar blocked in nxsem_wait(xmitsem)
     * 0x5010ff6c = #times that nxsem_wait returned (writer woke up).
     * blocks>wakeups at hang => a writer is stuck in nxsem_wait forever. */
    {
      const uint32_t w4[4] =
        {
          *((volatile uint32_t *)0x5010ff60),
          *((volatile uint32_t *)0x5010ff64),
          *((volatile uint32_t *)0x5010ff68),
          *((volatile uint32_t *)0x5010ff6c)
        };
      int j;
      dbg_putc('W');
      for (j = 0; j < 4; j++)
        {
          uint32_t v = w4[j];
          dbg_putc(':');
          dbg_putc(hex[(v >> 28) & 0xF]); dbg_putc(hex[(v >> 24) & 0xF]);
          dbg_putc(hex[(v >> 20) & 0xF]); dbg_putc(hex[(v >> 16) & 0xF]);
          dbg_putc(hex[(v >> 12) & 0xF]); dbg_putc(hex[(v >>  8) & 0xF]);
          dbg_putc(hex[(v >>  4) & 0xF]); dbg_putc(hex[v & 0xF]);
        }
    }
    /* Block-time snapshot (S:) taken by the serial driver at the moment a
     * console writer sleeps on xmitsem (every block; last write wins, so if
     * the CPU wedged the last block is the stranded one).  Also the dispatch
     * counter pair that tells whether the tick kept firing at hang time:
     *   0x5010ff00 = dsp  (non-zeroed total dispatch counter)
     *   0x5010ff04 = pr   (dsp value app recorded at the PR stage)
     *   0x5010ff08 = z    (dsp value app recorded at MARK('z'))
     *   0x5010ff14 = ena  (USJ int_ena at the stranded block)
     *   0x5010ff18 = st   (USJ int_st at the stranded block)
     *   0x5010ff1c = ht   (xmit head<<16|tail at the stranded block)
     *   0x5010ff20 = sem  (xmitsem count at the stranded block)
     *   0x5010ff24 = seq  (blocks sequence number of the stranded block)
     *   0x5010ff28 = jbk  (jpegnc stage marker at the stranded block)
     *   0x5010ff2c = mst  (mstatus CSR at the stranded block -> MIE bit)
     *   0x5010ff30 = mca  (mcause CSR at the stranded block -> trap source)
     *   0x5010ff34 = mth  (mintthresh CSR at the stranded block)
     *   0x5010ff38 = ep1c (USJ EP1_CONF: TX FIFO free bytes)
     * If mst.MIE==0 -> nested-trap MPIE corruption is the wedge root cause.
     * If mst.MIE==1 && mth==0xFF -> CLIC threshold blocking all IRQs.
     * If mst.MIE==1 && mth==0 && ep1c full -> USJ hardware stuck. */
    {
      const uint32_t s14[14] =
        {
          *((volatile uint32_t *)0x5010ff00),
          *((volatile uint32_t *)0x5010ff04),
          *((volatile uint32_t *)0x5010ff08),
          *((volatile uint32_t *)0x5010ff14),
          *((volatile uint32_t *)0x5010ff18),
          *((volatile uint32_t *)0x5010ff1c),
          *((volatile uint32_t *)0x5010ff20),
          *((volatile uint32_t *)0x5010ff24),
          *((volatile uint32_t *)0x5010ff28),
          *((volatile uint32_t *)0x5010ff2c),
          *((volatile uint32_t *)0x5010ff30),
          *((volatile uint32_t *)0x5010ff34),
          *((volatile uint32_t *)0x5010ff38)
        };
      int j;
      dbg_putc('S');
      for (j = 0; j < 13; j++)
        {
          uint32_t v = s14[j];
          dbg_putc(':');
          dbg_putc(hex[(v >> 28) & 0xF]); dbg_putc(hex[(v >> 24) & 0xF]);
          dbg_putc(hex[(v >> 20) & 0xF]); dbg_putc(hex[(v >> 16) & 0xF]);
          dbg_putc(hex[(v >> 12) & 0xF]); dbg_putc(hex[(v >>  8) & 0xF]);
          dbg_putc(hex[(v >>  4) & 0xF]); dbg_putc(hex[v & 0xF]);
        }

      /* Non-zeroed copy: 0x5010fec0-0x5010fef4 survives bringup zeroing so
       * xd can read wedge-time S-field values even when K-flood drops the
       * serial output.  Well below the 0x5010ffxx diagnostic zone. */
      for (j = 0; j < 13; j++)
        {
          *((volatile uint32_t *)(0x5010fec0 + j * 4)) = s14[j];
        }
    }
    dbg_putc('\n');

    /* Direct int_map scan (always-on, independent of the shared_isr once-dump):
     * Q <map95> <map104> <srcs routed to cpuint0(CLIC16)> ...
     * DR_REG_INTERRUPT_CORE0_BASE = 0x500D6000; map[src] = base + 4*src.
     * intr_matrix_route stores (cpuint + 16), so cpuint0 appears as value 16.
     * Identify which peripheral keeps asserting CLIC int16 at boot. */
    {
      volatile uint32_t *mapbase = (volatile uint32_t *)0x500D6000;
      int i;
      dbg_putc('Q');
      dbg_putc(':');
      for (i = 0; i < 2; i++)
        {
          uint32_t v = mapbase[95] & 0x7f;
          dbg_putc(hex[(v >> 4) & 0xF]); dbg_putc(hex[v & 0xF]);
          dbg_putc(':');
          v = mapbase[104] & 0x7f;
          dbg_putc(hex[(v >> 4) & 0xF]); dbg_putc(hex[v & 0xF]);
        }
      for (i = 0; i < 130; i++)
        {
          uint32_t v = mapbase[i] & 0x7f;
          if (v == 16)
            {
              dbg_putc(':');
              dbg_putc(hex[(i >> 4) & 0xF]); dbg_putc(hex[i & 0xF]);
            }
        }
      dbg_putc('\n');
    }

    /* Storm-end state snapshot (per-call captured by shared_intr_isr):
     * N mcause clic21 clic5 cliccfg clicinfo mstatus jpraw jpena jpst matched */
    {
      uint32_t nmc   = *((volatile uint32_t *)0x5010ff00);  /* last mcause in ISR */
      uint32_t n21   = *((volatile uint32_t *)0x5010ff04);  /* CLIC int21 full */
      uint32_t n5    = *((volatile uint32_t *)0x5010ff08);  /* CLIC int5 full */
      uint32_t ncfg  = *((volatile uint32_t *)0x5010ff0c);  /* CLICCFG 0x20800000 */
      uint32_t ninf  = *((volatile uint32_t *)0x5010ff10);  /* CLICINFO 0x20800004 */
      uint32_t nst   = *((volatile uint32_t *)0x5010ff14);  /* mstatus in ISR */
      uint32_t nraw  = *((volatile uint32_t *)0x5010ff18);  /* jpeg int_raw */
      uint32_t nena  = *((volatile uint32_t *)0x5010ff1c);  /* jpeg int_ena */
      uint32_t nst2  = *((volatile uint32_t *)0x5010ff20);  /* jpeg int_st */
      uint32_t nmat  = *((volatile uint32_t *)0x5010ff24);  /* matched source or -1 */
      uint32_t ring[16];
      int i;
      for (i = 0; i < 16; i++) ring[i] = *((volatile uint32_t *)(0x5010fe00 + i * 4));
      uint32_t ringhead = *((volatile uint32_t *)0x5010fe40);
      const uint32_t w[10] = { nmc, n21, n5, ncfg, ninf, nst, nraw, nena, nst2, nmat };
      dbg_putc('N');
      for (i = 0; i < 10; i++) {
        dbg_putc(':');
        dbg_putc(hex[(w[i] >> 28) & 0xF]); dbg_putc(hex[(w[i] >> 24) & 0xF]);
        dbg_putc(hex[(w[i] >> 20) & 0xF]); dbg_putc(hex[(w[i] >> 16) & 0xF]);
        dbg_putc(hex[(w[i] >> 12) & 0xF]); dbg_putc(hex[(w[i] >>  8) & 0xF]);
        dbg_putc(hex[(w[i] >>  4) & 0xF]); dbg_putc(hex[w[i] & 0xF]);
      }
      dbg_putc(':');
      for (i = 0; i < 16; i++) {
        dbg_putc(hex[(ring[i] >> 28) & 0xF]); dbg_putc(hex[(ring[i] >> 24) & 0xF]);
        dbg_putc(hex[(ring[i] >> 20) & 0xF]); dbg_putc(hex[(ring[i] >> 16) & 0xF]);
        dbg_putc(hex[(ring[i] >> 12) & 0xF]); dbg_putc(hex[(ring[i] >>  8) & 0xF]);
        dbg_putc(hex[(ring[i] >>  4) & 0xF]); dbg_putc(hex[ring[i] & 0xF]);
        if (i != 15) dbg_putc('.');
      }
      dbg_putc(':');
      dbg_putc(hex[(ringhead >> 28) & 0xF]); dbg_putc(hex[(ringhead >> 24) & 0xF]);
      dbg_putc(hex[(ringhead >> 20) & 0xF]); dbg_putc(hex[(ringhead >> 16) & 0xF]);
      dbg_putc(hex[(ringhead >> 12) & 0xF]); dbg_putc(hex[(ringhead >>  8) & 0xF]);
      dbg_putc(hex[(ringhead >>  4) & 0xF]); dbg_putc(hex[ringhead & 0xF]);
      dbg_putc('\n');
    }

    /* Reset the storm counter/latch so the next hang latches fresh values.
     * Do this AFTER the dump above: the dump shows the previous run's data. */

    *((volatile uint32_t *)0x5010ffe4) = 0;
    *((volatile uint32_t *)0x5010ffe8) = 0;
    *((volatile uint32_t *)0x5010ffec) = 0;
    *((volatile uint32_t *)0x5010ffd0) = 0;
    *((volatile uint32_t *)0x5010ffd4) = 0;
    *((volatile uint32_t *)0x5010ffd8) = 0;
    *((volatile uint32_t *)0x5010ffcc) = 0;
    *((volatile uint32_t *)0x5010ffdc) = 0;
    *((volatile uint32_t *)0x5010ffbc) = 0;
    *((volatile uint32_t *)0x5010ffc0) = 0;

    /* From here on dbg_putc() only records in the RAM marker and never
     * touches the USB-Serial-JTAG TX FIFO.  The polled per-byte path would
     * otherwise race the interrupt-driven console drain (esp_sendbuf) on the
     * same 64-byte FIFO and corrupt app output -- most visibly during the
     * ESP-HAL JPEG engine's DPUT trace inside jpeg_new_encoder_engine(). */

    dbg_console_tx_set(false);
#define BRINGUP_MARK(c) do { } while(0)
    *((volatile uint32_t *)0x5010ffc4) = 0;
    *((volatile uint32_t *)0x5010ffc8) = 0;
    *((volatile uint32_t *)0x5010ff90) = 0;
    *((volatile uint32_t *)0x5010ff94) = 0;
    *((volatile uint32_t *)0x5010ff98) = 0;
    *((volatile uint32_t *)0x5010ff80) = 0;
    *((volatile uint32_t *)0x5010ff84) = 0;
    *((volatile uint32_t *)0x5010ff88) = 0;
    *((volatile uint32_t *)0x5010ff8c) = 0;
    *((volatile uint32_t *)0x5010ff70) = 0;
    *((volatile uint32_t *)0x5010ff74) = 0;
    *((volatile uint32_t *)0x5010ff78) = 0;
    *((volatile uint32_t *)0x5010ff7c) = 0;
    *((volatile uint32_t *)0x5010ff00) = 0;   /* storm-end snapshot block */
    *((volatile uint32_t *)0x5010ff04) = 0;
    *((volatile uint32_t *)0x5010ff08) = 0;
    *((volatile uint32_t *)0x5010ff0c) = 0;
    *((volatile uint32_t *)0x5010ff10) = 0;
    *((volatile uint32_t *)0x5010ff14) = 0;
    *((volatile uint32_t *)0x5010ff18) = 0;
    *((volatile uint32_t *)0x5010ff1c) = 0;
    *((volatile uint32_t *)0x5010ff20) = 0;
    *((volatile uint32_t *)0x5010ff24) = 0;
    *((volatile uint32_t *)0x5010ff28) = 0;   /* jbk */
    *((volatile uint32_t *)0x5010ff2c) = 0;   /* mstatus (CSR 0x300) */
    *((volatile uint32_t *)0x5010ff30) = 0;   /* mcause  (CSR 0x342) */
    *((volatile uint32_t *)0x5010ff34) = 0;   /* mintthresh (CSR 0x347) */
    *((volatile uint32_t *)0x5010ff38) = 0;   /* USJ EP1_CONF */
    *((volatile uint32_t *)0x5010ff60) = 0;   /* W: datasent counter */
    *((volatile uint32_t *)0x5010ff64) = 0;   /* W: xmitsem sval */
    *((volatile uint32_t *)0x5010ff68) = 0;   /* W: blocks counter */
    *((volatile uint32_t *)0x5010ff6c) = 0;   /* W: wakes counter */
    {
      int i;
      for (i = 0; i < 16; i++) *((volatile uint32_t *)(0x5010fe00 + i * 4)) = 0;
      *((volatile uint32_t *)0x5010fe40) = 0;  /* mcause ring head */
    }
  }

  /* Neutralize the ROM bootloader's leftover UART0 interrupt route.
   *
   * The ROM routes UART0 (intr source 31) to CPU int 5 (CLIC 21) for its own
   * console.  This board's console lives on USB-Serial-JTAG, so nobody ever
   * services UART0: its interrupt line stays asserted (floating RX / stale
   * FIFO), and the first driver that shares cpuint 5 through esp_intr_alloc
   * (the JPEG encoder) and enables CLIC 21 turns that asserted line into an
   * unstoppable level storm - shared_intr_isr only knows the JPEG vector, the
   * JPEG status register reads 0 (no encode), nothing is cleared, and the
   * dispatch re-fires forever ('k' flood).  This was the root cause of the
   * jpegenc hang.
   *
   * Disable UART0's peripheral interrupt and re-route its source to the
   * reserved INT_MUX_DISABLED_INTNO (CPU int 6, never enabled) so UART0 can
   * never assert into any CLIC line again. */
  {
    volatile uint32_t *uart = (volatile uint32_t *)0x500CA000; /* DR_REG_UART0_BASE */
    uart[0x10 / 4] = 0xffffffffu;  /* UART_INT_CLR: clear every pending bit */
    uart[0x0c / 4] = 0;            /* UART_INT_ENA: mask the peripheral */
    volatile uint32_t *map = (volatile uint32_t *)0x500D6000;  /* INTR_CORE0 */
    map[31] = (map[31] & ~0x3Fu) | 6u;  /* source 31 -> INT_MUX_DISABLED_INTNO */
  }
  BRINGUP_MARK('1');  /* checkpoint: UART0 neutralized */

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  BRINGUP_MARK('2');  /* checkpoint: before procfs mount */
  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      _err("Failed to mount procfs at /proc: %d\n", ret);
    }
  BRINGUP_MARK('A');  /* checkpoint: procfs mounted */
#endif

#ifdef CONFIG_FS_TMPFS
  /* Mount the tmpfs file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      _err("Failed to mount tmpfs at %s: %d\n", CONFIG_LIBC_TMPDIR, ret);
    }
#endif

#if defined(CONFIG_ESPRESSIF_EFUSE)
  ret = esp_efuse_initialize("/dev/efuse");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init EFUSE: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_MWDT0
  ret = esp_wdt_initialize("/dev/watchdog0", ESP_WDT_MWDT0);
  if (ret < 0)
    {
      _err("Failed to initialize WDT: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_MWDT1
  ret = esp_wdt_initialize("/dev/watchdog1", ESP_WDT_MWDT1);
  if (ret < 0)
    {
      _err("Failed to initialize WDT: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_RWDT
  ret = esp_wdt_initialize("/dev/watchdog2", ESP_WDT_RWDT);
  if (ret < 0)
    {
      _err("Failed to initialize WDT: %d\n", ret);
    }
#endif

#ifdef CONFIG_TIMER
  ret = esp_timer_initialize(0);
  if (ret < 0)
    {
      _err("Failed to initialize Timer 0: %d\n", ret);
    }

#ifndef CONFIG_ONESHOT
  ret = esp_timer_initialize(1);
  if (ret < 0)
    {
      _err("Failed to initialize Timer 1: %d\n", ret);
    }
#endif
#endif

#ifdef CONFIG_ONESHOT
  ret = esp_oneshot_initialize();
  if (ret < 0)
    {
      _err("Failed to initialize Oneshot Timer: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP_RMT
  ret = board_rmt_txinitialize(RMT_OUTPUT_PIN);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_rmt_txinitialize() failed: %d\n", ret);
    }

  ret = board_rmt_rxinitialize(RMT_INPUT_PIN);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_rmt_txinitialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_RTC_DRIVER
  /* Initialize the RTC driver */

  ret = esp_rtc_driverinit();
  if (ret < 0)
    {
      _err("Failed to initialize the RTC driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_SPI
#  if defined(CONFIG_ESPRESSIF_SPI2_SLAVE) && defined(CONFIG_ESPRESSIF_SPI2)
  ret = board_spislavedev_initialize(ESPRESSIF_SPI2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SPI%d Slave driver: %d\n",
             ESPRESSIF_SPI2, ret);
    }
#  elif defined(CONFIG_ESPRESSIF_SPI2)
  ret = board_spidev_initialize(ESPRESSIF_SPI2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init spidev 2: %d\n", ret);
    }
#  endif

#  if defined(CONFIG_ESPRESSIF_SPI3_SLAVE) && defined(CONFIG_ESPRESSIF_SPI3)
  ret = board_spislavedev_initialize(ESPRESSIF_SPI3);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SPI%d Slave driver: %d\n",
             ESPRESSIF_SPI3, ret);
    }
#  elif defined(CONFIG_ESPRESSIF_SPI3)
  ret = board_spidev_initialize(ESPRESSIF_SPI3);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init spidev 3: %d\n", ret);
    }
#  endif

#  ifdef CONFIG_ESPRESSIF_SPI_BITBANG
  ret = board_spidev_initialize(ESPRESSIF_SPI_BITBANG);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init spidev 3: %d\n", ret);
    }
#  endif /* CONFIG_ESPRESSIF_SPI_BITBANG */

#  ifdef CONFIG_ESPRESSIF_LPSPI0
  ret = board_spidev_initialize(ESPRESSIF_LPSPI0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init lpspi: %d\n", ret);
    }
#  endif
#endif /* CONFIG_ESPRESSIF_SPI */

#ifdef CONFIG_ESPRESSIF_SPIFLASH
  ret = board_spiflash_init();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI Flash\n");
    }
#endif

#if defined(CONFIG_ESPRESSIF_I2S)
  /* Configure I2S peripheral interfaces */

  ret = board_i2s_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2S driver: %d\n", ret);
    }
#endif

#if defined(CONFIG_I2C_DRIVER)
  /* Configure I2C peripheral interfaces */

  ret = board_i2c_init();

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_BMP180
  /* Try to register BMP180 device in I2C0 */

  ret = board_bmp180_initialize(0);

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize BMP180 "
             "Driver for I2C0: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP_SDM
  struct esp_sdm_chan_config_s config =
  {
    .gpio_num = 5,
    .sample_rate_hz = 1000 * 1000,
    .flags = 0,
  };

  struct dac_dev_s *dev = esp_sdminitialize(config);
  ret = dac_register("/dev/dac0", dev);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize DAC driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_TEMP
  struct esp_temp_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG(10, 50);
  ret = esp_temperature_sensor_initialize(cfg);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize temperature sensor driver: %d\n",
             ret);
    }
#endif
#ifdef CONFIG_ESPRESSIF_TWAI0

  /* Initialize TWAI and register the TWAI driver. */

  ret = board_twai_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: TWAI0 board_twai_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_TWAI1

  /* Initialize TWAI and register the TWAI driver. */

  ret = board_twai_setup(1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: TWAI1 board_twai_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_TWAI2

  /* Initialize TWAI and register the TWAI driver. */

  ret = board_twai_setup(2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: TWAI2 board_twai_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_DEV_GPIO
  ret = esp_gpio_init();
  if (ret < 0)
    {
      ierr("Failed to initialize GPIO Driver: %d\n", ret);
    }
#endif

#if defined(CONFIG_INPUT_BUTTONS) && defined(CONFIG_INPUT_BUTTONS_LOWER)
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      ierr("ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_LEDC
  ret = board_ledc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_ledc_setup() failed: %d\n", ret);
    }
#endif /* CONFIG_ESPRESSIF_LEDC */

#ifdef CONFIG_ESP_MCPWM_CAPTURE
  ret = board_capture_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_capture_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP_MCPWM_MOTOR
  ret = board_motor_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_motor_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP_PCNT
  ret = board_pcnt_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_pcnt_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_PM
  /* Configure PM */

  ret = esp_pmconfigure();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_pmconfigure failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SYSTEM_NXDIAG_ESPRESSIF_CHIP_WO_TOOL
  ret = esp_nxdiag_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_nxdiag_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_ADC
  ret = board_adc_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_adc_init failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_ANA_COMPR0
  ret = esp_cmprinitialize(ESPRESSIF_COMP0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_cmprinitialize(%d) failed: %d\n",
             ESPRESSIF_COMP0, ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_ANA_COMPR1
  ret = esp_cmprinitialize(ESPRESSIF_COMP1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_cmprinitialize(%d) failed: %d\n",
             ESPRESSIF_COMP1, ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_EMAC
  BRINGUP_MARK('3');  /* checkpoint: before EMAC init */
  ret = board_emac_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_emac_init failed: %d\n", ret);
    }
  BRINGUP_MARK('B');  /* checkpoint: EMAC init done */
#endif

#ifdef CONFIG_ESPRESSIF_USE_LP_CORE
#  ifdef CONFIG_ESPRESSIF_LP_MAILBOX
  esp_lp_mailbox_init();
#  endif

  /* ULP initialization should be the handled later than
   * peripherals to use supported peripherals properly on ULP core
   */

  ret = esp_ulp_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_ulp_init failed: %d\n", ret);
    }
  else
    {
#  ifdef CONFIG_ESPRESSIF_ULP_USE_TEST_BIN
      esp_ulp_load_bin((char *)esp_ulp_bin, esp_ulp_bin_len);
#  endif
    }
#endif

#ifdef CONFIG_ESP32P4_MIPI_DSI
  ret = esp_mipi_dsi_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_mipi_dsi_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_VIDEO_FB
  ret = board_fb_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_fb_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32P4_CAMERA
  ret = esp_camera_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_camera_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32P4_MIPI_DSI
  /* Start the DSI DMA refresh after camera init. The camera no longer
   * resets the DW-GDMA controller (both go through the shared high-level
   * driver), so the order here only determines channel assignment: the
   * camera claims channel 0 and the display gets channel 1.
   */

  esp_mipi_dsi_start_refresh();

  /* The demo thread repaints the whole framebuffer every 2 seconds, which
   * would stomp on camera frames. Only run it as a standalone display
   * self-test, i.e. when there is no camera preview to interfere with.
   */

#ifndef CONFIG_ESP32P4_CAMERA
  esp_mipi_dsi_start_demo();
#endif
#endif



  /* Initialize Goodix GT911 touch controller (polling mode, no IRQ pin) */

#ifdef CONFIG_INPUT_GT9XX
  ret = board_gt911_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_gt911_initialize failed: %d\n", ret);
    }
#endif /* CONFIG_INPUT_GT9XX */

  /* Register JPEG encoder device (V4L2 M2M bridge to ESP-HAL) */

#ifdef CONFIG_ESP32P4_JPEG_ENCODER
  ret = esp_jpeg_encoder_register("/dev/video1");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_jpeg_encoder_register failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_EXAMPLES_CAMPILOT
  /* Auto-test campilot at boot: wait for network late-init then
   * send a text query to MiMo API. Output appears on serial console
   * so we can verify without interactive input. */
  BRINGUP_MARK('4');  /* checkpoint: before campilot task */
  task_create("cptest", 100, 16384, campilot_auto, NULL);
  BRINGUP_MARK('C');  /* checkpoint: campilot task created */
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  return ret;
}
