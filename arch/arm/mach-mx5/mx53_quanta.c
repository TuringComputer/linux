/*
 * Copyright (C) 2015 Turing Computer. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/powerkey.h>
#include <linux/ahci_platform.h>
#include <linux/gpio_keys.h>
#include <linux/mfd/da9052/da9052.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/keypad.h>
#include <asm/mach/flash.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/mxc_dvfs.h>
#include <mach/iomux-mx53.h>
#include <mach/i2c.h>
#include <mach/mxc_iim.h>
#include <mach/check_fuse.h>

#include "crm_regs.h"
#include "devices.h"
#include "usb.h"
#include "pmic.h"
#include "mx53_quanta.h"

/*!
 * @file mach-mx5/mx53_quanta.c
 *
 * @brief This file contains MX53 quanta board specific initialization routines.
 *
 * @ingroup MSL_MX53
 */

#define GPIO0						QUANTA_SOM_GPIO00		// CSI Reset
#define GPIO1						QUANTA_SOM_GPIO01		// USB Hub Reset
#define GPIO2						QUANTA_SOM_GPIO02
#define GPIO3						QUANTA_SOM_GPIO03
#define GPIO4						QUANTA_SOM_GPIO04
#define GPIO5						QUANTA_SOM_GPIO05
#define GPIO6						QUANTA_SOM_GPIO06
#define GPIO7						QUANTA_SOM_GPIO07
#define GPIO8						QUANTA_SOM_GPIO08
#define GPIO9						QUANTA_SOM_GPIO09
#define GPIO10						QUANTA_SOM_GPIO10
#define GPIO11						QUANTA_SOM_GPIO11
#define GPIO12						QUANTA_SOM_GPIO12
#define GPIO13						QUANTA_SOM_GPIO13
#define GPIO14						QUANTA_SOM_GPIO14		// LED 1
#define GPIO15						QUANTA_SOM_GPIO15		// LED 2
#define GPIO16						QUANTA_SOM_GPIO16		// LED 3
#define GPIO17						QUANTA_SOM_GPIO17		// LED 4
#define GPIO18						QUANTA_SOM_GPIO18		// LED 5
#define GPIO19						QUANTA_SOM_GPIO19		// BUTTON 1
#define GPIO20						QUANTA_SOM_GPIO20		// BUTTON 2
#define GPIO21						QUANTA_SOM_GPIO21		// BUTTON 3
#define GPIO22						QUANTA_SOM_GPIO22		// HDMI_CEC_D
#define GPIO23						QUANTA_SOM_GPIO23		// HDMI INT
#define GPIO24						QUANTA_SOM_GPIO24		// HDMI Reset
#define GPIO25						QUANTA_SOM_GPIO25		// Headphone Detect
#define OSC_CKIH1_EN				QUANTA_SOM_GPIO26
#define LCD_EN						QUANTA_SOM_GPIO27
#define OTG_PWR_EN					QUANTA_SOM_OTG_PWR_EN
#define OTG_OC						QUANTA_SOM_OTG_OC
#define SD_CD						QUANTA_SOM_SD1_CD
#define SD_WP						QUANTA_SOM_SD1_WP

#define MX53_OFFSET					(0x20000000)
#define TZIC_WAKEUP0_OFFSET         (0x0E00)
#define TZIC_WAKEUP1_OFFSET         (0x0E04)
#define TZIC_WAKEUP2_OFFSET         (0x0E08)
#define TZIC_WAKEUP3_OFFSET         (0x0E0C)
#define GPIO7_0_11_IRQ_BIT			(0x1<<11)

extern void pm_i2c_init(u32 base_addr);

static iomux_v3_cfg_t mx53_quanta_pads[] = {
		/*AUDMUX*/
		MX53_PAD_CSI0_DAT7__AUDMUX_AUD3_RXD,
		MX53_PAD_CSI0_DAT4__AUDMUX_AUD3_TXC,
		MX53_PAD_CSI0_DAT5__AUDMUX_AUD3_TXD,
		MX53_PAD_CSI0_DAT6__AUDMUX_AUD3_TXFS,
		MX53_PAD_DISP0_DAT19__AUDMUX_AUD5_RXD,
		MX53_PAD_DISP0_DAT16__AUDMUX_AUD5_TXC,
		MX53_PAD_DISP0_DAT17__AUDMUX_AUD5_TXD,
		MX53_PAD_DISP0_DAT18__AUDMUX_AUD5_TXFS,
		/*CAN2*/
		MX53_PAD_KEY_ROW4__CAN2_RXCAN,
		MX53_PAD_KEY_COL4__CAN2_TXCAN,
		/*ECSPI1*/
		MX53_PAD_EIM_D17__ECSPI1_MISO,
		MX53_PAD_EIM_D18__ECSPI1_MOSI,
		MX53_PAD_KEY_COL0__ECSPI1_SCLK,
		/*EMI*/
		/*All Others pins are dedicated*/
		MX53_PAD_EIM_LBA__EMI_WEIM_LBA,
		/*ESDHC1*/
		MX53_PAD_SD1_CLK__ESDHC1_CLK,
		MX53_PAD_SD1_CMD__ESDHC1_CMD,
		MX53_PAD_SD1_DATA0__ESDHC1_DAT0,
		MX53_PAD_SD1_DATA1__ESDHC1_DAT1,
		MX53_PAD_SD1_DATA2__ESDHC1_DAT2,
		MX53_PAD_SD1_DATA3__ESDHC1_DAT3,
		/*ESDHC2*/
		MX53_PAD_SD2_CLK__ESDHC2_CLK,
		MX53_PAD_SD2_CMD__ESDHC2_CMD,
		MX53_PAD_SD2_DATA0__ESDHC2_DAT0,
		MX53_PAD_SD2_DATA1__ESDHC2_DAT1,
		MX53_PAD_SD2_DATA2__ESDHC2_DAT2,
		MX53_PAD_SD2_DATA3__ESDHC2_DAT3,
		/*ESDHC3*/
		MX53_PAD_PATA_IORDY__ESDHC3_CLK,
		MX53_PAD_PATA_RESET_B__ESDHC3_CMD,
		MX53_PAD_PATA_DATA8__ESDHC3_DAT0,
		MX53_PAD_PATA_DATA9__ESDHC3_DAT1,
		MX53_PAD_PATA_DATA10__ESDHC3_DAT2,
		MX53_PAD_PATA_DATA11__ESDHC3_DAT3,
		MX53_PAD_PATA_DATA0__ESDHC3_DAT4,
		MX53_PAD_PATA_DATA1__ESDHC3_DAT5,
		MX53_PAD_PATA_DATA2__ESDHC3_DAT6,
		MX53_PAD_PATA_DATA3__ESDHC3_DAT7,
		MX53_PAD_PATA_DA_0__ESDHC3_RST,
		/*FEC*/
		MX53_PAD_FEC_MDC__FEC_MDC,
		MX53_PAD_FEC_MDIO__FEC_MDIO,
		MX53_PAD_FEC_RXD0__FEC_RDATA_0,
		MX53_PAD_FEC_RXD1__FEC_RDATA_1,
		MX53_PAD_FEC_CRS_DV__FEC_RX_DV,
		MX53_PAD_FEC_RX_ER__FEC_RX_ER,
		MX53_PAD_FEC_TXD0__FEC_TDATA_0,
		MX53_PAD_FEC_TXD1__FEC_TDATA_1,
		MX53_PAD_FEC_REF_CLK__FEC_TX_CLK,
		MX53_PAD_FEC_TX_EN__FEC_TX_EN,
		/*GPIO1*/
		//MX53_PAD_GPIO_0__GPIO1_0,
		MX53_PAD_GPIO_4__GPIO1_4,
		MX53_PAD_GPIO_5__GPIO1_5,
		MX53_PAD_GPIO_7__GPIO1_7,
		MX53_PAD_GPIO_9__GPIO1_9,
		/*GPIO2*/
		MX53_PAD_PATA_DATA13__GPIO2_13,
		MX53_PAD_PATA_DATA14__GPIO2_14,
		MX53_PAD_EIM_OE__GPIO2_25,
		MX53_PAD_PATA_DATA5__GPIO2_5,
		MX53_PAD_PATA_DATA7__GPIO2_7,
		/*GPIO3*/
		MX53_PAD_EIM_DA13__GPIO3_13,
		MX53_PAD_EIM_D19__GPIO3_19,
		MX53_PAD_EIM_D20__GPIO3_20,
		MX53_PAD_EIM_D22__GPIO3_22,
		MX53_PAD_EIM_D23__GPIO3_23,
		MX53_PAD_EIM_D28__GPIO3_28,
		/*GPIO4*/
		MX53_PAD_DI0_DISP_CLK__GPIO4_16,
		MX53_PAD_DI0_PIN15__GPIO4_17,
		MX53_PAD_DI0_PIN2__GPIO4_18,
		MX53_PAD_DI0_PIN3__GPIO4_19,
		MX53_PAD_DI0_PIN4__GPIO4_20,
		MX53_PAD_DISP0_DAT0__GPIO4_21,
		MX53_PAD_DISP0_DAT1__GPIO4_22,
		MX53_PAD_DISP0_DAT2__GPIO4_23,
		MX53_PAD_DISP0_DAT3__GPIO4_24,
		MX53_PAD_DISP0_DAT5__GPIO4_26,
		MX53_PAD_DISP0_DAT6__GPIO4_27,
		MX53_PAD_DISP0_DAT9__GPIO4_30,
		MX53_PAD_DISP0_DAT10__GPIO4_31,
		MX53_PAD_GPIO_19__GPIO4_5,
		MX53_PAD_KEY_COL2__GPIO4_10,
		/*GPIO5*/
		MX53_PAD_DISP0_DAT21__GPIO5_15,
		MX53_PAD_DISP0_DAT22__GPIO5_16,
		MX53_PAD_DISP0_DAT23__GPIO5_17,
		MX53_PAD_EIM_A25__GPIO5_2,
		MX53_PAD_CSI0_DAT10__GPIO5_28,
		MX53_PAD_CSI0_DAT11__GPIO5_29,
		MX53_PAD_DISP0_DAT11__GPIO5_5,
		MX53_PAD_DISP0_DAT12__GPIO5_6,
		MX53_PAD_DISP0_DAT13__GPIO5_7,
		MX53_PAD_DISP0_DAT14__GPIO5_8,
		MX53_PAD_DISP0_DAT15__GPIO5_9,
		/*GPIO6*/
		//No muxing
		/*GPIO7*/
		MX53_PAD_GPIO_16__GPIO7_11,
		MX53_PAD_GPIO_18__GPIO7_13,
		MX53_PAD_PATA_DIOR__GPIO7_3,
		/*I2C1*/
		MX53_PAD_CSI0_DAT9__I2C1_SCL,
		MX53_PAD_CSI0_DAT8__I2C1_SDA,
		/*I2C2*/
		MX53_PAD_EIM_EB2__I2C2_SCL,
		MX53_PAD_EIM_D16__I2C2_SDA,
		/*I2C3*/
		MX53_PAD_GPIO_3__I2C3_SCL,
		MX53_PAD_GPIO_6__I2C3_SDA,
		/*IPU*/
		MX53_PAD_CSI0_DAT12__IPU_CSI0_D_12,
		MX53_PAD_CSI0_DAT13__IPU_CSI0_D_13,
		MX53_PAD_CSI0_DAT14__IPU_CSI0_D_14,
		MX53_PAD_CSI0_DAT15__IPU_CSI0_D_15,
		MX53_PAD_CSI0_DAT16__IPU_CSI0_D_16,
		MX53_PAD_CSI0_DAT17__IPU_CSI0_D_17,
		MX53_PAD_CSI0_DAT18__IPU_CSI0_D_18,
		MX53_PAD_CSI0_DAT19__IPU_CSI0_D_19,
		MX53_PAD_CSI0_DATA_EN__IPU_CSI0_DATA_EN,
		MX53_PAD_CSI0_MCLK__IPU_CSI0_HSYNC,
		MX53_PAD_CSI0_PIXCLK__IPU_CSI0_PIXCLK,
		MX53_PAD_CSI0_VSYNC__IPU_CSI0_VSYNC,
		MX53_PAD_EIM_A16__IPU_DI1_DISP_CLK,
		MX53_PAD_EIM_D29__IPU_DI1_PIN15,
		MX53_PAD_EIM_DA11__IPU_DI1_PIN2,
		MX53_PAD_EIM_DA12__IPU_DI1_PIN3,
		MX53_PAD_EIM_DA9__IPU_DISP1_DAT_0,
		MX53_PAD_EIM_DA8__IPU_DISP1_DAT_1,
		MX53_PAD_EIM_EB1__IPU_DISP1_DAT_10,
		MX53_PAD_EIM_EB0__IPU_DISP1_DAT_11,
		MX53_PAD_EIM_A17__IPU_DISP1_DAT_12,
		MX53_PAD_EIM_A18__IPU_DISP1_DAT_13,
		MX53_PAD_EIM_A19__IPU_DISP1_DAT_14,
		MX53_PAD_EIM_A20__IPU_DISP1_DAT_15,
		MX53_PAD_EIM_A21__IPU_DISP1_DAT_16,
		MX53_PAD_EIM_A22__IPU_DISP1_DAT_17,
		MX53_PAD_EIM_A23__IPU_DISP1_DAT_18,
		MX53_PAD_EIM_A24__IPU_DISP1_DAT_19,
		MX53_PAD_EIM_DA7__IPU_DISP1_DAT_2,
		MX53_PAD_EIM_D31__IPU_DISP1_DAT_20,
		MX53_PAD_EIM_D30__IPU_DISP1_DAT_21,
		MX53_PAD_EIM_D26__IPU_DISP1_DAT_22,
		MX53_PAD_EIM_D27__IPU_DISP1_DAT_23,
		MX53_PAD_EIM_DA6__IPU_DISP1_DAT_3,
		MX53_PAD_EIM_DA5__IPU_DISP1_DAT_4,
		MX53_PAD_EIM_DA4__IPU_DISP1_DAT_5,
		MX53_PAD_EIM_DA3__IPU_DISP1_DAT_6,
		MX53_PAD_EIM_DA2__IPU_DISP1_DAT_7,
		MX53_PAD_EIM_DA1__IPU_DISP1_DAT_8,
		MX53_PAD_EIM_DA0__IPU_DISP1_DAT_9,
		/*LDB*/
		MX53_PAD_LVDS0_CLK_P__LDB_LVDS0_CLK,
		MX53_PAD_LVDS0_TX0_P__LDB_LVDS0_TX0,
		MX53_PAD_LVDS0_TX1_P__LDB_LVDS0_TX1,
		MX53_PAD_LVDS0_TX2_P__LDB_LVDS0_TX2,
		/*OSC32K*/
		MX53_PAD_GPIO_10__OSC32k_32K_OUT,
		/*SJC*/
		//No Muxing
		/*SRC*/
		//No Muxing
		/*SRTC*/
		//No Muxing
		/*TCU*/
		//No Muxing
		/*UART1*/
		MX53_PAD_EIM_EB3__UART1_RI,
		MX53_PAD_PATA_DIOW__UART1_TXD_MUX,
		MX53_PAD_PATA_DMACK__UART1_RXD_MUX,
		/*UART2*/
		MX53_PAD_PATA_DMARQ__UART2_TXD_MUX,
		MX53_PAD_PATA_BUFFER_EN__UART2_RXD_MUX,
		/*UART3*/
		MX53_PAD_PATA_DA_1__UART3_CTS,
		MX53_PAD_PATA_DA_2__UART3_RTS,
		MX53_PAD_PATA_CS_0__UART3_TXD_MUX,
		MX53_PAD_PATA_CS_1__UART3_RXD_MUX,
		/*UART5*/
		MX53_PAD_KEY_COL1__UART5_TXD_MUX,
		MX53_PAD_KEY_ROW1__UART5_RXD_MUX,
		/*USBOH3*/
		MX53_PAD_KEY_ROW3__USBOH3_H2_DM,
		MX53_PAD_KEY_COL3__USBOH3_H2_DP,
		/* PWM */
		MX53_PAD_GPIO_1__PWM2_PWMO,
		MX53_PAD_GPIO_9__PWM1_PWMO,
		/*WDOG1*/
		//MX53_PAD_DISP0_DAT8__WDOG1_WDOG_B,
		MX53_PAD_DISP0_DAT8__GPIO4_29,
		/*CAM mclk*/
		MX53_PAD_GPIO_0__CCM_SSI_EXT1_CLK
};

static void quanta_da9053_irq_wakeup_only_fixup(void)
{
	void __iomem *tzic_base;
	tzic_base = ioremap(MX53_TZIC_BASE_ADDR, SZ_4K);
	if (NULL == tzic_base) {
		pr_err("fail to map MX53_TZIC_BASE_ADDR\n");
		return;
	}
	__raw_writel(0, tzic_base + TZIC_WAKEUP0_OFFSET);
	__raw_writel(0, tzic_base + TZIC_WAKEUP1_OFFSET);
	__raw_writel(0, tzic_base + TZIC_WAKEUP2_OFFSET);
	/* only enable irq wakeup for da9053 */
	__raw_writel(GPIO7_0_11_IRQ_BIT, tzic_base + TZIC_WAKEUP3_OFFSET);
	iounmap(tzic_base);
	pr_info("only da9053 irq is wakeup-enabled\n");
}

static void quanta_suspend_enter(void)
{
	if (!board_is_mx53_quanta_mc34708()) {
		quanta_da9053_irq_wakeup_only_fixup();
		da9053_suspend_cmd_sw();
	}
}

static void quanta_suspend_exit(void)
{
	if (!board_is_mx53_quanta_mc34708()) {
		if (da9053_get_chip_version())
			da9053_restore_volt_settings();
	}
}

static struct mxc_pm_platform_data quanta_pm_data = {
	.suspend_enter = quanta_suspend_enter,
	.suspend_exit = quanta_suspend_exit,
};

#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake, debounce_ms)	\
{																			\
	.gpio		= gpio_num,													\
	.type		= EV_KEY,													\
	.code		= ev_code,													\
	.active_low	= act_low,													\
	.desc		= "btn " descr,												\
	.wakeup		= wake,														\
	.debounce_interval = debounce_ms,										\
}

static struct gpio_keys_button quanta_buttons[] = {
	GPIO_BUTTON(GPIO19, KEY_MENU, 1, "menu", 0, 30),
	GPIO_BUTTON(GPIO20, KEY_BACK, 1, "back", 0, 30),
	GPIO_BUTTON(GPIO21, KEY_HOME, 1, "home", 1, 30),
};

static struct gpio_keys_platform_data quanta_button_data = {
	.buttons	= quanta_buttons,
	.nbuttons	= ARRAY_SIZE(quanta_buttons),
};

static struct platform_device quanta_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &quanta_button_data,
	}
};

#define GPIO_LED(gpio_num, def_state, desc)									\
{																			\
	.gpio		= gpio_num,													\
	.name		= desc, 													\
	.active_low = 0,														\
	.retain_state_suspended = 0,											\
	.default_state = def_state,												\
}

static struct gpio_led quanta_leds[] = {
		GPIO_LED(GPIO14, LEDS_GPIO_DEFSTATE_ON, "led1"),
		GPIO_LED(GPIO15, LEDS_GPIO_DEFSTATE_ON, "led2"),
		GPIO_LED(GPIO16, LEDS_GPIO_DEFSTATE_ON, "led3"),
		GPIO_LED(GPIO17, LEDS_GPIO_DEFSTATE_ON, "led4"),
		GPIO_LED(GPIO18, LEDS_GPIO_DEFSTATE_ON, "led5"),
};

static struct gpio_led_platform_data quanta_leds_data = {
		.leds = quanta_leds,
		.num_leds = ARRAY_SIZE(quanta_leds),
};

static struct platform_device quanta_leds_device = {
	.name		= "leds-gpio",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &quanta_leds_data,
	}
};

static struct fb_videomode video_modes[] = {
	{
	 /* 800x480 @ 57 Hz , pixel clk @ 27MHz */
	 "CLAA-WVGA", 57, 800, 480, 37037, 40, 60, 10, 10, 20, 10,
	 FB_SYNC_CLK_LAT_FALL,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* 800x480 @ 60 Hz , pixel clk @ 32MHz */
	 "SEIKO-WVGA", 60, 800, 480, 29850, 89, 164, 23, 10, 10, 10,
	 FB_SYNC_CLK_LAT_FALL,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	/* 1600x1200 @ 60 Hz 162M pixel clk*/
	"UXGA", 60, 1600, 1200, 6172,
	304, 64,
	1, 46,
	192, 3,
	FB_SYNC_HOR_HIGH_ACT|FB_SYNC_VERT_HIGH_ACT,
	FB_VMODE_NONINTERLACED,
	0,},
};

static struct platform_pwm_backlight_data mxc_pwm_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

extern void mx5_ipu_reset(void);
static struct mxc_ipu_config mxc_ipu_data = {
	.rev = 3,
	.reset = mx5_ipu_reset,
};

extern void mx5_vpu_reset(void);
static struct mxc_vpu_platform_data mxc_vpu_data = {
	.iram_enable = true,
	.iram_size = 0x14000,
	.reset = mx5_vpu_reset,
};

static struct fec_platform_data fec_data = {
	.phy = PHY_INTERFACE_MODE_RMII,
};

static struct mxc_dvfs_platform_data dvfs_core_data = {
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.gpc_vcr_offset = MXC_GPC_VCR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 30,
};

static struct mxc_bus_freq_platform_data bus_freq_data;

static struct ldb_platform_data ldb_data = {
	.ext_ref = 1,
	.boot_enable = MXC_LDBDI0,
};

static void mxc_iim_enable_fuse(void)
{
	u32 reg;

	if (!ccm_base)
		return;

	/* enable fuse blown */
	reg = readl(ccm_base + 0x64);
	reg |= 0x10;
	writel(reg, ccm_base + 0x64);
}

static void mxc_iim_disable_fuse(void)
{
	u32 reg;

	if (!ccm_base)
		return;
	/* enable fuse blown */
	reg = readl(ccm_base + 0x64);
	reg &= ~0x10;
	writel(reg, ccm_base + 0x64);
}

static struct mxc_iim_data iim_data = {
	.bank_start = MXC_IIM_MX53_BANK_START_ADDR,
	.bank_end   = MXC_IIM_MX53_BANK_END_ADDR,
	.enable_fuse = mxc_iim_enable_fuse,
	.disable_fuse = mxc_iim_disable_fuse,
};

static struct resource mxcfb_resources[] = {
	[0] = {
	       .flags = IORESOURCE_MEM,
	       },
};

static struct mxc_fb_platform_data fb_data[] = {
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB565,
	 .mode_str = "CLAA-WVGA",
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	 },
	{
	 .interface_pix_fmt = IPU_PIX_FMT_GBR24,
	 .mode_str = "VGA-XGA",
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	 },
};

extern int primary_di;

static int __init
mxc_init_fb(void)
{
	if (!machine_is_mx53_quanta())
		return 0;

	/*for quanta board, set default display as VGA*/
	if (primary_di < 0)
		primary_di = 1;

	if (primary_di) {
		printk(KERN_INFO "DI1 is primary\n");
		/* DI1 -> DP-BG channel: */
		mxc_fb_devices[1].num_resources = ARRAY_SIZE(mxcfb_resources);
		mxc_fb_devices[1].resource = mxcfb_resources;
		mxc_register_device(&mxc_fb_devices[1], &fb_data[1]);

		/* DI0 -> DC channel: */
		mxc_register_device(&mxc_fb_devices[0], &fb_data[0]);
	} else {
		printk(KERN_INFO "DI0 is primary\n");

		/* DI0 -> DP-BG channel: */
		mxc_fb_devices[0].num_resources = ARRAY_SIZE(mxcfb_resources);
		mxc_fb_devices[0].resource = mxcfb_resources;
		mxc_register_device(&mxc_fb_devices[0], &fb_data[0]);

		/* DI1 -> DC channel: */
		mxc_register_device(&mxc_fb_devices[1], &fb_data[1]);
	}

	/*
	 * DI0/1 DP-FG channel:
	 */
	mxc_register_device(&mxc_fb_devices[2], NULL);

	return 0;
}
device_initcall(mxc_init_fb);

static void sii902x_hdmi_reset(void)
{
	gpio_set_value(GPIO24, 0);
	msleep(10);
	gpio_set_value(GPIO24, 1);
	msleep(10);
}

static int sii902x_get_pins(void)
{
	/* Sii902x HDMI controller */
	gpio_request(GPIO24, "disp0-reset");
	gpio_direction_output(GPIO24, 0);
	gpio_request(GPIO23, "disp0-detect");
	gpio_direction_input(GPIO23);
	return 1;
}

static void sii902x_put_pins(void)
{
	gpio_free(GPIO24);
	gpio_free(GPIO23);
}

static struct mxc_lcd_platform_data sii902x_hdmi_data = {
	.reset = sii902x_hdmi_reset,
	.fb_id = "DISP3 BG",
	.get_pins = sii902x_get_pins,
	.put_pins = sii902x_put_pins,
};

static struct imxi2c_platform_data mxci2c_data = {
       .bitrate = 100000,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
		{
			.type = "sgtl5000-i2c",
			.addr = 0x0A,
		},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {


};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
		{
			.type = "sii902x",
			.addr = 0x39,
			.irq = gpio_to_irq(GPIO23),
			.platform_data = &sii902x_hdmi_data,
		},

};

static int sdhc_write_protect(struct device *dev)
{
	int ret = 0;

	if (to_platform_device(dev)->id == 2)
		ret = gpio_get_value(SD_WP);

	return ret;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret = 0;

	if (to_platform_device(dev)->id == 2)
		ret = gpio_get_value(SD_CD);

	return ret;
}

static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32 | MMC_VDD_32_33,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.wp_status = sdhc_write_protect,
	.status = sdhc_get_card_det_status,
	.card_inserted_state = 0,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

static struct mxc_mmc_platform_data mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA
		| MMC_CAP_DATA_DDR,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 1,
	.clock_mmc = "esdhc_clk",
};

static struct mxc_mmc_platform_data mmc3_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
	| MMC_VDD_31_32 | MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_34_35 |MMC_VDD_35_36,
	.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 1,
	.clock_mmc = "esdhc_clk",
};

static int headphone_det_status(void)
{
	return (gpio_get_value(GPIO25) == 0);
}

static int mxc_sgtl5000_init(void);

static struct mxc_audio_platform_data sgtl5000_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 5,
	.hp_irq = gpio_to_irq(GPIO25),
	.hp_status = headphone_det_status,
	.init = mxc_sgtl5000_init,
	.ext_ram_rx = 1,
};

static int mxc_sgtl5000_init(void)
{
	sgtl5000_data.sysclk = 12288000;
	return 0;
}

static struct platform_device mxc_sgtl5000_device = {
	.name = "imx-3stack-sgtl5000",
};

static struct mxc_asrc_platform_data mxc_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static void mx53_quanta_otghost_vbus(bool on)
{
	if (on)
		gpio_set_value(OTG_PWR_EN, 1);
	else
		gpio_set_value(OTG_PWR_EN, 0);
}

static void mxc_register_powerkey(pwrkey_callback pk_cb)
{
	pmic_event_callback_t power_key_event;

	power_key_event.param = (void *)1;
	power_key_event.func = (void *)pk_cb;
	pmic_event_subscribe(EVENT_PWRONI, power_key_event);
}

static int mxc_pwrkey_getstatus(int id)
{
	int sense;

	pmic_read_reg(REG_INT_SENSE1, &sense, 0xffffffff);
	if (sense & (1 << 3))
		return 0;

	return 1;
}

static struct power_key_platform_data pwrkey_data = {
	.key_value = KEY_POWER,
	.register_pwrkey = mxc_register_powerkey,
	.get_key_status = mxc_pwrkey_getstatus,
};

/*!
 * Board specific fixup function. It is called by \b setup_arch() in
 * setup.c file very early on during kernel starts. It allows the user to
 * statically fill in the proper values for the passed-in parameters. None of
 * the parameters is used currently.
 *
 * @param  desc         pointer to \b struct \b machine_desc
 * @param  tags         pointer to \b struct \b tag
 * @param  cmdline      pointer to the command line
 * @param  mi           pointer to \b struct \b meminfo
 */
static void __init
fixup_mxc_board(struct machine_desc *desc, struct tag *tags, char **cmdline, struct meminfo *mi)
{
	struct tag *t;
	struct tag *mem_tag = 0;
	int total_mem = SZ_1G;
	int left_mem = 0;
	int gpu_mem = SZ_128M;
	int fb_mem = SZ_32M;
	char *str;

	mxc_set_cpu_type(MXC_CPU_MX53);

	for_each_tag(mem_tag, tags) {
		if (mem_tag->hdr.tag == ATAG_MEM) {
			total_mem = mem_tag->u.mem.size;
			break;
		}
	}

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "mem=");
			if (str != NULL) {
				str += 4;
				left_mem = memparse(str, &str);
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_nommu");
			if (str != NULL)
				gpu_data.enable_mmu = 0;

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_memory=");
			if (str != NULL) {
				str += 11;
				gpu_mem = memparse(str, &str);
			}

			break;
		}
	}

	if (gpu_data.enable_mmu)
		gpu_mem = 0;

	if (left_mem == 0 || left_mem > total_mem)
		left_mem = total_mem - gpu_mem - fb_mem;

	if (mem_tag) {
		fb_mem = total_mem - left_mem - gpu_mem;
		if (fb_mem < 0) {
			gpu_mem = total_mem - left_mem;
			fb_mem = 0;
		}
		mem_tag->u.mem.size = left_mem;

		/*reserve memory for gpu*/
		if (!gpu_data.enable_mmu) {
			gpu_device.resource[5].start =
				mem_tag->u.mem.start + left_mem;
			gpu_device.resource[5].end =
				gpu_device.resource[5].start + gpu_mem - 1;
		}
#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
		if (fb_mem) {
			mxcfb_resources[0].start =
				gpu_data.enable_mmu ?
				mem_tag->u.mem.start + left_mem :
				gpu_device.resource[5].end + 1;
			mxcfb_resources[0].end =
				mxcfb_resources[0].start + fb_mem - 1;
		} else {
			mxcfb_resources[0].start = 0;
			mxcfb_resources[0].end = 0;
		}
#endif
	}
}

static void mx53_quanta_power_off(void)
{
	pr_info("Power off!\n");

	if(!board_is_mx53_quanta_mc34708())
	{
		da9053_power_off();
	}
}

static void __init
mx53_quanta_io_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx53_quanta_pads, ARRAY_SIZE(mx53_quanta_pads));

	/* SD1 */
	gpio_request(SD_CD, "SD_CD");
	gpio_direction_input(SD_CD);
	gpio_request(SD_WP, "SD_WP");
	gpio_direction_input(SD_WP);

	/* USB PWR enable */
	gpio_request(OTG_PWR_EN, "OTG_PWR_EN");
	gpio_direction_output(OTG_PWR_EN, 0);

	/* USB Hub RSTn */
	gpio_request(GPIO1, "USB_HUB_RSTn");
	gpio_direction_output(GPIO1, 1);

	/* PMIC_INT */
	gpio_request(QUANTA_SOM_PMIC_INT, "PMIC_INT");
	gpio_direction_input(QUANTA_SOM_PMIC_INT);

	/* HEADPHONE_DET */
	gpio_request(GPIO25, "HEADPHONE_DET");
	gpio_direction_input(GPIO25);

	/* LCD panel power enable */
	gpio_request(LCD_EN, "LCD_EN");
	gpio_direction_output(LCD_EN, 1);

	/* OSC_CKIH1_EN */
	gpio_request(OSC_CKIH1_EN, "OSC_CKIH1_EN");
	gpio_direction_output(OSC_CKIH1_EN, 1);
}

/*!
 * Board specific initialization.
 */
static void __init
mxc_board_init(void)
{

	pr_info("Initializing Turing's IMX53 Board\n");

	mxc_ipu_data.di_clk[0] = clk_get(NULL, "ipu_di0_clk");
	mxc_ipu_data.di_clk[1] = clk_get(NULL, "ipu_di1_clk");
	mxc_ipu_data.csi_clk[0] = clk_get(NULL, "ssi_ext1_clk");

	/*
	 *ssi_ext1_clk was enbled in arch/arm/mach-mx5/clock.c, and it was kept
	 *open to provide clock for audio codec on i.Mx53 Quickstart, but MX53
	 *QUANTA board have no needs to do that, so we close it here
	 */
	clk_disable(mxc_ipu_data.csi_clk[0]);

	mxc_cpu_common_init();
	mx53_quanta_io_init();

	mxcsdhc1_device.resource[2].start = gpio_to_irq(SD_CD);
	mxcsdhc1_device.resource[2].end = gpio_to_irq(SD_CD);

	mxc_register_device(&mxc_dma_device, NULL);
	mxc_register_device(&mxc_wdt_device, NULL);

	mxc_register_device(&mxci2c_devices[0], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[1], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[2], &mxci2c_data);

	mxc_register_device(&mx5_pmu_device, NULL);

	if (board_is_mx53_quanta_mc34708()) {
		pr_info("MX53-QUANTA v1.0 - CPU Rev B. MC34708\n");
		// PMIC MC34708
		mx53_quanta_init_mc34708();
		dvfs_core_data.reg_id = "SW1A";
		bus_freq_data.gp_reg_id = "SW1A";
		bus_freq_data.lp_reg_id = "SW2";
		mxc_register_device(&mxc_powerkey_device, &pwrkey_data);
	} else {
		pr_info("MX53-QUANTA EVB v1.0 - CPU Rev D. DA9053\n");
		// PMIC DA9053
		mx53_quanta_init_da9052();
		dvfs_core_data.reg_id = "DA9052_BUCK_CORE";
		bus_freq_data.gp_reg_id = "DA9052_BUCK_CORE";
		bus_freq_data.lp_reg_id = "DA9052_BUCK_PRO";
	}

	mxc_register_device(&mxc_rtc_device, NULL);
	mxc_register_device(&mxc_ipu_device, &mxc_ipu_data);
	mxc_register_device(&mxc_ldb_device, &ldb_data);

	if (!mxc_fuse_get_vpu_status())
		mxc_register_device(&mxcvpu_device, &mxc_vpu_data);
	if (!mxc_fuse_get_gpu_status())
		mxc_register_device(&gpu_device, &gpu_data);

	mxc_register_device(&mxcscc_device, NULL);
	mxc_register_device(&pm_device, &quanta_pm_data);
	mxc_register_device(&mxc_dvfs_core_device, &dvfs_core_data);
	mxc_register_device(&busfreq_device, &bus_freq_data);
	mxc_register_device(&mxc_iim_device, &iim_data);
	mxc_register_device(&mxc_pwm2_device, NULL);
	mxc_register_device(&mxc_pwm1_backlight_device, &mxc_pwm_backlight_data);
	mxc_register_device(&mxcsdhc1_device, &mmc1_data);
	mxc_register_device(&mxcsdhc2_device, &mmc2_data);
	mxc_register_device(&mxcsdhc3_device, &mmc3_data);
	mxc_register_device(&mxc_ssi1_device, NULL);
	mxc_register_device(&mxc_ssi2_device, NULL);
	mxc_register_device(&ahci_fsl_device, &sata_data);
	mxc_register_device(&imx_ahci_device_hwmon, NULL);
	mxc_register_device(&mxc_fec_device, &fec_data);
	mxc_register_device(&mxc_ptp_device, NULL);

	/* ASRC is only available for MX53 TO2.0 */
	if (mx53_revision() >= IMX_CHIP_REVISION_2_0) {
		mxc_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
		clk_put(mxc_asrc_data.asrc_core_clk);
		mxc_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
		clk_put(mxc_asrc_data.asrc_audio_clk);
		mxc_register_device(&mxc_asrc_device, &mxc_asrc_data);
	}

	i2c_register_board_info(0, mxc_i2c0_board_info, ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info, ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info, ARRAY_SIZE(mxc_i2c2_board_info));

	sgtl5000_data.ext_ram_clk = clk_get(NULL, "emi_fast_clk");
	clk_put(sgtl5000_data.ext_ram_clk);
	mxc_register_device(&mxc_sgtl5000_device, &sgtl5000_data);

	mx5_usb_dr_init();
	mx5_set_otghost_vbus_func(mx53_quanta_otghost_vbus);
	mx5_usbh1_init();
	mxc_register_device(&mxc_v4l2_device, NULL);
	mxc_register_device(&mxc_v4l2out_device, NULL);

	platform_device_register(&quanta_button_device);
	platform_device_register(&quanta_leds_device);

	pm_power_off = mx53_quanta_power_off;
	pm_i2c_init(I2C1_BASE_ADDR - MX53_OFFSET);
}

static void __init
mx53_quanta_timer_init(void)
{
	struct clk *uart_clk;

	mx53_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("mxcintuart.4", NULL);
	early_console_setup(MX53_BASE_ADDR(UART5_BASE_ADDR), uart_clk);
}

static struct sys_timer mxc_timer = {
	.init	= mx53_quanta_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX53_QUANTA data structure.
 */
MACHINE_START(MX53_QUANTA, "MX53 QUANTA Board")
	/* Maintainer: Turing Computer */
	.fixup = fixup_mxc_board,
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
