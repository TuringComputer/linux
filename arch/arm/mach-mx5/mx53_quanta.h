/*
 * mx53_quanta.h
 *
 *  Created on: Jul 17, 2015
 *      Author: mauricio
 */

#ifndef MX53_QUANTA_H_
#define MX53_QUANTA_H_

#define GPIOx(port, pin)			(((port - 1) * 32) + pin)

/* MX53 QUANTA SOM-IMX53 GPIO PIN configurations */
// General Purpose I/Os
#define QUANTA_SOM_GPIO00			GPIOx(3, 13)
#define QUANTA_SOM_GPIO01			GPIOx(4, 5)
#define QUANTA_SOM_GPIO02			GPIOx(5, 16)
#define QUANTA_SOM_GPIO03			GPIOx(4, 31)
#define QUANTA_SOM_GPIO04			GPIOx(5, 5)
#define QUANTA_SOM_GPIO05			GPIOx(4, 27)
#define QUANTA_SOM_GPIO06			GPIOx(5, 4)
#define QUANTA_SOM_GPIO07			GPIOx(4, 19)
#define QUANTA_SOM_GPIO08			GPIOx(7, 3)
#define QUANTA_SOM_GPIO09			GPIOx(4, 21)
#define QUANTA_SOM_GPIO10			GPIOx(5, 6)
#define QUANTA_SOM_GPIO11			GPIOx(4, 23)
#define QUANTA_SOM_GPIO12			GPIOx(4, 22)
#define QUANTA_SOM_GPIO13			GPIOx(5, 17)
#define QUANTA_SOM_GPIO14			GPIOx(5, 7)
#define QUANTA_SOM_GPIO15			GPIOx(4, 30)
#define QUANTA_SOM_GPIO16			GPIOx(5, 8)
#define QUANTA_SOM_GPIO17			GPIOx(5, 9)
#define QUANTA_SOM_GPIO18			GPIOx(4, 10)
#define QUANTA_SOM_GPIO19			GPIOx(4, 18)
#define QUANTA_SOM_GPIO20			GPIOx(5, 15)
#define QUANTA_SOM_GPIO21			GPIOx(4, 16)
#define QUANTA_SOM_GPIO22			GPIOx(3, 23)
#define QUANTA_SOM_GPIO23			GPIOx(3, 28)
#define QUANTA_SOM_GPIO24			GPIOx(3, 22)
#define QUANTA_SOM_GPIO25			GPIOx(3, 20)
#define QUANTA_SOM_GPIO26			GPIOx(2, 14)
#define QUANTA_SOM_GPIO27			GPIOx(2, 25)
// USB OTG I/Os
#define QUANTA_SOM_OTG_PWR_EN		GPIOx(2, 5)
#define QUANTA_SOM_OTG_OC			GPIOx(2, 7)
// SDIO1 Card Detect and Write Protect pins
#define QUANTA_SOM_SD1_CD			GPIOx(1, 7)
#define QUANTA_SOM_SD1_WP			GPIOx(1, 9)
// PMIC Interrupt
#define QUANTA_SOM_PMIC_INT			GPIOx(7, 11)

#define MX53_OFFSET					(0x20000000)

#define TZIC_WAKEUP0_OFFSET         (0x0E00)
#define TZIC_WAKEUP1_OFFSET         (0x0E04)
#define TZIC_WAKEUP2_OFFSET         (0x0E08)
#define TZIC_WAKEUP3_OFFSET         (0x0E0C)
#define GPIO7_0_11_IRQ_BIT			(0x1<<11)

extern void pm_i2c_init(u32 base_addr);

extern int __init mx53_quanta_init_da9052(void);
extern int __init mx53_quanta_init_mc34708(void);

#endif /* QUANTA_SOM_IMX53_H_ */

