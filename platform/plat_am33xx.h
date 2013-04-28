#if !defined(__RFM12_PLAT_AM33XX_H__)
#define __RFM12_PLAT_AM33XX_H__

#include <linux/ioport.h>
#include <asm/io.h>

// the amount of RFM12 modules connected to the beaglebone
// per default, we have settings for one board. if you want
// more boards, you need to add the necessary settings below
// as well.
#define NUM_RFM12_BOARDS			1

/*
* default config for beaglebone (one RFM12 module)
*
*	p9/27:	rfm12b IRQ
*  p9/29:  mosi/miso
*  p9/30:  mosi/miso
*  p9/31:  sclk
*  p9/42:  cs
*/

struct am33xx_board_config board_configs[NUM_RFM12_BOARDS] = {
	{
		.irq_pin		= 115,	// gpio3_19
		.spi_bus		= 2,	// spi port on beaglebone header
		.spi_cs			= 1		// CS 1
	}
};

struct am33xx_pinmux_settings {
	u32 pin_addr;
	u32 settings;
	u8 claimed;
};

#define AM33XX_CONTROL_BASE		0x44e10000

// these pinmux settings will be applied when the module is loaded.
// if you want to connect multiple boards, add the definitions
// and settings for the required pins here.
static struct am33xx_pinmux_settings pinmux_settings[] = {
	{
		// beaglebone pin p9/27 (mcasp0_fsr)
		//		set as GPIO3_19 (mode 7), PULLUP ENABLED, INPUT ENABLED 
		.pin_addr = AM33XX_CONTROL_BASE + 0x9a4,
		.settings = 0x7 | (2 << 3) | (1 << 5),
		.claimed = 0
	},
	{
		// beaglebone pin p9/42 (ecap0_in_pwm0_out)
		//		set as SPI1_CS1 (mode 2), PULLUP ENABLED, INPUT ENABLED
		.pin_addr = AM33XX_CONTROL_BASE + 0x964,
		.settings = 0x2 | (2 << 3) | (1 << 5),
		.claimed = 0
	},
	{
		// beaglebone pin p9/29 (mcasp0_fsx)
		//		set as SPI1_D0 (mode 3), PULLUP ENABLED, INPUT ENABLED
		.pin_addr = AM33XX_CONTROL_BASE + 0x994,
		.settings = 0x3 | (2 << 3) | (1 << 5),
		.claimed = 0
	},
	{
		// beaglebone pin p9/30 (mcasp0_axr0)
		//		set as SPI1_D1 (mode 3), PULLUP ENABLED, INPUT ENABLED
		.pin_addr = AM33XX_CONTROL_BASE + 0x998,
		.settings = 0x3 | (0 << 3) | (1 << 5),
		.claimed = 0
	},
	{
		// beaglebone pin p9/31 (mcasp0_aclkx)
		//		set as SPI1_SCLK (mode 3), PULLUP ENABLED, INPUT DISABLED
		.pin_addr = AM33XX_CONTROL_BASE + 0x990,
		.settings = 0x3 | (0 << 3) | (1 << 5),
		.claimed = 0
	},
	{ 0, 0}
};

static int
spi_init_pinmux_settings(void);
static int
spi_cleanup_pinmux_settings(void);

static int
spi_init_pinmux_settings(void)
{	
	void* addr = NULL;
	struct am33xx_pinmux_settings* pin_conf = &pinmux_settings[0];
		
	while (0 != pin_conf->pin_addr) {
		if (pin_conf->claimed) continue;
		
		if (NULL == request_mem_region(pin_conf->pin_addr, 4, RFM12B_DRV_NAME)) {
			printk(KERN_ALERT RFM12B_DRV_NAME
				" : unable to obtain I/O memory address 0x%x during pin muxing.\n",
				pin_conf->pin_addr
			);
		
			(void)spi_cleanup_pinmux_settings();
		
			return -EBUSY;
		}
		
		addr = ioremap(pin_conf->pin_addr, 4);
		if (NULL == addr) {
			printk(KERN_ALERT RFM12B_DRV_NAME
				" : unable to ioremap memory address 0x%x during pin muxing.\n",
				pin_conf->pin_addr
			);
			
			(void)spi_cleanup_pinmux_settings();
			
			return -EBUSY;
		}
		
		iowrite32(pin_conf->settings, addr);
		iounmap(addr);
		
		pin_conf->claimed = 1;
		pin_conf++;
	}
	
	return 0;
}

static int
spi_cleanup_pinmux_settings(void)
{	
	struct am33xx_pinmux_settings* pin_conf = &pinmux_settings[0];
	
	while (0 != pin_conf->pin_addr) {
		if (pin_conf->claimed) {
			release_mem_region(pin_conf->pin_addr, 4);
			pin_conf->claimed = 0;
		}
		
		pin_conf++;
	}
	
	return 0;
}

#endif // __RFM12_PLAT_AM33XX_H__
