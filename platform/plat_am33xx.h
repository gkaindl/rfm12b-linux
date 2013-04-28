#if !defined(__RFM12_PLAT_AM33XX_H__)
#define __RFM12_PLAT_AM33XX_H__

#include <linux/ioport.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/spi/spi.h>
#include <asm/io.h>
#include <mach/gpio.h>

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

struct am33xx_board_config {
	u16 irq_pin;
	u16 spi_bus;
	u16 spi_cs;
};

struct am33xx_board_config board_configs[NUM_RFM12_BOARDS] = {
	{
		.irq_pin		= 115,	// gpio3_19
		.spi_bus		= 2,	// spi port on beaglebone header
		.spi_cs			= 1		// CS 1
	}
};

static const u16 am33xx_irq_pin = 115;	// gpio3_19

#define AM33XX_CONTROL_BASE		0x44e10000

struct am33xx_pinmux_settings {
	u32 pin_addr;
	u32 settings;
	u8 claimed;
};

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

struct am33xx_active_board {
	u16 irq;
	void* irq_data;
	struct spi_device* spi_device;
	struct {
		u8 gpio_claimed:1;
		u8 irq_claimed:1;
		u8 irq_enabled:1;
	} state;
};

static struct am33xx_active_board active_boards[NUM_RFM12_BOARDS];

static irqreturn_t
am33xx_irq_handler(int irq, void* dev_id);

static int
am33xx_init_pinmux_settings(void);
static int
am33xx_cleanup_pinmux_settings(void);

static int
am33xx_setup_irq_pins(void);
static int
am33xx_cleanup_irq_pins(void);

static int
am33xx_register_spi_devices(void);
static int
am33xx_deregister_spi_devices(void);

static irqreturn_t
am33xx_irq_handler(int irq, void* dev_id)
{
	int idx = (int)dev_id;
	
	if (idx >= 0 && idx < NUM_RFM12_BOARDS) {
		struct am33xx_active_board* brd = &active_boards[idx];
		
		if (brd->state.irq_enabled) {
			brd->state.irq_enabled = 0;
			disable_irq_nosync(brd->irq);
		}
		
		rfm12_handle_interrupt((struct rfm12_data*)brd->irq_data);
	}
	
	return IRQ_HANDLED;
}

static int
platform_irq_handled(int identifier)
{
	struct am33xx_active_board* brd = &active_boards[identifier];
	struct am33xx_board_config* cfg = &board_configs[identifier];
	
	if (identifier < 0 || identifier > NUM_RFM12_BOARDS) {
		return -ENODEV;
	}
	
	if (0 == brd->state.irq_enabled) {
		if (0 == gpio_get_value(cfg->irq_pin))
			rfm12_handle_interrupt((struct rfm12_data*)brd->irq_data);
		else {
			brd->state.irq_enabled = 1;
			enable_irq(brd->irq);
		}
	}

	return 0;
}

static int
am33xx_init_pinmux_settings(void)
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
		
			(void)am33xx_cleanup_pinmux_settings();
		
			return -EBUSY;
		}
		
		addr = ioremap(pin_conf->pin_addr, 4);
		if (NULL == addr) {
			printk(KERN_ALERT RFM12B_DRV_NAME
				" : unable to ioremap memory address 0x%x during pin muxing.\n",
				pin_conf->pin_addr
			);
			
			(void)am33xx_cleanup_pinmux_settings();
			
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
am33xx_cleanup_pinmux_settings(void)
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

static int
am33xx_setup_irq_pins(void)
{
	int err, i;
		
	for (i=0; i<NUM_RFM12_BOARDS; i++) {
		err = gpio_request_one(board_configs[i].irq_pin, GPIOF_IN,
				RFM12B_DRV_NAME " irq pin");
		if (0 != err) {
			printk(KERN_ALERT RFM12B_DRV_NAME
				" : unable to obtain GPIO pin %u.\n",
				board_configs[i].irq_pin
			);
			
			goto errReturn;
		}
		
		active_boards[i].state.gpio_claimed = 1;
	
		err = gpio_to_irq(board_configs[i].irq_pin);
		if (err < 0) {
			printk(
				KERN_ALERT RFM12B_DRV_NAME
				" : unable to obtain IRQ for GPIO pin %u: %i.\n",
				board_configs[i].irq_pin, err
			);
			
			goto gpioErrReturn;
		}
		
		active_boards[i].irq = (u16)err;
	}
	
	err = 0;
	return err;

gpioErrReturn:
	while (i >= 0) {
		gpio_free(board_configs[i].irq_pin);
		active_boards[i].state.gpio_claimed = 0;
		i--;
	}

errReturn:	
	return err;
}

static int
am33xx_cleanup_irq_pins(void)
{
	int i;
	
	for (i=0; i<NUM_RFM12_BOARDS; i++) {
		(void)platform_irq_cleanup(i);
		
		if (active_boards[i].state.gpio_claimed) {
			gpio_free(board_configs[i].irq_pin);
			active_boards[i].state.gpio_claimed = 0;
		}
	}
	
	return 0;
}

static int
platform_module_init(void)
{
	int err;
	
	err = am33xx_init_pinmux_settings();
	if (0 != err) goto muxFailed;
	
	err = am33xx_setup_irq_pins();
	if (0 != err) goto irqFailed;
	
	err = am33xx_register_spi_devices();
	if (0 != err) goto spiFailed;
	
	return err;

spiFailed:
	am33xx_cleanup_irq_pins();
irqFailed:
	am33xx_cleanup_pinmux_settings();
muxFailed:
	return err;
}

static int
platform_module_cleanup(void)
{
	(void)am33xx_cleanup_pinmux_settings();
	(void)am33xx_cleanup_irq_pins();
	(void)am33xx_deregister_spi_devices();
	
	return 0;
}

static int
platform_irq_identifier_for_spi_device(u16 spi_bus, u16 spi_cs)
{
	int i;
	
	for (i=0; i<NUM_RFM12_BOARDS; i++) {
		if (spi_bus == board_configs[i].spi_bus &&
			spi_cs == board_configs[i].spi_cs)
			return i;
	}
	
	return -1;
}

static int
platform_irq_init(int identifier, void* rfm12_data)
{
	int err;
	struct am33xx_active_board* brd = &active_boards[identifier];
	struct am33xx_board_config* cfg = &board_configs[identifier];

	if (identifier < 0 || identifier > NUM_RFM12_BOARDS)
		return -ENODEV;

	if (brd->state.irq_claimed)
		return -EBUSY;

	err = request_irq(
		brd->irq,
		am33xx_irq_handler,
		IRQF_TRIGGER_FALLING | IRQF_DISABLED,
		RFM12B_DRV_NAME,
		(void*)identifier
	);

	if (0 == err) {
		brd->state.irq_claimed = 1;
		brd->state.irq_enabled = 1;
		brd->irq_data = rfm12_data;
	} else
		printk(
			KERN_ALERT RFM12B_DRV_NAME
			" : unable to activate IRQ %u: %i.\n",
			brd->irq, err
		);

	if (0 == gpio_get_value(cfg->irq_pin))
		am33xx_irq_handler(brd->irq, (void*)identifier);

	return err;	
}

static int
platform_irq_cleanup(int identifier)
{
	int err = 0;
	
	if (identifier < 0 || identifier > NUM_RFM12_BOARDS) {
		err = -ENODEV;
	} else {
		struct am33xx_active_board* brd = &active_boards[identifier];
		
		if (brd->state.irq_claimed) {
			free_irq(brd->irq, (void*)identifier);
			brd->state.irq_claimed = 0;
			brd->irq_data = NULL;
		}
	}
	
	return err;
}

static int
am33xx_register_spi_devices(void)
{
	int i, err = 0;
	struct spi_master* spi_master;
	struct spi_device* spi_device;
	struct device* sdev;
	char buf[128];
	
	for (i=0; i<NUM_RFM12_BOARDS; i++) {
		spi_master = spi_busnum_to_master(board_configs[i].spi_bus);
		if (NULL == spi_master) {
			err = -ENODEV;
			printk(
				KERN_ALERT RFM12B_DRV_NAME
					" : no spi_master found for busnum %u.\n",
					board_configs[i].spi_bus
			);
			
			goto errReturn;
		}
		
		spi_device = spi_alloc_device(spi_master);
		if (NULL == spi_device) {
			printk(
				KERN_ALERT RFM12B_DRV_NAME
					" : spi_alloc_device() failed.\n"
			);
			err = -ENOMEM;
			goto errReturn;
		}
		
		spi_device->chip_select = board_configs[i].spi_cs;
		
		snprintf(
			buf,
			sizeof(buf),
			"%s.%u", 
			dev_name(&spi_device->master->dev),
			spi_device->chip_select
		);
		
		sdev = bus_find_device_by_name(spi_device->dev.bus, NULL, buf);
		if (NULL != sdev) {
			spi_dev_put(spi_device);
			
			printk(
				KERN_ALERT RFM12B_DRV_NAME
					" : driver [%s] already registered for [%s]\n",
					(sdev->driver && sdev->driver->name) ?
						sdev->driver->name : "unknown",
					buf
			);
			
			err = -EBUSY;
			goto errReturn;
		}
		
		spi_device->max_speed_hz = RFM12B_SPI_MAX_HZ;
		spi_device->mode = RFM12B_SPI_MODE;
		spi_device->bits_per_word = RFM12B_SPI_BITS;
		spi_device->chip_select = board_configs[i].spi_cs;
		spi_device->irq = -1; /* we do our own interrupt handling */
		spi_device->controller_state = NULL;
		spi_device->controller_data = NULL;
		strlcpy(spi_device->modalias, RFM12B_DRV_NAME, SPI_NAME_SIZE);
		
		err = spi_add_device(spi_device);
		if (0 != err) {
			spi_dev_put(spi_device);
			
			printk(
				KERN_ALERT RFM12B_DRV_NAME
					" : failed to register SPI device: %i\n",
					err
			);
		} else
			active_boards[i].spi_device = spi_device;
		
		put_device(&spi_master->dev);
		spi_master = NULL;
	}
	
	return err;
	
errReturn:
	if (NULL != spi_master)
		put_device(&spi_master->dev);

	(void)am33xx_deregister_spi_devices();

	return err;
}

static int
am33xx_deregister_spi_devices(void)
{
	int i;
	
	for (i=0; i<NUM_RFM12_BOARDS; i++) {
		if (NULL != active_boards[i].spi_device) {
			spi_unregister_device(active_boards[i].spi_device);
			spi_dev_put(active_boards[i].spi_device);
			
			active_boards[i].spi_device = NULL;
		}
	}
	
	return 0;
}

#endif // __RFM12_PLAT_AM33XX_H__
