#include <linux/ioport.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <asm/io.h>
#include <mach/gpio.h>


#define AM33XX_CONTROL_BASE		0x44e10000

/*
 * default config for beaglebone
 *
 *	p9/27:	rfm12b IRQ
 *  p9/29:  mosi/miso
 *  p9/30:  mosi/miso
 *  p9/31:  sclk
 *  p9/42:  cs
 */

static const u16 am33xx_irq_pin = 115;	// gpio3_19
static const u16 am33xx_spi_bus = 2;	// spi port on bbone header
static const u16 am33xx_spi_cs	= 1;	// CS 0

struct am33xx_config {
	u16 irq;
	struct spi_device* spi_device;
	struct {
		u8 irq_active:1;
		u8 gpio_claimed:1;
	} state;
};

struct am33xx_pin_config {
	u32 pin_addr;
	u32 settings;
	u8 claimed;
};

static struct am33xx_pin_config am33xx_pin_configs[] = {
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
		.settings = 0x3 | (2 << 3) | (1 << 5),
		.claimed = 0
	},
	{
		// beaglebone pin p9/31 (mcasp0_aclkx)
		//		set as SPI1_SCLK (mode 3), PULLUP ENABLED, INPUT DISABLED
		.pin_addr = AM33XX_CONTROL_BASE + 0x990,
		.settings = 0x3 | (2 << 3),
		.claimed = 0
	},
	{ 0, 0}
};

static struct am33xx_config am33xx_conf;

static irqreturn_t
am33xx_irq_handler(int irq, void* dev_id);

static int
am33xx_init_pinmux_settings(struct rfm12_data* dev_data);
static int
am33xx_cleanup_pinmux_settings(struct rfm12_data* dev_data);

static int
am33xx_setup_irq_pin(struct rfm12_data* dev_data);
static int
am33xx_cleanup_irq_pin(struct rfm12_data* dev_data);

static int
am33xx_register_spi_device(struct rfm12_data* dev_data);
static int
am33xx_deregister_spi_device(struct rfm12_data* dev_data);

static irqreturn_t
am33xx_irq_handler(int irq, void* dev_id)
{
	return IRQ_HANDLED;
}

static int
am33xx_init_pinmux_settings(struct rfm12_data* dev_data)
{	
	void* addr = NULL;
	struct am33xx_pin_config* pin_conf = &am33xx_pin_configs[0];
	
	(void)dev_data;
	
	while (0 != pin_conf->pin_addr) {
		if (pin_conf->claimed) continue;
		
		if (NULL == request_mem_region(pin_conf->pin_addr, 4, RFM12B_DRV_NAME)) {
			printk(KERN_ALERT RFM12B_DRV_NAME
				" : unable to obtain I/O memory address 0x%x during pin muxing.\n",
				pin_conf->pin_addr
			);
		
			(void)am33xx_cleanup_pinmux_settings(dev_data);
		
			return -EBUSY;
		}
		
		addr = ioremap(pin_conf->pin_addr, 4);
		if (NULL == addr) {
			printk(KERN_ALERT RFM12B_DRV_NAME
				" : unable to ioremap memory address 0x%x during pin muxing.\n",
				pin_conf->pin_addr
			);
			
			(void)am33xx_cleanup_pinmux_settings(dev_data);
			
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
am33xx_cleanup_pinmux_settings(struct rfm12_data* dev_data)
{	
	struct am33xx_pin_config* pin_conf = &am33xx_pin_configs[0];

	(void)dev_data;
	
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
am33xx_setup_irq_pin(struct rfm12_data* dev_data)
{
	int err;
	
	err = gpio_request_one(am33xx_irq_pin, GPIOF_IN, RFM12B_DRV_NAME " irq pin");
	if (0 != err) {
		printk(KERN_ALERT RFM12B_DRV_NAME
			" : unable to obtain GPIO pin %u.\n",
			am33xx_irq_pin
		);
		
		goto errReturn;
	}
	
	am33xx_conf.state.gpio_claimed = 1;

	err = gpio_to_irq(am33xx_irq_pin);
	if (err < 0) {
		printk(
			KERN_ALERT RFM12B_DRV_NAME
			" : unable to obtain IRQ for GPIO pin %u: %i.\n",
			am33xx_irq_pin, err
		);
		
		goto gpioErrReturn;
	}
	
	dev_data->irq = (u16)err;
	am33xx_conf.irq = dev_data->irq;
	
	err = 0;
	return err;

gpioErrReturn:
	gpio_free(am33xx_irq_pin);
	am33xx_conf.state.gpio_claimed = 0;

errReturn:	
	return err;
}

static int
am33xx_cleanup_irq_pin(struct rfm12_data* dev_data)
{
	(void)dev_data;
	(void)platform_irq_cleanup(dev_data);
	
	if (am33xx_conf.state.gpio_claimed) {
		gpio_free(am33xx_irq_pin);
		am33xx_conf.state.gpio_claimed = 0;
	}
	
	return 0;
}

static int
platform_module_init(struct rfm12_data* dev_data)
{
	int err;
	
	err = am33xx_init_pinmux_settings(dev_data);
	if (0 != err) goto muxFailed;
	
	err = am33xx_setup_irq_pin(dev_data);
	if (0 != err) goto irqFailed;
	
	err = am33xx_register_spi_device(dev_data);
	if (0 != err) goto spiFailed;
	
	return err;

spiFailed:
	am33xx_cleanup_irq_pin(dev_data);
irqFailed:
	am33xx_cleanup_pinmux_settings(dev_data);
muxFailed:
	return err;
}

static int
platform_module_cleanup(struct rfm12_data* dev_data)
{
	(void)am33xx_cleanup_pinmux_settings(dev_data);
	(void)am33xx_cleanup_irq_pin(dev_data);
	(void)am33xx_deregister_spi_device(dev_data);
	
	return 0;
}

static int
platform_irq_init(struct rfm12_data* dev_data)
{
	int err;
	
	(void)dev_data;
	
	if (am33xx_conf.state.irq_active) return -EBUSY;
	
	err = request_irq(am33xx_conf.irq,
					  am33xx_irq_handler,
					  IRQF_DISABLED,
					  RFM12B_DRV_NAME,
					  (void*)dev_data);
	
	if (0 == err)
		am33xx_conf.state.irq_active = 1;
	else
		printk(
			KERN_ALERT RFM12B_DRV_NAME
			" : unable to activate IRQ %u: %i.\n",
			am33xx_conf.irq, err
		);
	
	return err;	
}

static int
platform_irq_cleanup(struct rfm12_data* dev_data)
{
	int err = 0;
	
	(void)dev_data;
	
	if (am33xx_conf.state.irq_active) {
		free_irq(am33xx_conf.irq, (void*)dev_data);
		am33xx_conf.state.irq_active = 0;
	}
	
	return err;
}

static int
am33xx_register_spi_device(struct rfm12_data* dev_data)
{
	int err = 0;
	struct spi_master* spi_master;
	struct spi_device* spi_device;
	struct device* sdev;
	char buf[128];
	
	spi_master = spi_busnum_to_master(am33xx_spi_bus);
	if (NULL == spi_master) {
		err = -ENODEV;
		printk(
			KERN_ALERT RFM12B_DRV_NAME
			" : no spi_master found for busnum %u.\n",
			am33xx_spi_bus
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
		goto freeMaster;
	}
	
	spi_device->chip_select = am33xx_spi_cs;
	
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
		goto freeMaster;
	}
	
	spi_device->max_speed_hz = RFM12B_SPI_MAX_HZ;
	spi_device->mode = RFM12B_SPI_MODE;
	spi_device->bits_per_word = RFM12B_SPI_BITS;
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
		am33xx_conf.spi_device = spi_device;
	
freeMaster:
	put_device(&spi_master->dev);
errReturn:
	return err;
}

static int
am33xx_deregister_spi_device(struct rfm12_data* dev_data)
{
	if (NULL != am33xx_conf.spi_device) {
		spi_unregister_device(am33xx_conf.spi_device);
		spi_dev_put(am33xx_conf.spi_device);
		
		am33xx_conf.spi_device = NULL;
	}
	
	return 0;
}
