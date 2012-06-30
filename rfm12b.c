#include <linux/module.h>
#include <linux/kernel.h>


#define RFM12B_DRV_NAME		"RFM12B"

#define RFM12B_SPI_MAX_HZ	2000000
#define RFM12B_SPI_MODE		SPI_MODE_0
#define RFM12B_SPI_BITS		8

struct rfm12_data {
	u16			irq;
};

static int platform_module_init(struct rfm12_data* dev_data);
static int platform_irq_init(struct rfm12_data* dev_data);
static int platform_irq_cleanup(struct rfm12_data* dev_data);
static int platform_module_cleanup(struct rfm12_data* dev_data);

#include "platform/plat_am33xx.h"

struct rfm12_data dev_data;

int __init init_module(void)
{
	int err;
	
	err = platform_module_init(&dev_data);
	
	printk(KERN_INFO "Hello world %u.\n", err);
	
	return 0;
}

void __exit cleanup_module(void)
{
	int err;
	
	err = platform_module_cleanup(&dev_data);
	
	printk(KERN_INFO "Goodbye world 1.\n");
}

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Georg Kaindl <gkaindl (AT) mac.com>");
MODULE_DESCRIPTION("kernel driver for rfm12b digital radio module");
MODULE_VERSION("0.0.1");
