#if !defined(__PLATFORM_H__)

static int
platform_module_init(struct rfm12_data* dev_data);

static int
platform_irq_init(struct rfm12_data* dev_data);

static int
platform_irq_cleanup(struct rfm12_data* dev_data);

static int
platform_module_cleanup(struct rfm12_data* dev_data);

#endif
