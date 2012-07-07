#if !defined(__PLATFORM_H__)

static int
platform_module_init(void);

static int
platform_module_cleanup(void);

static int
platform_irq_init(void* ctx);

static int
platform_irq_handled(void* ctx);

static int
platform_irq_cleanup(void* ctx);

#endif
