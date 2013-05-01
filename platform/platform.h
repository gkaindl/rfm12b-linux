#if !defined(__PLATFORM_H__)
#define __PLATFORM_H__

static int
platform_module_init(void);

static int
platform_module_cleanup(void);

static int
platform_irq_identifier_for_spi_device(u16 spi_bus, u16 spi_cs);

static int
platform_irq_init(int identifier, void* rfm12_data);

static int
platform_irq_handled(int identifier);

static int
platform_irq_cleanup(int identifier);

struct spi_rfm12_board_config {
	u16 irq_pin;
	u16 spi_bus;
	u16 spi_cs;
};

struct rfm12_data;

static void
rfm12_handle_interrupt(struct rfm12_data* rfm12);

#endif
