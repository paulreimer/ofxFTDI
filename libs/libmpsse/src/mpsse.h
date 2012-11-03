#ifndef _LIBMPSSE_H_ 
#define _LIBMPSSE_H_

#include <ftdi.h>
#include <stdint.h>

#ifdef SWIGPYTHON
#include "swig.h"
#define MPSSE_READ_FUNCTION_PTR 	swig_string_data (*Read)(struct mpsse_context *mpsse, int size);
#define MPSSE_MCU_READ_FUNCTION_PTR	swig_string_data (*Read)(struct mpsse_context *mpsse, int size, int address);
#define MPSSE_TRANSFER_FUNCTION_PTR	swig_string_data (*Transfer)(struct mpsse_context *mpsse, char *data, int size);
#else
#define MPSSE_READ_FUNCTION_PTR		char *(*Read)(struct mpsse_context *mpsse, int size);
#define MPSSE_MCU_READ_FUNCTION_PTR	char *(*Read)(struct mpsse_context *mpsse, int size, int address);
#define MPSSE_TRANSFER_FUNCTION_PTR	char *(*Transfer)(struct mpsse_context *mpsse, char *data, int size);
#endif

#define MPSSE_OK		0
#define MPSSE_FAIL		-1

#define MSB			0x00
#define LSB			0x08

#define CHUNK_SIZE		65535
#define SPI_RW_SIZE		(63 * 1024) 
#define SPI_TRANSFER_SIZE	1024
#define I2C_TRANSFER_SIZE	64

#define LATENCY_MS		2
#define TIMEOUT_DIVISOR		1000000
#define USB_TIMEOUT		120000
#define SETUP_DELAY		25000

#define BITMODE_RESET		0
#define BITMODE_MPSSE		2

#define CMD_SIZE		3
#define MAX_SETUP_COMMANDS	10
#define SS_TX_COUNT		3

#define LOW			0
#define HIGH			1
#define NUM_GPIOL_PINS		4
#define NUM_GPIO_PINS		12

#define NULL_CONTEXT_ERROR_MSG	"NULL MPSSE context pointer!"

/* FTDI interfaces */
enum interface
{
	IFACE_ANY	= INTERFACE_ANY,
	IFACE_A 	= INTERFACE_A,
	IFACE_B		= INTERFACE_B,
	IFACE_C		= INTERFACE_C,
	IFACE_D		= INTERFACE_D
};

/* Common clock rates */
enum clock_rates
{
	ONE_HUNDRED_KHZ  = 100000,
	FOUR_HUNDRED_KHZ = 400000,
	ONE_MHZ 	 = 1000000,
	TWO_MHZ		 = 2000000,
	FIVE_MHZ	 = 5000000,
	SIX_MHZ 	 = 6000000,
	TEN_MHZ		 = 10000000,
	TWELVE_MHZ 	 = 12000000,
	THIRTY_MHZ 	 = 30000000,
	SIXTY_MHZ 	 = 60000000
};

/* Supported MPSSE modes */
enum modes
{
	SPI0    = 1,
	SPI1    = 2,
	SPI2    = 3,
	SPI3    = 4,
	I2C     = 5,
	GPIO    = 6,
	BITBANG = 7,
	MCU8	= 8,
	MCU16	= 9
};

enum pins
{
	SK	= 1,
	DO	= 2,
	DI	= 4,
	CS	= 8 ,
	GPIO0	= 16,
	GPIO1	= 32,
	GPIO2	= 64,
	GPIO3	= 128
};

enum gpio_pins
{
	GPIOL0 = 0,
	GPIOL1 = 1,
	GPIOL2 = 2,
	GPIOL3 = 3,
	GPIOH0 = 4,
	GPIOH1 = 5,
	GPIOH2 = 6,
	GPIOH3 = 7,
	GPIOH4 = 8,
	GPIOH5 = 9,
	GPIOH6 = 10,
	GPIOH7 = 11
};

enum i2c_ack
{
	ACK  = 0,
	NACK = 1
};

#define DEFAULT_TRIS            (SK | DO | CS | GPIO0 | GPIO1 | GPIO2 | GPIO3)  /* SK/DO/CS and GPIOs are outputs, DI is an input */
#define DEFAULT_PORT            (SK | CS)       				/* SK and CS are high, all others low */

enum mpsse_commands
{
	INVALID_COMMAND		= 0xAB,
	ENABLE_3_PHASE_CLOCK	= 0x8C,
	DISABLE_3_PHASE_CLOCK	= 0x8D,
	DISABLE_ADAPTIVE_CLOCK	= 0x97,
	TCK_X5			= 0x8A,
	TCK_D5			= 0x8B,
	CLOCK_N_CYCLES		= 0x8E,
	CLOCK_N8_CYCLES		= 0x8F,
	PULSE_CLOCK_IO_HIGH	= 0x94,
	PULSE_CLOCK_IO_LOW	= 0x95,
	CLOCK_N8_CYCLES_IO_HIGH	= 0x9C,
	CLOCK_N8_CYCLES_IO_LOW	= 0x9D,
	TRISTATE_IO		= 0x9E,
	CPU_SEND_IMMEDIATE	= 0x87,
	CPU_READ_SHORT		= 0x90,
	CPU_READ_LONG		= 0x91,
	CPU_WRITE_SHORT		= 0x92,
	CPU_WRITE_LONG		= 0x93
};

enum low_bits_status
{
	STARTED,
	STOPPED
};

struct vid_pid
{
	int vid;
	int pid;
	char *description;
};

struct mpsse_context
{
	char *description;
	struct ftdi_context ftdi;
	enum modes mode;
	enum low_bits_status status;
	int vid;
	int pid;
	int clock;
	int xsize;
	int open;
	uint8_t tris;
	uint8_t pstart;
	uint8_t pstop;
	uint8_t pidle;
	uint8_t gpioh;
	uint8_t trish;
	uint8_t bitbang;
	uint8_t tx;
	uint8_t rx;
	uint8_t txrx;
	uint8_t tack;
	uint8_t rack;
};

struct mpsse_mcu_t
{
	struct mpsse_context *(*Open)(enum modes mode);
	int (*Write)(struct mpsse_context *mpsse, char *data, int size, int address);
	MPSSE_MCU_READ_FUNCTION_PTR
	void (*Close)(struct mpsse_context *mpsse);
};

struct mpsse_spi_t
{
	struct mpsse_context *(*Open)(enum modes mode, int freq, int endianess);
	int (*Start)(struct mpsse_context *mpsse);
	int (*Stop)(struct mpsse_context *mpsse);
	void (*Close)(struct mpsse_context *mpsse);
	void (*SetCSIdle)(struct mpsse_context *mpsse, int idle);
	int (*Write)(struct mpsse_context *mpsse, char *data, int size);
	MPSSE_READ_FUNCTION_PTR
	MPSSE_TRANSFER_FUNCTION_PTR
};

struct mpsse_i2c_t
{
	struct mpsse_context *(*Open)(int freq);
	int (*Start)(struct mpsse_context *mpsse);
	int (*Stop)(struct mpsse_context *mpsse);
	int (*Write)(struct mpsse_context *mpsse, char *data, int size);
	MPSSE_READ_FUNCTION_PTR
	void (*Close)(struct mpsse_context *mpsse);
	int (*GetAck)(struct mpsse_context *mpsse);
	void (*SetAck)(struct mpsse_context *mpsse, int ack);
	void (*SendAcks)(struct mpsse_context *mpsse);
	void (*SendNacks)(struct mpsse_context *mpsse);
	int (*Tristate)(struct mpsse_context *mpsse);
};

struct mpsse_gpio_t
{
	struct mpsse_context *(*Open)(void);
	void (*Close)(struct mpsse_context *mpsse);
	int (*PinHigh)(struct mpsse_context *mpsse, int pin);
	int (*PinLow)(struct mpsse_context *mpsse, int pin);
	int (*ReadPins)(struct mpsse_context *mpsse);
	int (*PinState)(struct mpsse_context *mpsse, int pin, int state);
	int (*ClockUntilHigh)(struct mpsse_context *mpsse);
	int (*ClockUntilLow)(struct mpsse_context *mpsse);
	int (*ToggleClock)(struct mpsse_context *mpsse, int count);
	int (*ToggleClockX8)(struct mpsse_context *mpsse, int count, int gpio);
};

struct mpsse_t
{
	struct mpsse_spi_t SPI;
	struct mpsse_i2c_t I2C;
	struct mpsse_mcu_t MCU;
	struct mpsse_gpio_t GPIO;
	struct mpsse_gpio_t BITBANG;
	struct mpsse_context *(*Open)(int vid, int pid, enum modes mode, int freq, int endianess, int interface, const char *description, const char *serial);
	void (*Close)(struct mpsse_context *mpsse);
	char *(*ErrorString)(struct mpsse_context *mpsse);
	int (*SetMode)(struct mpsse_context *mpsse, int endianess);
	int (*SetClock)(struct mpsse_context *mpsse, uint32_t freq);
	int (*GetClock)(struct mpsse_context *mpsse);
	int (*GetVid)(struct mpsse_context *mpsse);
	int (*GetPid)(struct mpsse_context *mpsse);
	char *(*GetDescription)(struct mpsse_context *mpsse);
	int (*SetLoopback)(struct mpsse_context *mpsse, int enable);
	int (*Version)(void);
} MPSSE;

struct mpsse_context *mpsse_mcu_open(enum modes mode);
struct mpsse_context *mpsse_i2c_open(int freq);
struct mpsse_context *mpsse_gpio_open(void);
struct mpsse_context *mpsse_bitbang_open(void);
struct mpsse_context *mpsse_easy_open(enum modes mode, int freq, int endianess);
struct mpsse_context *mpsse_open(int vid, int pid, enum modes mode, int freq, int endianess, int interface, const char *description, const char *serial);
void mpsse_close(struct mpsse_context *mpsse);
char *mpsse_error_string(struct mpsse_context *mpsse);
int mpsse_set_mode(struct mpsse_context *mpsse, int endianess);
int mpsse_set_clock(struct mpsse_context *mpsse, uint32_t freq);
int mpsse_get_clock(struct mpsse_context *mpsse);
int mpsse_get_vid(struct mpsse_context *mpsse);
int mpsse_get_pid(struct mpsse_context *mpsse);
char *mpsse_get_description(struct mpsse_context *mpsse);
int mpsse_set_loopback(struct mpsse_context *mpsse, int enable);
void mpsse_set_cs_idle(struct mpsse_context *mpsse, int idle);
int mpsse_start(struct mpsse_context *mpsse);
int mpsse_write(struct mpsse_context *mpsse, char *data, int size);
int mpsse_mcu_write(struct mpsse_context *mpsse, char *data, int size, int address);
int mpsse_stop(struct mpsse_context *mpsse);
int mpsse_get_ack(struct mpsse_context *mpsse);
void mpsse_set_ack(struct mpsse_context *mpsse, int ack);
void mpsse_send_acks(struct mpsse_context *mpsse);
void mpsse_send_nacks(struct mpsse_context *mpsse);
int mpsse_pin_high(struct mpsse_context *mpsse, int pin);
int mpsse_pin_low(struct mpsse_context *mpsse, int pin);
int mpsse_read_pins(struct mpsse_context *mpsse);
int mpsse_pin_state(struct mpsse_context *mpsse, int pin, int state);
int mpsse_clock_until_high(struct mpsse_context *mpsse);
int mpsse_clock_until_low(struct mpsse_context *mpsse);
int mpsse_toggle_clock(struct mpsse_context *mpsse, int count);
int mpsse_toggle_clock_x8(struct mpsse_context *mpsse, int count, int gpio);
int mpsse_tristate(struct mpsse_context *mpsse);
int mpsse_version(void);

#ifdef SWIGPYTHON
swig_string_data mpsse_read(struct mpsse_context *mpsse, int size);
swig_string_data mpsse_mcu_read(struct mpsse_context *mpsse, int size, int address);
swig_string_data mpsse_transfer(struct mpsse_context *mpsse, char *data, int size);
#else
char *mpsse_read(struct mpsse_context *mpsse, int size);
char *mpsse_mcu_read(struct mpsse_context *mpsse, int size, int address);
char *mpsse_transfer(struct mpsse_context *mpsse, char *data, int size);
#endif

#endif
