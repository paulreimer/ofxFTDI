/*
 * Main libmpsse source file.
 *
 * Craig Heffner
 * 27 December 2011
 */

#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <ftdi.h>
#include "mpsse.h"
#include "support.h"
#include "config.h"

/* List of known FT2232-based devices */
struct vid_pid supported_devices[] = { 
			{ 0x0403, 0x6010, "FT2232 Future Technology Devices International, Ltd" }, 
			{ 0x0403, 0x6011, "FT4232 Future Technology Devices International, Ltd" }, 
			{ 0x0403, 0x6014, "FT232H Future Technology Devices International, Ltd" },

			/* These devices are based on FT2232 chips, but have not been tested. */
			{ 0x0403, 0x8878, "Bus Blaster v2 (channel A)" },
			{ 0x0403, 0x8879, "Bus Blaster v2 (channel B)" },
			{ 0x0403, 0xBDC8, "Turtelizer JTAG/RS232 Adapter A" },
			{ 0x0403, 0xCFF8, "Amontec JTAGkey" },
			{ 0x15BA, 0x0003, "Olimex Ltd. OpenOCD JTAG" },
			{ 0x15BA, 0x0004, "Olimex Ltd. OpenOCD JTAG TINY" },
			
			{ 0, 0, NULL }
};

/* Called on library load */
static void _mpsse_ldInit(void) __attribute__((__constructor__));
static void _mpsse_ldInit(void)
{
	MPSSE.Open			= &mpsse_open;
	MPSSE.Close			= &mpsse_close;
	MPSSE.Version			= &mpsse_version;
	MPSSE.ErrorString		= &mpsse_error_string;
	MPSSE.SetMode			= &mpsse_set_mode;
	MPSSE.SetClock			= &mpsse_set_clock;
	MPSSE.GetClock			= &mpsse_get_clock;
	MPSSE.GetVid			= &mpsse_get_vid;
	MPSSE.GetPid			= &mpsse_get_pid;
	MPSSE.GetDescription		= &mpsse_get_description;
	MPSSE.SetLoopback		= &mpsse_set_loopback;
	
	MPSSE.GPIO.Open			= &mpsse_gpio_open;
	MPSSE.GPIO.PinHigh		= &mpsse_pin_high;
	MPSSE.GPIO.PinLow		= &mpsse_pin_low;
	MPSSE.GPIO.ReadPins		= &mpsse_read_pins;
	MPSSE.GPIO.PinState		= &mpsse_pin_state;
	MPSSE.GPIO.ClockUntilHigh	= &mpsse_clock_until_high;
	MPSSE.GPIO.ClockUntilLow	= &mpsse_clock_until_low;
	MPSSE.GPIO.ToggleClock		= &mpsse_toggle_clock;
	MPSSE.GPIO.ToggleClockX8	= &mpsse_toggle_clock_x8;
	MPSSE.GPIO.Close		= &mpsse_close;

	MPSSE.BITBANG.Open		= &mpsse_bitbang_open;
        MPSSE.BITBANG.PinHigh		= &mpsse_pin_high;
        MPSSE.BITBANG.PinLow		= &mpsse_pin_low;
        MPSSE.BITBANG.ReadPins		= &mpsse_read_pins;
        MPSSE.BITBANG.PinState		= &mpsse_pin_state;
        MPSSE.BITBANG.ClockUntilHigh	= &mpsse_clock_until_high;
        MPSSE.BITBANG.ClockUntilLow	= &mpsse_clock_until_low;
        MPSSE.BITBANG.ToggleClock	= &mpsse_toggle_clock;
        MPSSE.BITBANG.ToggleClockX8	= &mpsse_toggle_clock_x8;
	MPSSE.BITBANG.Close		= &mpsse_close;

	MPSSE.SPI.Open			= &mpsse_easy_open;
	MPSSE.SPI.Read			= &mpsse_read;
	MPSSE.SPI.Write			= &mpsse_write;
	MPSSE.SPI.Transfer		= &mpsse_transfer;
	MPSSE.SPI.SetCSIdle		= &mpsse_set_cs_idle;
	MPSSE.SPI.Start			= &mpsse_start;
	MPSSE.SPI.Stop			= &mpsse_stop;
	MPSSE.SPI.Close			= &mpsse_close;

	MPSSE.I2C.Open			= &mpsse_i2c_open;
	MPSSE.I2C.Read			= &mpsse_read;
	MPSSE.I2C.Write			= &mpsse_write;
	MPSSE.I2C.Start			= &mpsse_start;
	MPSSE.I2C.Stop			= &mpsse_stop;
	MPSSE.I2C.Tristate		= &mpsse_tristate;
	MPSSE.I2C.GetAck		= &mpsse_get_ack;
	MPSSE.I2C.SetAck		= &mpsse_set_ack;
	MPSSE.I2C.SendAcks		= &mpsse_send_acks;
	MPSSE.I2C.SendNacks		= &mpsse_send_nacks;
	MPSSE.I2C.Close			= &mpsse_close;
	
	MPSSE.MCU.Open			= &mpsse_mcu_open;
	MPSSE.MCU.Read			= &mpsse_mcu_read;
	MPSSE.MCU.Write			= &mpsse_mcu_write;
	MPSSE.MCU.Close			= &mpsse_close;
};

/*
 * Opens and initializes the first FTDI device in MCU mode.
 *
 * @mode - The MCU mode to use. One of: MCU8, MCU16
 *
 * Returns a pointer to an MPSSE context structure.
 * On success, mpsse->open will be set to 1.
 * On failure, mpsse->open will be set to 0.
 */
struct mpsse_context *mpsse_mcu_open(enum modes mode)
{
	return mpsse_easy_open(mode, 0, MSB);
}

/*
 * Opens and initializes the first FTDI device in I2C mode.
 *
 * @freq - The I2C frequency to use.
 *
 * Returns a pointer to an MPSSE context structure.
 * On success, mpsse->open will be set to 1.
 * On failure, mpsse->open will be set to 0.
 */
struct mpsse_context *mpsse_i2c_open(int freq)
{
	return mpsse_easy_open(I2C, freq, MSB);
}

/*
 * Opens and initializes the first FTDI device in GPIO mode.
 *
 * Returns a pointer to an MPSSE context structure.
 * On success, mpsse->open will be set to 1.
 * On failure, mpsse->open will be set to 0.
 */
struct mpsse_context *mpsse_gpio_open(void)
{
	return mpsse_easy_open(GPIO, 0, 0);
}

/*
 * Opens and initializes the first FTDI device in Bitbang mode.
 *
 * Returns a pointer to an MPSSE context structure.
 * On success, mpsse->open will be set to 1.
 * On failure, mpsse->open will be set to 0.
 */
struct mpsse_context *mpsse_bitbang_open(void)
{
	return mpsse_easy_open(BITBANG, 0, 0);
}

/*
 * Opens and initializes the first FTDI device found.
 * 
 * @mode      - Mode to open the device in. One of enum modes.
 * @freq      - Clock frequency to use for the specified mode.
 * @endianess - Specifies how data is clocked in/out (MSB, LSB).
 *
 * Returns a pointer to an MPSSE context structure. 
 * On success, mpsse->open will be set to 1.
 * On failure, mpsse->open will be set to 0.
 */
struct mpsse_context *mpsse_easy_open(enum modes mode, int freq, int endianess)
{
	int i = 0;
	struct mpsse_context *mpsse = NULL;

	for(i=0; supported_devices[i].vid != 0; i++)
	{
		if((mpsse = mpsse_open(supported_devices[i].vid, supported_devices[i].pid, mode, freq, endianess, IFACE_A, NULL, NULL)) != NULL)
		{
			if(mpsse->open)
			{
				mpsse->description = supported_devices[i].description;
				break;
			}
			/* If there is another device still left to try, free the context pointer and try again */
			else if(supported_devices[i+1].vid != 0)
			{
				mpsse_close(mpsse);
				mpsse = NULL;
			}
		}
	}

	return mpsse;
}

/* 
 * Open device by VID/PID 
 *
 * @vid         - Device vendor ID.
 * @pid         - Device product ID.
 * @mode        - MPSSE mode, one of enum modes.
 * @freq        - Clock frequency to use for the specified mode.
 * @endianess   - Specifies how data is clocked in/out (MSB, LSB).
 * @interface   - FTDI interface to use (IFACE_A - IFACE_D).
 * @description - Device product description (set to NULL if not needed).
 * @serial      - Device serial number (set to NULL if not needed).
 *
 * Returns a pointer to an MPSSE context structure. 
 * On success, mpsse->open will be set to 1.
 * On failure, mpsse->open will be set to 0.
 */
struct mpsse_context *mpsse_open(int vid, int pid, enum modes mode, int freq, int endianess, int interface, const char *description, const char *serial)
{
	int status = 0;
	struct mpsse_context *mpsse = NULL;

	mpsse = malloc(sizeof(struct mpsse_context));
	if(mpsse)
	{
		memset(mpsse, 0, sizeof(struct mpsse_context));

		/* ftdilib initialization */
		if(ftdi_init(&mpsse->ftdi) == 0)
		{
			/* Set the FTDI interface  */
			ftdi_set_interface(&mpsse->ftdi, interface);

			/* Open the specified device */
			if(ftdi_usb_open_desc(&mpsse->ftdi, vid, pid, description, serial) == 0)
			{
				mpsse->mode = mode;
				mpsse->vid = vid;
				mpsse->pid = pid;
				mpsse->status = STOPPED;

				/* Set the appropriate transfer size for the requested protocol */
				if(mpsse->mode == I2C)
				{
					mpsse->xsize = I2C_TRANSFER_SIZE;
				}
				else
				{
					mpsse->xsize = SPI_RW_SIZE;
				}
	
				status |= ftdi_usb_reset(&mpsse->ftdi);
				status |= ftdi_set_latency_timer(&mpsse->ftdi, LATENCY_MS);
				status |= ftdi_write_data_set_chunksize(&mpsse->ftdi, CHUNK_SIZE);
				status |= ftdi_read_data_set_chunksize(&mpsse->ftdi, CHUNK_SIZE);
				status |= ftdi_set_bitmode(&mpsse->ftdi, 0, BITMODE_RESET);

				if(status == 0)
				{
					/* Set the read and write timeout periods */
					set_timeouts(mpsse, USB_TIMEOUT);
					
					if(mpsse->mode != BITBANG)
					{
						ftdi_set_bitmode(&mpsse->ftdi, 0, BITMODE_MPSSE);

						if(mpsse_set_clock(mpsse, freq) == MPSSE_OK)
						{
							if(mpsse_set_mode(mpsse, endianess) == MPSSE_OK)
							{
								mpsse->open = 1;

								/* Give the chip a few mS to initialize */
								usleep(SETUP_DELAY);

								/* 
								 * Not all FTDI chips support all the commands that SetMode may have sent.
								 * This clears out any errors from unsupported commands that might have been sent during set up. 
								 */
								ftdi_usb_purge_buffers(&mpsse->ftdi);
							}
						}
					}
					else
					{
						/* Skip the setup functions if we're just operating in BITBANG mode */
						if(ftdi_set_bitmode(&mpsse->ftdi, 0xFF, BITMODE_BITBANG) == 0)
						{
							mpsse->open = 1;
						}
					}
				}
			}
		}
	}

	return mpsse;
}

/* 
 * Closes the device, deinitializes libftdi, and frees the MPSSE context pointer.
 *
 * @mpsse - MPSSE context pointer.
 *
 * Returns void.
 */
void mpsse_close(struct mpsse_context *mpsse)
{
	if(mpsse)
	{
		if(mpsse->open)
		{
			ftdi_set_bitmode(&mpsse->ftdi, 0, BITMODE_RESET);
			ftdi_usb_close(&mpsse->ftdi);
		}

		ftdi_deinit(&mpsse->ftdi);
		free(mpsse);
		mpsse = NULL;
	}

	return;
}

/*
 * Sets the appropriate transmit and receive commands based on the requested mode and byte order.
 *
 * @mpsse     - MPSSE context pointer.
 * @endianess - MPSSE_MSB or MPSSE_LSB.
 *
 * Returns MPSSE_OK on success.
 * Returns MPSSE_FAIL on failure.
 */
int mpsse_set_mode(struct mpsse_context *mpsse, int endianess)
{
	int retval = MPSSE_OK, i = 0, setup_commands_size = 0;
	char buf[CMD_SIZE] = { 0 };
	char setup_commands[CMD_SIZE*MAX_SETUP_COMMANDS] = { 0 };

	/* Do not call is_valid_context() here, as the FTDI chip may not be completely configured when SetMode is called */
	if(mpsse)
	{
		/* Read and write commands need to include endianess */
		mpsse->tx   = MPSSE_DO_WRITE | endianess;
		mpsse->rx   = MPSSE_DO_READ  | endianess;
		mpsse->txrx = MPSSE_DO_WRITE | MPSSE_DO_READ | endianess;

		/* Clock, data out, chip select pins are outputs; all others are inputs. */
		mpsse->tris = DEFAULT_TRIS;

		/* Clock and chip select pins idle high; all others are low */
		mpsse->pidle = mpsse->pstart = mpsse->pstop = DEFAULT_PORT;

		/* During reads and writes the chip select pin is brought low */
		mpsse->pstart &= ~CS;

		/* Disable FTDI internal loopback */
	        mpsse_set_loopback(mpsse, 0);

		/* Send ACKs by default */
		mpsse_set_ack(mpsse, ACK);

		/* Ensure adaptive clock is disabled */
		setup_commands[setup_commands_size++] = DISABLE_ADAPTIVE_CLOCK;

		switch(mpsse->mode)
		{
			case SPI0:
				/* SPI mode 0 clock idles low */
				mpsse->pidle &= ~SK;
				mpsse->pstart &= ~SK;
				mpsse->pstop &= ~SK;
				/* SPI mode 0 propogates data on the falling edge and read data on the rising edge of the clock */
				mpsse->tx |= MPSSE_WRITE_NEG;
				mpsse->rx &= ~MPSSE_READ_NEG;
				mpsse->txrx |= MPSSE_WRITE_NEG;
				mpsse->txrx &= ~MPSSE_READ_NEG;
				break;
			case SPI3:
				/* SPI mode 3 clock idles high */
				mpsse->pidle |= SK;
				mpsse->pstart |= SK;
				/* Keep the clock low while the CS pin is brought high to ensure we don't accidentally clock out an extra bit */
				mpsse->pstop &= ~SK;
				/* SPI mode 3 propogates data on the falling edge and read data on the rising edge of the clock */
				mpsse->tx |= MPSSE_WRITE_NEG;
				mpsse->rx &= ~MPSSE_READ_NEG;
				mpsse->txrx |= MPSSE_WRITE_NEG;
				mpsse->txrx &= ~MPSSE_READ_NEG;
				break;
			case SPI1:
				/* SPI mode 1 clock idles low */
				mpsse->pidle &= ~SK;
				/* Since this mode idles low, the start condition should ensure that the clock is low */
				mpsse->pstart &= ~SK;
				/* Even though we idle low in this mode, we need to keep the clock line high when we set the CS pin high to prevent
				 * an unintended clock cycle from being sent by the FT2232. This way, the clock goes high, but does not go low until
				 * after the CS pin goes high.
				 */
				mpsse->pstop |= SK;
				/* Data read on falling clock edge */
				mpsse->rx |= MPSSE_READ_NEG;
				mpsse->tx &= ~MPSSE_WRITE_NEG;
				mpsse->txrx |= MPSSE_READ_NEG;
				mpsse->txrx &= ~MPSSE_WRITE_NEG;
				break;
			case SPI2:
				/* SPI 2 clock idles high */
				mpsse->pidle |= SK;
				mpsse->pstart |= SK;
				mpsse->pstop |= SK;
				/* Data read on falling clock edge */
				mpsse->rx |= MPSSE_READ_NEG;
				mpsse->tx &= ~MPSSE_WRITE_NEG;
				mpsse->txrx |= MPSSE_READ_NEG;
				mpsse->txrx &= ~MPSSE_WRITE_NEG;
				break;
			case I2C:
				/* I2C propogates data on the falling clock edge and reads data on the falling (or rising) clock edge */
				mpsse->tx |= MPSSE_WRITE_NEG;
				mpsse->rx &= ~MPSSE_READ_NEG;
				/* In I2C, both the clock and the data lines idle high */
				mpsse->pidle |= DO | DI;
				/* I2C start bit == data line goes from high to low while clock line is high */
				mpsse->pstart &= ~DO & ~DI;
				/* I2C stop bit == data line goes from low to high while clock line is high - set data line low here, so the transition to the idle state triggers the stop condition. */
				mpsse->pstop &= ~DO & ~DI;
				/* Enable three phase clock to ensure that I2C data is available on both the rising and falling clock edges */
				setup_commands[setup_commands_size++] = ENABLE_3_PHASE_CLOCK;
				break;
			case MCU8:
				mpsse->rx = CPU_READ_SHORT;
				mpsse->tx = CPU_WRITE_LONG;
				break;
			case MCU16:
				mpsse->rx = CPU_READ_LONG;
				mpsse->tx = CPU_WRITE_LONG;
				break;
			case GPIO:
				break;
			default:
				retval = MPSSE_FAIL;
		}

		/* Send any setup commands to the chip */
		if(retval == MPSSE_OK && setup_commands_size > 0)
		{
			retval = raw_write(mpsse, (unsigned char *) &setup_commands, setup_commands_size);
		}
	
		if(retval == MPSSE_OK)
		{
			/* Set the idle pin states */
			set_bits_low(mpsse, mpsse->pidle);
	
			/* All GPIO pins are inputs, pulled low */
			mpsse->trish = 0xFF;
			mpsse->gpioh = 0x00;
	
	                buf[i++] = SET_BITS_HIGH;
	                buf[i++] = mpsse->gpioh;
	                buf[i++] = mpsse->trish;
	
			retval = raw_write(mpsse, (unsigned char *) &buf, i);
		}
	}
	else
	{
		retval = MPSSE_FAIL;
	}	

	return retval;
}

/* 
 * Sets the appropriate divisor for the desired clock frequency.
 *
 * @mpsse - MPSSE context pointer.
 * @freq  - Desired clock frequency in hertz.
 *
 * Returns MPSSE_OK on success.
 * Returns MPSSE_FAIL on failure.
 */
int mpsse_set_clock(struct mpsse_context *mpsse, uint32_t freq)
{
	int retval = MPSSE_FAIL;
	uint32_t system_clock = 0;
	uint16_t divisor = 0;
	char buf[CMD_SIZE] = { 0 };

	/* Do not call is_valid_context() here, as the FTDI chip may not be completely configured when SetClock is called */
	if(mpsse)
	{
		if(freq > SIX_MHZ)
		{
			buf[0] = TCK_X5;
			system_clock = SIXTY_MHZ;
		}
		else
		{
			buf[0] = TCK_D5;
			system_clock = TWELVE_MHZ;
		}
		
		if(raw_write(mpsse, (unsigned char *) &buf, 1) == MPSSE_OK)
		{
			if(freq <= 0)
			{
				divisor = 0xFFFF;
			}
			else
			{
				divisor = freq2div(system_clock, freq);
			}
	
			buf[0] = TCK_DIVISOR;
			buf[1] = (divisor & 0xFF);
			buf[2] = ((divisor >> 8) & 0xFF);
	
			if(raw_write(mpsse, (unsigned char *) &buf, 3) == MPSSE_OK)
			{
				mpsse->clock = div2freq(system_clock, divisor);
				retval = MPSSE_OK;
			}
		}
	}
	
	return retval;
}

/* 
 * Retrieves the last error string from libftdi.
 *
 * @mpsse - MPSSE context pointer.
 *
 * Returns a pointer to the last error string.
 */
char *mpsse_error_string(struct mpsse_context *mpsse)
{
	if(mpsse != NULL)
	{
		return ftdi_get_error_string(&mpsse->ftdi);
	}

	return NULL_CONTEXT_ERROR_MSG;
}

/* 
 * Gets the currently configured clock rate.
 *
 * @mpsse - MPSSE context pointer.
 *
 * Returns the existing clock rate in hertz.
 */
int mpsse_get_clock(struct mpsse_context *mpsse)
{
	int clock = 0;

	if(is_valid_context(mpsse))
	{
		clock = mpsse->clock;
	}

	return clock;
}

/*
 * Returns the vendor ID of the FTDI chip.
 * 
 * @mpsse - MPSSE context pointer.
 *
 * Returns the integer value of the vendor ID.
 */
int mpsse_get_vid(struct mpsse_context *mpsse)
{
	int vid = 0;

	if(is_valid_context(mpsse))
	{
		vid = mpsse->vid;
	}

	return vid;
}

/*
 * Returns the product ID of the FTDI chip.
 *
 * @mpsse - MPSSE context pointer.
 *
 * Returns the integer value of the product ID.
 */
int mpsse_get_pid(struct mpsse_context *mpsse)
{
	int pid = 0;

	if(is_valid_context(mpsse) )
	{
		pid = mpsse->pid;
	}

	return pid;
}

/*
 * Returns the description of the FTDI chip, if any.
 *
 * @mpsse - MPSSE context pointer.
 *
 * Returns the description of the FTDI chip.
 */
char *mpsse_get_description(struct mpsse_context *mpsse)
{
	char *description = NULL;
	
	if(is_valid_context(mpsse))
	{
		description = mpsse->description;
	}

	return description;
}

/* 
 * Enable / disable internal loopback.
 * 
 * @mpsse  - MPSSE context pointer.
 * @enable - Zero to disable loopback, 1 to enable loopback.
 *
 * Returns MPSSE_OK on success.
 * Returns MPSSE_FAIL on failure.
 */
int mpsse_set_loopback(struct mpsse_context *mpsse, int enable)
{
	char buf[1] = { 0 };
	int retval = MPSSE_FAIL;

	if(is_valid_context(mpsse))
	{
		if(enable)
		{
			buf[0] = LOOPBACK_START;
		}
		else
		{
			buf[0] = LOOPBACK_END;
		}

		retval = raw_write(mpsse, (unsigned char *) &buf, 1);
	}

	return retval;
}

/*
 * Sets the idle state of the chip select pin. CS idles high by default.
 *
 * @mpsse - MPSSE context pointer.
 * @idle  - Set to 1 to idle high, 0 to idle low.
 *
 * Returns void.
 */
void mpsse_set_cs_idle(struct mpsse_context *mpsse, int idle)
{
	if(is_valid_context(mpsse))
	{
		if(idle > 0)
		{
			/* Chip select idles high, active low */
			mpsse->pidle |= CS;
			mpsse->pstop |= CS;
			mpsse->pstart &= ~CS;
		}
		else
		{
			/* Chip select idles low, active high */
			mpsse->pidle &= ~CS;
			mpsse->pstop &= ~CS;
			mpsse->pstart |= CS;
		}
	}

	return;
}

/*
 * Send data start condition.
 *
 * @mpsse - MPSSE context pointer.
 *
 * Returns MPSSE_OK on success.
 * Returns MPSSE_FAIL on failure.
 */
int mpsse_start(struct mpsse_context *mpsse)
{
	int status = MPSSE_OK;

	if(is_valid_context(mpsse))
	{

		if(mpsse->mode == I2C && mpsse->status == STARTED)
		{
			/* Set the default pin states while the clock is low since this is an I2C repeated start condition */
			status |= set_bits_low(mpsse, (mpsse->pidle & ~SK));
	
			/* Make sure the pins are in their default idle state */
			status |= set_bits_low(mpsse, mpsse->pidle);
		}

		/* Set the start condition */
		status |= set_bits_low(mpsse, mpsse->pstart);

		/* 
		 * Hackish work around to properly support SPI mode 3.
		 * SPI3 clock idles high, but needs to be set low before sending out 
		 * data to prevent unintenteded clock glitches from the FT2232.
		 */
		if(mpsse->mode == SPI3)
	        {
			status |= set_bits_low(mpsse, (mpsse->pstart & ~SK));
	        }
		/*
		 * Hackish work around to properly support SPI mode 1.
		 * SPI1 clock idles low, but needs to be set high before sending out
		 * data to preven unintended clock glitches from the FT2232.
		 */
		else if(mpsse->mode == SPI1)
		{
			status |= set_bits_low(mpsse, (mpsse->pstart | SK));
		}
		
		mpsse->status = STARTED;
	}
	else
	{
		status = MPSSE_FAIL;
		mpsse->status = STOPPED;
	}

	return status;
}

/*
 * Send data out via the selected serial protocol.
 *
 * @mpsse - MPSSE context pointer.
 * @data  - Buffer of data to send.
 * @size  - Size of data.
 *
 * Returns MPSSE_OK on success.
 * Returns MPSSE_FAIL on failure.
 */
int mpsse_write(struct mpsse_context *mpsse, char *data, int size)
{
	unsigned char *buf = NULL;
	int retval = MPSSE_FAIL, buf_size = 0, txsize = 0, n = 0;

	if(is_valid_context(mpsse))
	{
		if(mpsse->mode)
		{
			while(n < size)
			{
				txsize = size - n;
				if(txsize > mpsse->xsize)
				{
					txsize = mpsse->xsize;
				}
	
				/* 
				 * For I2C we need to send each byte individually so that we can 
				 * read back each individual ACK bit, so set the transmit size to 1.
				 */
				if(mpsse->mode == I2C)
				{
					txsize = 1;
				}
	
				buf = build_block_buffer(mpsse, mpsse->tx, (unsigned char *) (data + n), txsize, &buf_size);
				if(buf)
				{	
					retval = raw_write(mpsse, buf, buf_size);
					n += txsize;
					free(buf);
	
					if(retval == MPSSE_FAIL)
					{
						break;
					}
				
					/* Read in the ACK bit and store it in mpsse->rack */
					if(mpsse->mode == I2C)
					{
						raw_read(mpsse, (unsigned char *) &mpsse->rack, 1);
					}
				}
				else
				{
					break;
				}
			}
		}
	
		if(retval == MPSSE_OK && n == size)
		{
			retval = MPSSE_OK;
		}
	}
		
	return retval;
}

/*
 * Reads data over the selected serial protocol.
 * 
 * @mpsse - MPSSE context pointer.
 * @size  - Number of bytes to read.
 *
 * Returns a pointer to the read data on success.
 * Returns NULL on failure.
 */
#ifdef SWIGPYTHON
swig_string_data mpsse_read(struct mpsse_context *mpsse, int size)
#else
char *mpsse_read(struct mpsse_context *mpsse, int size)
#endif
{
	unsigned char *data = NULL, *buf = NULL;
	char sbuf[SPI_RW_SIZE] = { 0 };
	int n = 0, rxsize = 0, data_size = 0;

	if(is_valid_context(mpsse))
	{
		if(mpsse->mode)
		{
			buf = malloc(size);
			if(buf)
			{
				memset(buf, 0, size);
	
				while(n < size)
				{
					rxsize = size - n;
					if(rxsize > mpsse->xsize)
					{
						rxsize = mpsse->xsize;
					}
	
					data = build_block_buffer(mpsse, mpsse->rx, (unsigned char *) &sbuf, rxsize, &data_size);
					if(data)
					{
						if(raw_write(mpsse, data, data_size) == MPSSE_OK)
						{
							n += raw_read(mpsse, buf+n, rxsize);
						}
						else
						{
							break;
						}
						
						free(data);
					}
					else
					{
						break;
					}
				}
			}
		}
	}
	
#ifdef SWIGPYTHON
	swig_string_data sdata = { 0 };
	sdata.size = n;
	sdata.data = (char *) buf;
	return sdata;
#else
	return (char *) buf;
#endif
}

/*
 * Reads and writes data over the selected serial protocol (SPI only).
 * 
 * @mpsse - MPSSE context pointer.
 * @data  - Buffer containing bytes to write.
 * @size  - Number of bytes to transfer.
 *
 * Returns a pointer to the read data on success.
 * Returns NULL on failure.
 */
#ifdef SWIGPYTHON
swig_string_data mpsse_transfer(struct mpsse_context *mpsse, char *data, int size)
#else
char *mpsse_transfer(struct mpsse_context *mpsse, char *data, int size)
#endif
{
	unsigned char *txdata = NULL, *buf = NULL;
	int n = 0, data_size = 0, rxsize = 0;

	if(is_valid_context(mpsse))
	{
		/* Make sure we're configured for one of the SPI modes */
		if(mpsse->mode >= SPI0 && mpsse->mode <= SPI3)
		{
			buf = malloc(size);
			if(buf)
			{
				memset(buf, 0, size);

				/* When sending and recieving, FTDI chips don't seem to like large data blocks. Limit the size of each block to SPI_TRANSFER_SIZE */
				rxsize = size - n;
				if(rxsize > SPI_TRANSFER_SIZE)
				{
					rxsize = SPI_TRANSFER_SIZE;
				}

				while(n < size)
				{
					txdata = build_block_buffer(mpsse, mpsse->txrx, (unsigned char *) (data + n), rxsize, &data_size);
					if(txdata)
					{
						if(raw_write(mpsse, txdata, data_size) == MPSSE_OK)
						{
							n += raw_read(mpsse, (buf + n), rxsize);
						}
						else
						{
							break;
						}

						free(txdata);
					}
					else
					{
						break;
					}
				}
			}
		}
	}

#ifdef SWIGPYTHON
	swig_string_data sdata = { 0 };
	sdata.size = n;
	sdata.data = (char *) buf;
	return sdata;
#else
	return (char *) buf;
#endif
}

/*
 * Returns the last received ACK bit.
 *
 * @mpsse - MPSSE context pointer.
 *
 * Returns either an ACK (0) or a NACK (1).
 */
int mpsse_get_ack(struct mpsse_context *mpsse)
{	
	int ack = 0;

	if(is_valid_context(mpsse))
	{
		ack = (mpsse->rack & 0x01);
	}

	return ack;
}

/*
 * Sets the transmitted ACK bit.
 *
 * @mpsse - MPSSE context pointer.
 * @ack   - 0 to send ACKs, 1 to send NACKs.
 *
 * Returns void.
 */
void mpsse_set_ack(struct mpsse_context *mpsse, int ack)
{
	if(is_valid_context(mpsse))
	{
		if(ack == NACK)
		{
			mpsse->tack = 0xFF;
		}
		else
		{
			mpsse->tack = 0x00;
		}
	}

	return;
}

/*
 * Causes libmpsse to send ACKs after each read byte in I2C mode.
 *
 * @mpsse - MPSSE context pointer.
 *
 * Returns void.
 */
void mpsse_send_acks(struct mpsse_context *mpsse)
{
	return mpsse_set_ack(mpsse, ACK);
}

/*
 * Causes libmpsse to send NACKs after each read byte in I2C mode.
 *
 * @mpsse - MPSSE context pointer.
 *
 * Returns void.
 */
void mpsse_send_nacks(struct mpsse_context *mpsse)
{
	return mpsse_set_ack(mpsse, NACK);
}

/*
 * Send data stop condition.
 *
 * @mpsse - MPSSE context pointer.
 *
 * Returns MPSSE_OK on success.
 * Returns MPSSE_FAIL on failure.
 */
int mpsse_stop(struct mpsse_context *mpsse)
{
	int retval = MPSSE_OK;

	if(is_valid_context(mpsse))
	{
		/* In I2C mode, we need to ensure that the data line goes low while the clock line is low to avoid sending an inadvertent start condition */
		if(mpsse->mode == I2C)
		{
			retval |= set_bits_low(mpsse, (mpsse->pidle & ~DO & ~SK));
		}

		/* Send the stop condition */
		retval |= set_bits_low(mpsse, mpsse->pstop);

		if(retval == MPSSE_OK)
		{
			/* Restore the pins to their idle states */
			retval |= set_bits_low(mpsse, mpsse->pidle);
		}
		
		mpsse->status = STOPPED;
	}
	else
	{
		retval = MPSSE_FAIL;
		mpsse->status = STOPPED;
	}

	return retval;
}

/*
 * Reads the specified number of bytes starting at the given address in MCU emulation mode.
 *
 * @mpsse   - MPSSE context pointer.
 * @size    - Number of bytes to read.
 * @address - Read start address.
 *
 * Returns a pointer to the read data on success.
 * Returns NULL on failure.
 */
#ifdef SWIGPYTHON
swig_string_data mpsse_mcu_read(struct mpsse_context *mpsse, int size, int address)
#else
char *mpsse_mcu_read(struct mpsse_context *mpsse, int size, int address)
#endif
{
	int i = 0, j = 0, txsize = 0, rxsize = 0;
	unsigned char *txbuf = NULL, *rxbuf = NULL;

	rxbuf = malloc(rxsize);
	if(rxbuf)
	{
		memset(rxbuf, 0, rxsize);
	
		txsize = (size * CMD_SIZE) + 1;
		txbuf = malloc(txsize);
		if(txbuf)
		{
			memset(txbuf, 0, txsize);
		
			for(i=address; i<(address+size); i++)
			{
				txbuf[j++] = mpsse->rx;
				if(mpsse->rx == CPU_READ_LONG)
				{
					txbuf[j++] = (uint8_t) ((i >> 8) & 0xFF);
				}
				txbuf[j++] = (uint8_t) (i & 0xFF);
			}
	
			txbuf[j++] = CPU_SEND_IMMEDIATE;
	
			if(raw_write(mpsse, txbuf, j) == MPSSE_OK)
			{
				if(raw_read(mpsse, rxbuf, rxsize) != rxsize)
				{
					free(rxbuf);
					rxbuf = NULL;
				}
			}
	
			free(txbuf);
		}
	}

#ifdef SWIGPYTHON
	swig_string_data sdata = { 0 };
	sdata.size = rxsize;
	sdata.data = (char *) rxbuf;
	return sdata;
#else
	return (char *) rxbuf;
#endif
}

/*
 * Writes the given data using MCU emulation mode.
 *
 * @mpsse   - MPSSE context pointer.
 * @data    - Pointer to a buffer of data to write.
 * @size    - Size of data buffer.
 * @address - Start address for writing.
 *
 * Returns MPSSE_OK on success, MPSSE_FAIL on failure.
 */
int mpsse_mcu_write(struct mpsse_context *mpsse, char *data, int size, int address)
{
	unsigned char *buf = NULL;
	int buf_size = 0, i = 0, j = 0, retval = MPSSE_FAIL;

	buf_size = (size * (CMD_SIZE + 1));
	buf = malloc(buf_size);
	if(buf)
	{
		memset(buf, 0, buf_size);

		for(i=0; i<size; i++)
		{
			buf[j++] = mpsse->tx;
			if(mpsse->tx == CPU_WRITE_LONG)
			{
				buf[j++] = (uint8_t) (((address + i) >> 8) & 0xFF);
			}
			buf[j++] = (uint8_t) ((address + i) & 0xFF);
			buf[j++] = data[i];
		}

		retval = raw_write(mpsse, buf, j);
	}

	return retval;
}

/* 
 * Sets the specified pin high. 
 *
 * @mpsse - MPSSE context pointer.
 * @pin   - Pin number to set high.
 * 
 * Returns MPSSE_OK on success.
 * Returns MPSSE_FAIL on failure.
 */
int mpsse_pin_high(struct mpsse_context *mpsse, int pin)
{
	int retval = MPSSE_FAIL;

	if(is_valid_context(mpsse))
	{
		retval = gpio_write(mpsse, pin, HIGH);
	}

	return retval;
}

/*
 * Sets the specified pin low.
 *
 * @mpsse - MPSSE context pointer.
 * @pin   - Pin number to set low.
 *
 * Returns MPSSE_OK on success.
 * Returns MPSSE_FAIL on failure.
 */
int mpsse_pin_low(struct mpsse_context *mpsse, int pin)
{
	int retval = MPSSE_FAIL;
	
	if(is_valid_context(mpsse))
	{
		retval = gpio_write(mpsse, pin, LOW);
	}

	return retval;
}

/*
 * Reads the state of the chip's pins. For use in BITBANG mode only.
 *
 * @mpsse - MPSSE context pointer.
 *
 * Returns a byte with the corresponding pin's bits set to 1 or 0.
 */
int mpsse_read_pins(struct mpsse_context *mpsse)
{
	uint8_t val = 0;

	if(is_valid_context(mpsse))
	{
		ftdi_read_pins((struct ftdi_context *) &mpsse->ftdi, (unsigned char *) &val);
	}

	return (int) val;
}

/*
 * Checks if a specific pin is high or low. For use in BITBANG mode only.
 *
 * @mpsse - MPSSE context pointer.
 * @pin   - The pin number.
 * @state - The state of the pins, as returned by ReadPins.
 *          If set to -1, ReadPins will automatically be called.
 *
 * Returns a 1 if the pin is high, 0 if the pin is low.
 */
int mpsse_pin_state(struct mpsse_context *mpsse, int pin, int state)
{
	if(state == -1)
	{
		state = mpsse_read_pins(mpsse);
	}

	/* If not in bitbang mode, the specified pin should be one of GPIOLx. Convert these defines into an absolute pin number. */
	if(mpsse->mode != BITBANG)
	{
		pin += NUM_GPIOL_PINS;
	}

	return ((state & (1 << pin)) >> pin);
}

/*
 * Toggles the clock pin until GPIOL1 is pulled high.
 *
 * @mpsse - MPSSE context pointer.
 *
 * Returns MPSSE_OK on success, MPSSE_FAIL on failure.
 */
int mpsse_clock_until_high(struct mpsse_context *mpsse)
{
	unsigned char cmd[] = { PULSE_CLOCK_IO_HIGH };

	return raw_write(mpsse, (unsigned char *) &cmd, sizeof(cmd));
}

/*
 * Toggles the clock pin until GPIOL1 is pulled low.
 *
 * @mpsse - MPSSE context pointer.
 *
 * Returns MPSSE_OK on success, MPSSE_FAIL on failure.
 */
int mpsse_clock_until_low(struct mpsse_context *mpsse)
{
	unsigned char cmd[] = { PULSE_CLOCK_IO_LOW };
	
	return raw_write(mpsse, (unsigned char *) &cmd, sizeof(cmd));
}

/*
 * Toggles the clock 1-8 cycles without transferring any data.
 * 
 * @mpsse - MPSSE context pointer.
 * @count - Number of clock cycles to send (1-8).
 *
 * Returns MPSSE_OK on success, MPSSE_FAIL on failure.
 */
int mpsse_toggle_clock(struct mpsse_context *mpsse, int count)
{
	unsigned char cmd[CMD_SIZE-1] = { 0 };

	count--;

	cmd[0] = CLOCK_N_CYCLES;
	cmd[1] = (uint8_t) (count & 7);

	return raw_write(mpsse, (unsigned char *) &cmd, sizeof(cmd));
}

/*
 * Toggles the clock 8 * count cycles without transferring any data.
 *
 * @mpsse - MPSSE context pointer.
 * @count - Number of clock cycles to send (1-65536); actual number of clock
 *          cycles sent will be eight times this value.
 * @gpio  - If set to 1, the clock cycle loop will be interrupted if GPIOL1 is pulled high.
 *          If set to 0, the clock cycle loop will be interrupted if GPIOL1 is pulled low.
 *          If set to -1, the state of GPIOL1 will be ignored.
 *
 * Returns MPSSE_OK on success, MPSSE_FAIL on failure.
 */
int mpsse_toggle_clock_x8(struct mpsse_context *mpsse, int count, int gpio)
{
	unsigned char cmd[CMD_SIZE] = { 0 };

	count--;

	switch(gpio)
	{
		case 0:
			cmd[0] = CLOCK_N8_CYCLES_IO_LOW;
			break;
		case 1:
			cmd[0] = CLOCK_N8_CYCLES_IO_HIGH;
			break;
		default:
			cmd[0] = CLOCK_N8_CYCLES;
			break;
	}

	cmd[1] = (uint8_t) (count & 0xFF);
	cmd[2] = (uint8_t) ((count >> 8) & 0xFF); 

	return MPSSE_FAIL;
}

/*
 * Places all I/O pins into a tristate mode.
 *
 * @mpsse - MPSSE context pointer.
 *
 * Returns MPSSE_OK on success, MPSSE_FAIL on failure.
 */
int mpsse_tristate(struct mpsse_context *mpsse)
{
	unsigned char cmd[CMD_SIZE] = { 0 };

	/* Tristate the all I/O pins (FT232H only) */
	cmd[0] = TRISTATE_IO;
	cmd[1] = 0xFF;
	cmd[2] = 0xFF;

	return raw_write(mpsse, (unsigned char *) &cmd, sizeof(cmd));
}

/* 
 * Returns the libmpsse version number. 
 * High nibble is major version, low nibble is minor version.
 */
int mpsse_version(void)
{
	int major = 0, minor = 0, version = 0;
	char *version_string = NULL, *decimal_ptr = NULL;

	version_string = strdup(PACKAGE_VERSION);
	if(version_string)
	{
		decimal_ptr = strchr(version_string, '.');
		if(decimal_ptr && strlen(decimal_ptr) > 1)
		{
			memset(decimal_ptr, 0, 1);
			minor = atoi(decimal_ptr+1);
		}

		major = atoi(version_string);
		
		free(version_string);
	}

	version = (major << 4) + (minor & 0x0F);

	return version;
}

