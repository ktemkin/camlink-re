/**
 *  Note: this license applies to this file. Other files in this project
 *  may be licensed differently, because Cypress likes to reserve their rights.
 *
 *  Or maybe not; I'm not a lawyer, I'm probably not in your jurisdiction, and I
 *  definitely don't believe software patents and restricted examples are helping anyone.
 *
 *       DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE 
 *                    Version 2, December 2004 
 *
 * Copyright (C) 2019 Katherine Temkin <k@ktemkin.com>, if you're not angry with the code
 * Copyright (C) 2019 Someone Else if you are angry and/or want to send me a C&D
 *
 * Everyone is permitted to copy and distribute verbatim or modified 
 * copies of this license document, and changing it is allowed as long 
 * as the name is changed. 
 *
 *            DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE 
 *   TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION 
 *
 *  0. You just DO WHAT THE FUCK YOU WANT TO.
*/

#include "sanity.h" // finally
#include "spi_flash.h"
#include "exploration.h"

/**
 * Generic buffer for USB transcations.
 */
uint8_t buffer[4096] __attribute__ ((aligned (32)));

/**
 * GPIO pin constants for each of the FPGA communication pins.
 */
enum {
    GPIO_CS   = 50,
    GPIO_CCLK = 51,
    GPIO_MOSI = 52,
    GPIO_MISO = 57
};


/**
 * Constants for each of the vendor requests.
 */
enum {
    // FPGA configuration commands.
    REQUEST_READ_FPGA_ID              = 0x50,
    REQUEST_READ_FPGA_STATUS          = 0x51,
    REQUEST_START_FPGA_CONFIGURATION  = 0x52,
    REQUEST_STREAM_FPGA_CONFIGURATION = 0x53,
    REQUEST_FINISH_FPGA_CONFIGURATION = 0x54,

    // Cypress standard commands.
    REQUEST_GET_FIRMWARE_ID           = 0xB0,

    // SPI flash.
    REQUEST_SPI_FLASH_WRITE           = 0xC2,
    REQUEST_SPI_FLASH_READ            = 0xC3,
    REQUEST_SPI_FLASH_ERASE           = 0xC4
   
};


/**
 * Constants for each of the FPGA configuration opcodes.
 */
enum {
    // Control requests.
    CONFIGURATION_OP_REFRESH         = 0x79,

    // Information requests.
    CONFIGURATION_OP_GET_IDCODE      = 0xE0,
    CONFIGURATION_OP_READ_STATUS     = 0x3C,
    CONFIGURATION_OP_CHECK_BUSY      = 0xF0,

    // In-system configuration operations.
    CONFIGURATION_OP_ISC_ENABLE      = 0xC6,
    CONFIGURATION_OP_ISC_ERASE       = 0x0E,
    CONFIGURATION_OP_SET_ADDRESS     = 0x46,
    CONFIGURATION_OP_BITSTREAM_BURST = 0x7a,
    CONFIGURATION_OP_ISC_DISABLE     = 0x26
};

/**
 * FPGA status register flags.
 */
 enum {
     FPGA_STATUS_FLAG_DONE = (1 << 8)
 };


/**
 * Drives the FPGA's chip-select low, starting an SPI transfer.
 */
void assert_cs(void)
{
    // Drive CS low, and pull clock down from idle at the same time.
    set_gpio_value(GPIO_CS, false);
    set_gpio_value(GPIO_CCLK, false);
}


/**
 * Drives the FPGA's chip-select high, ending an SPI transfer.
 */
void release_cs(void)
{
    // Drive CS high.
    set_gpio_value(GPIO_CS, true);
}


/**
* Adds a half-bit delay to our bitbanged SPI, slowing down our operating frequency.
*/
static void half_bit_delay(void)
{
    // For now, it looks like we don't need a delay.
}


/**
 * Transmits and receives a single bit over our configuration SPI bus.
 *
 * @param bit_to_send True iff this SPI cycle should issue a logic '1'.
 * @return True iff the device provided a logic '1' to read.
 */
bool fpga_exchange_bit(bool bit_to_send)
{
    bool value_read;

    // Scan out our new bit.
    set_gpio_value(GPIO_MOSI, bit_to_send);
    
    // Create our rising edge.
	half_bit_delay();
    set_gpio_value(GPIO_CCLK, true);

    // Read in the data on the SPI bus, and create our falling edge.
    half_bit_delay();
    value_read = get_gpio_value(GPIO_MISO);
    set_gpio_value(GPIO_CCLK, false);

    return value_read;
}


/**
 * Sends and receives a single byte over our bitbanged SPI bus.
 */
uint8_t fpga_exchange_byte(uint8_t to_send)
{
    uint8_t received = 0;

    for (unsigned i = 0; i < 8; ++i) {
        bool bit_to_send = !!(to_send & 0b10000000);
        bool bit_received = fpga_exchange_bit(bit_to_send);

        // Add the newly received bit to our byte, and move forward in the byte to transmit/
        received = (received << 1) | (bit_received ? 1 : 0);
        to_send <<= 1;
    }

    return received;
}


/**
 * Sends and receives a single byte over our bitbanged SPI bus.
 */
void fpga_send_byte(uint8_t to_send)
{

    for (unsigned i = 0; i < 8; ++i) {
        fpga_exchange_bit(to_send >> 7);
        to_send <<= 1;
    }
}


/**
 * Transmits and receives a collection of bytes over the SPI bus.
 * Does not handle CS assertion/de-assertion.
 */
void fpga_exchange_data(uint8_t *rx_buffer, uint8_t *tx_buffer, size_t length)
{
    for (size_t i = 0; i < length; ++i) {
        rx_buffer[i] = fpga_exchange_byte(tx_buffer[i]);
    }
}


void fpga_send_opcode(uint8_t opcode)
{
    fpga_send_byte(opcode);

    // Per the configuration guide, wait 24 cycles before reading our response.
    fpga_send_byte(0x00);
    fpga_send_byte(0x00);
    fpga_send_byte(0x00);
}


/**
 * Performs a simple FPGA 'in' command, which issues a single opcode, and then reads a four-byte result.
 */
uint32_t fpga_simple_command(uint8_t opcode, bool get_response)
{
    uint8_t result[4];
    uint32_t *result_int = (uint32_t *)result;

    // Start our transaction by asserting CS and sending the opcode.
    assert_cs();
    fpga_send_opcode(opcode);

    // Read the response...
    if (get_response) {
        for (unsigned i = 0; i < sizeof(result); ++i) {
            result[i] = fpga_exchange_byte(0x00);
        }
    }

    // ... terminate the transaction ...
    release_cs();

    // ... and return the result.
    return *result_int;
}



/**
 * Requests the FPGA's status byte.
 */
uint32_t fpga_get_id(void)
{
    return fpga_simple_command(CONFIGURATION_OP_GET_IDCODE, true);
}

/**
 * Requests the FPGA's status byte.
 */
uint32_t fpga_get_status(void)
{
    return fpga_simple_command(CONFIGURATION_OP_READ_STATUS, true);
}


/**
 * Requests the FPGA's status byte.
 */
bool fpga_is_busy(void)
{
    uint8_t busy;

    // Perform our core data exchange.
    assert_cs();
    fpga_send_opcode(CONFIGURATION_OP_CHECK_BUSY);
    busy = fpga_exchange_byte(0x00);
    release_cs();

    return !!busy;
}


/**
 * Waits for the FPGA to have completed its last operation.
 */
void fpga_wait_for_operation_completion(void)
{
    while (fpga_is_busy());
}



/* Convenience function that sends a uint32_t on EP0. */
void send_uint32_on_ep0(uint32_t datum)
{
    send_on_ep0(&datum, sizeof(datum));
}


/**
 * Handles a request to start FPGA configuration.
 */
void handle_start_fpga_configuration_request(void)
{
    // Trigger an FPGA reconfiguration.
    fpga_simple_command(CONFIGURATION_OP_REFRESH, false);
    fpga_wait_for_operation_completion();
    CyU3PThreadSleep(1000);

    // Enable in-system configuration mode.
    fpga_simple_command(CONFIGURATION_OP_ISC_ENABLE, false);

    // Erase the device's configuration SRAM.
    fpga_simple_command(CONFIGURATION_OP_ISC_ERASE, false);
    fpga_wait_for_operation_completion();

    // Start at the beginning of our configuration.
    fpga_simple_command(CONFIGURATION_OP_SET_ADDRESS, false);

    // Read the device's status, and include it as a response.
    send_uint32_on_ep0(fpga_get_status());

    // Prepare to scan out the bitstream itself.
    // We'll issue the opcode and the required 24-bits of zeroes, but
    // leave the configuration channel open for bitstream transmission.
    assert_cs();
    fpga_exchange_byte(CONFIGURATION_OP_BITSTREAM_BURST);
    fpga_exchange_byte(0x00);
    fpga_exchange_byte(0x00);
    fpga_exchange_byte(0x00);
}



/**
 * Handles a request to finish FPGA configuration.
 */
void handle_fpga_bitstream_request(uint16_t length)
{
    int rc = receive_on_ep0(buffer, length, NULL);
    if (!rc) {
        for (int i = 0; i < length; ++i) {
            fpga_send_byte(buffer[i]);
        }
    }
}


/**
 * Handles a request to finish FPGA configuration.
 */
int handle_finish_fpga_configuration_request(void)
{
    uint32_t status;
 
    // Terminate the active bitstream scanout operation.
    release_cs();
    status = fpga_get_status();

    // If the operation completed successfully, drop out of configuration mode.
    fpga_simple_command(CONFIGURATION_OP_ISC_DISABLE, false);
    
    // Either way, send back the status word we read.
    send_uint32_on_ep0(status);
    return 0;
}



/**
 * Core vendor request handling function.
 */
int handle_vendor_request(uint8_t request_type, uint8_t request, uint16_t value, uint16_t index, uint16_t length)
{
    int status = 0;

    switch (request)
    {

        // General firmware ID query.
        case REQUEST_GET_FIRMWARE_ID:
            send_on_ep0 ("CAMLINK\0", 8);
            return 0;


        // Simple queries.
        case REQUEST_READ_FPGA_ID:
            send_uint32_on_ep0(fpga_get_id());
            return 0;
        case REQUEST_READ_FPGA_STATUS:
            send_uint32_on_ep0(fpga_get_status());
            return 0;


        // FPGA configuration.
        case REQUEST_START_FPGA_CONFIGURATION:
            handle_start_fpga_configuration_request();
        case REQUEST_STREAM_FPGA_CONFIGURATION:
            handle_fpga_bitstream_request(length);
            return 0;

        case REQUEST_FINISH_FPGA_CONFIGURATION:
            return handle_finish_fpga_configuration_request();

        // SPI flash program.
        case REQUEST_SPI_FLASH_WRITE:
            status = receive_on_ep0(buffer, length, NULL);
            if (status == CY_U3P_SUCCESS) {
                status = spi_flash_program(index, length, buffer, false);
            }
            return status;


        // SPI flash read.
        case REQUEST_SPI_FLASH_READ:
            memset(buffer, 0, sizeof (buffer));
            status = spi_flash_program(index, length, buffer, true);
            if (status == CY_U3P_SUCCESS) {
                send_on_ep0(buffer, length);
            }
            return status;


        // SPI flash erase or get status.
        // FIXME: separate these
        case REQUEST_SPI_FLASH_ERASE:
            status = spi_flash_erase_sector(value, (index & 0xFF), buffer);
            if (status == 0)
            {
                if (value == 0) {
                    send_on_ep0(buffer, length);
                } else {
                    send_ep0_ack();
                }
            }
            return status;

        default:
            return -1;
    }
}

/**
 * Quick GPIO set up function.
 */
void set_up_pin(uint8_t number, bool is_input)
{
    CyU3PGpioSimpleConfig_t gpioConfig;

    gpioConfig.outValue = CyTrue;
    gpioConfig.driveLowEn = !is_input;
    gpioConfig.driveHighEn = !is_input;
    gpioConfig.inputEn = is_input;
    gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
    CyU3PGpioSetSimpleConfig(number, &gpioConfig);
}


/**
 * Configures the GPIO bus to allow for us to communicate with the FPGA.
 */
void set_up_gpio(void)
{
    CyU3PGpioClock_t gpioClock;
    CyU3PIoMatrixConfig_t io_cfg;

    /* Configure the IO matrix for the device. On the FX3 DVK board, the COM port 
     * is connected to the IO(53:56). So since we need to use SPI, we will have to
     * either not use UART or use an external UART controller on the IO(46:49). */
    CyU3PMemSet ((uint8_t *)&io_cfg, 0, sizeof(io_cfg));
    io_cfg.isDQ32Bit = CyFalse;
    io_cfg.s0Mode = CY_U3P_SPORT_INACTIVE;
    io_cfg.s1Mode = CY_U3P_SPORT_INACTIVE;
    io_cfg.useUart   = CyTrue;
    io_cfg.useI2C    = CyTrue;
    io_cfg.useI2S    = CyFalse;
    io_cfg.useSpi    = CyTrue;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
    io_cfg.gpioSimpleEn[0]  = 0;
    io_cfg.gpioSimpleEn[1]  = (1 << 18) | (1 << 19) | (1 << 20) |  (1 << 25); // 50, 51, 52, 57
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    CyU3PDeviceConfigureIOMatrix (&io_cfg);

    /**
     * Set up the overall GPIO controller.
     */
    gpioClock.fastClkDiv = 2;
    gpioClock.slowClkDiv = 0;
    gpioClock.simpleDiv = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
    gpioClock.clkSrc = CY_U3P_SYS_CLK;
    gpioClock.halfDiv = 0;
    CyU3PGpioInit(&gpioClock, NULL);

    /**
     * Set up our configuration GPIO pins:
     *     50: CS
     *     51: CCLK
     *     52: IO0
     *     57: IO1
     */
    set_up_pin(GPIO_CS,   false);
    set_up_pin(GPIO_CCLK, false);
    set_up_pin(GPIO_MOSI, false);
    set_up_pin(GPIO_MISO, true);

    set_gpio_value(GPIO_CCLK, false);
}


/*
 * Main function
 */
int main(void)
{
    /* Initialize the device */
    CyU3PDeviceInit(NULL);
    CyU3PDeviceCacheControl(CyTrue, CyTrue, CyTrue);

    // Set up the GPIO pins we use to e.g. configure the FPGA.
    set_up_gpio();

    /* This is a non returnable call for initializing the RTOS kernel */
    CyU3PKernelEntry();

    /* Dummy return to make the compiler happy */
    return 0;
}
