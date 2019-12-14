/*
## Cypress USB 3.0 Platform source file (cyfxflashprog.c)
## ===========================
##
##  Copyright Cypress Semiconductor Corporation, 2010-2011,
##  All Rights Reserved
##  UNPUBLISHED, LICENSED SOFTWARE.
##
##  CONFIDENTIAL AND PROPRIETARY INFORMATION
##  WHICH IS THE PROPERTY OF CYPRESS.
##
##  Use of this file is governed
##  by the license agreement included in the file
##
##     <install>/license/license.txt
##
##  where <install> is the Cypress software
##  installation root directory path.
##
## ===========================
*/

/* This application is used to program the FX3 firmware onto I2C EEPROM or SPI FLASH devices
   connected to the FX3 device. The firmware enumerates as a custom device communicating with
   the CyUsb3.sys driver and provides a set of vendor commands that can be used to download the
   firmware to the EEPROM/FLASH devices.
 */

#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "cyu3i2c.h"
#include "cyu3spi.h"
#include "cyu3uart.h"
#include "cyfxflashprog.h"

CyU3PThread appThread;                  /* Application thread object. */
CyBool_t glIsApplnActive = CyFalse;

/* Firmware ID variable that may be used to verify flash programmer firmware. */
const uint8_t glFirmwareID[32] __attribute__ ((aligned (32))) = { 'C', 'A', 'M', 'L', 'I', 'N', 'K', '\0' };

uint8_t glEp0Buffer[4096] __attribute__ ((aligned (32)));

uint16_t glI2cPageSize = 0x40;   /* I2C Page size to be used for transfers. */
uint16_t glSpiPageSize = 0x100;  /* SPI Page size to be used for transfers. */

CyU3PDmaChannel glI2cTxHandle;   /* I2C Tx channel handle */
CyU3PDmaChannel glI2cRxHandle;   /* I2C Rx channel handle */
CyU3PDmaChannel glSpiTxHandle;   /* SPI Tx channel handle */
CyU3PDmaChannel glSpiRxHandle;   /* SPI Rx channel handle */

/* Initialize the debug module with UART. */
CyU3PReturnStatus_t
CyFxDebugInit (
        void)
{
    CyU3PUartConfig_t uartConfig;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Initialize and configure the UART for logging. */
    status = CyU3PUartInit ();
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    CyU3PMemSet ((uint8_t *)&uartConfig, 0, sizeof (uartConfig));
    uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
    uartConfig.stopBit  = CY_U3P_UART_ONE_STOP_BIT;
    uartConfig.parity   = CY_U3P_UART_NO_PARITY;
    uartConfig.txEnable = CyTrue;
    uartConfig.rxEnable = CyFalse;
    uartConfig.flowCtrl = CyFalse;
    uartConfig.isDma    = CyTrue;
    status = CyU3PUartSetConfig (&uartConfig, NULL);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Set the dma for an inifinity transfer */
    status = CyU3PUartTxSetBlockXfer (0xFFFFFFFF);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Start the debug module for printing log messages. */
    status = CyU3PDebugInit (CY_U3P_LPP_SOCKET_UART_CONS, 8);

    return status;
}

/* I2c initialization for EEPROM programming. */
CyU3PReturnStatus_t
CyFxFlashProgI2cInit (uint16_t pageLen)
{
    CyU3PI2cConfig_t i2cConfig;
    CyU3PDmaChannelConfig_t dmaConfig;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Initialize and configure the I2C master module. */
    status = CyU3PI2cInit ();
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Start the I2C master block. The bit rate is set at 100KHz.
     * The data transfer is done via DMA. */
    CyU3PMemSet ((uint8_t *)&i2cConfig, 0, sizeof(i2cConfig));
    i2cConfig.bitRate    = 100000;
    i2cConfig.busTimeout = 0xFFFFFFFF;
    i2cConfig.dmaTimeout = 0xFFFF;
    i2cConfig.isDma      = CyTrue;

    status = CyU3PI2cSetConfig (&i2cConfig, NULL);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Now create the DMA channels required for read and write. */
    CyU3PMemSet ((uint8_t *)&dmaConfig, 0, sizeof(dmaConfig));
    dmaConfig.size           = pageLen;
    /* No buffers need to be allocated as this will be used
     * only in override mode. */
    dmaConfig.count          = 0;
    dmaConfig.prodAvailCount = 0;
    dmaConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
    dmaConfig.prodHeader     = 0;
    dmaConfig.prodFooter     = 0;
    dmaConfig.consHeader     = 0;
    dmaConfig.notification   = 0;
    dmaConfig.cb             = NULL;

    /* Create a channel to write to the EEPROM. */
    dmaConfig.prodSckId = CY_U3P_CPU_SOCKET_PROD;
    dmaConfig.consSckId = CY_U3P_LPP_SOCKET_I2C_CONS;
    status = CyU3PDmaChannelCreate (&glI2cTxHandle,
            CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaConfig);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Create a channel to read from the EEPROM. */
    dmaConfig.prodSckId = CY_U3P_LPP_SOCKET_I2C_PROD;
    dmaConfig.consSckId = CY_U3P_CPU_SOCKET_CONS;
    status = CyU3PDmaChannelCreate (&glI2cRxHandle,
            CY_U3P_DMA_TYPE_MANUAL_IN, &dmaConfig);

    if (status == CY_U3P_SUCCESS)
    {
        glI2cPageSize = pageLen;
    }

    return status;
}

/* I2C read / write for programmer application. */
CyU3PReturnStatus_t
CyFxFlashProgI2cTransfer (
        uint16_t  byteAddress,
        uint8_t   devAddr,
        uint16_t  byteCount,
        uint8_t  *buffer,
        CyBool_t  isRead)
{
    CyU3PDmaBuffer_t buf_p;
    CyU3PI2cPreamble_t preamble;
    uint16_t pageCount = (byteCount / glI2cPageSize);
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if (byteCount == 0)
    {
        return CY_U3P_SUCCESS;
    }

    if ((byteCount % glI2cPageSize) != 0)
    {
        pageCount ++;
    }

    CyU3PDebugPrint (2, "I2C access - dev: 0x%x, address: 0x%x, size: 0x%x, pages: 0x%x.\r\n",
            devAddr, byteAddress, byteCount, pageCount);

    /* Update the buffer address and status. */
    buf_p.buffer = buffer;
    buf_p.status = 0;

    while (pageCount != 0)
    {
        if (isRead)
        {
            /* Update the preamble information. */
            preamble.length    = 4;
            preamble.buffer[0] = devAddr;
            preamble.buffer[1] = (uint8_t)(byteAddress >> 8);
            preamble.buffer[2] = (uint8_t)(byteAddress & 0xFF);
            preamble.buffer[3] = (devAddr | 0x01);
            preamble.ctrlMask  = 0x0004;

            buf_p.size = glI2cPageSize;
            buf_p.count = glI2cPageSize;

            status = CyU3PI2cSendCommand (&preamble, glI2cPageSize, isRead);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
            status = CyU3PDmaChannelSetupRecvBuffer (&glI2cRxHandle, &buf_p);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
            status = CyU3PDmaChannelWaitForCompletion(&glI2cRxHandle,
                    CY_FX_FLASH_PROG_TIMEOUT);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
        }
        else /* Write */
        {
            /* Update the preamble information. */
            preamble.length    = 3;
            preamble.buffer[0] = devAddr;
            preamble.buffer[1] = (uint8_t)(byteAddress >> 8);
            preamble.buffer[2] = (uint8_t)(byteAddress & 0xFF);
            preamble.ctrlMask  = 0x0000;

            buf_p.size = glI2cPageSize;
            buf_p.count = glI2cPageSize;

            status = CyU3PDmaChannelSetupSendBuffer (&glI2cTxHandle,
                    &buf_p);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
            status = CyU3PI2cSendCommand (&preamble, glI2cPageSize, isRead);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
            status = CyU3PDmaChannelWaitForCompletion(&glI2cTxHandle,
                    CY_FX_FLASH_PROG_TIMEOUT);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
        }

        /* Update the parameters */
        byteAddress  += glI2cPageSize;
        buf_p.buffer += glI2cPageSize;
        pageCount --;

        /* Need a delay between write operations. */
        CyU3PThreadSleep (10);
    }

    return CY_U3P_SUCCESS;
}

/* SPI initialization for flash programmer application. */
CyU3PReturnStatus_t
CyFxFlashProgSpiInit (uint16_t pageLen)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PSpiConfig_t spiConfig;
    CyU3PDmaChannelConfig_t dmaConfig;

    /* Start the SPI module and configure the master. */
    status = CyU3PSpiInit();
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Start the SPI master block. Run the SPI clock at 8MHz
     * and configure the word length to 8 bits. Also configure
     * the slave select using FW. */
    CyU3PMemSet ((uint8_t *)&spiConfig, 0, sizeof(spiConfig));
    spiConfig.isLsbFirst = CyFalse;
    spiConfig.cpol       = CyTrue;
    spiConfig.ssnPol     = CyFalse;
    spiConfig.cpha       = CyTrue;
    spiConfig.leadTime   = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.lagTime    = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_FW;
    spiConfig.clock      = 8000000;
    spiConfig.wordLen    = 8;

    status = CyU3PSpiSetConfig (&spiConfig, NULL);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Create the DMA channels for SPI write and read. */
    CyU3PMemSet ((uint8_t *)&dmaConfig, 0, sizeof(dmaConfig));
    dmaConfig.size           = pageLen;
    /* No buffers need to be allocated as this channel
     * will be used only in override mode. */
    dmaConfig.count          = 0;
    dmaConfig.prodAvailCount = 0;
    dmaConfig.dmaMode        = CY_U3P_DMA_MODE_BYTE;
    dmaConfig.prodHeader     = 0;
    dmaConfig.prodFooter     = 0;
    dmaConfig.consHeader     = 0;
    dmaConfig.notification   = 0;
    dmaConfig.cb             = NULL;

    /* Channel to write to SPI flash. */
    dmaConfig.prodSckId = CY_U3P_CPU_SOCKET_PROD;
    dmaConfig.consSckId = CY_U3P_LPP_SOCKET_SPI_CONS;
    status = CyU3PDmaChannelCreate (&glSpiTxHandle,
            CY_U3P_DMA_TYPE_MANUAL_OUT, &dmaConfig);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Channel to read from SPI flash. */
    dmaConfig.prodSckId = CY_U3P_LPP_SOCKET_SPI_PROD;
    dmaConfig.consSckId = CY_U3P_CPU_SOCKET_CONS;
    status = CyU3PDmaChannelCreate (&glSpiRxHandle,
            CY_U3P_DMA_TYPE_MANUAL_IN, &dmaConfig);

    if (status == CY_U3P_SUCCESS)
    {
        glSpiPageSize = pageLen;
    }

    return status;
}

/* Wait for the status response from the SPI flash. */
CyU3PReturnStatus_t
CyFxFlashProgSpiWaitForStatus (
        void)
{
    uint8_t buf[2], rd_buf[2];
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Wait for status response from SPI flash device. */
    do
    {
        buf[0] = 0x06;  /* Write enable command. */

        CyU3PSpiSetSsnLine (CyFalse);
        status = CyU3PSpiTransmitWords (buf, 1);
        CyU3PSpiSetSsnLine (CyTrue);
        if (status != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (2, "SPI WR_ENABLE command failed\n\r");
            return status;
        }

        buf[0] = 0x05;  /* Read status command */

        CyU3PSpiSetSsnLine (CyFalse);
        status = CyU3PSpiTransmitWords (buf, 1);
        if (status != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (2, "SPI READ_STATUS command failed\n\r");
            CyU3PSpiSetSsnLine (CyTrue);
            return status;
        }

        status = CyU3PSpiReceiveWords (rd_buf, 2);
        CyU3PSpiSetSsnLine (CyTrue);
        if(status != CY_U3P_SUCCESS)
        {
            CyU3PDebugPrint (2, "SPI status read failed\n\r");
            return status;
        }

    } while ((rd_buf[0] & 1)|| (!(rd_buf[0] & 0x2)));

    return CY_U3P_SUCCESS;
}

/* SPI read / write for programmer application. */
CyU3PReturnStatus_t
CyFxFlashProgSpiTransfer (
        uint16_t  pageAddress,
        uint16_t  byteCount,
        uint8_t  *buffer,
        CyBool_t  isRead)
{
    CyU3PDmaBuffer_t buf_p;
    uint8_t location[4];
    uint32_t byteAddress = 0;
    uint16_t pageCount = (byteCount / glSpiPageSize);
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if (byteCount == 0)
    {
        return CY_U3P_SUCCESS;
    }
    if ((byteCount % glSpiPageSize) != 0)
    {
        pageCount ++;
    }

    buf_p.buffer = buffer;
    buf_p.status = 0;

    byteAddress  = pageAddress * glSpiPageSize;
    CyU3PDebugPrint (2, "SPI access - addr: 0x%x, size: 0x%x, pages: 0x%x.\r\n",
            byteAddress, byteCount, pageCount);

    while (pageCount != 0)
    {
        location[1] = (byteAddress >> 16) & 0xFF;       /* MS byte */
        location[2] = (byteAddress >> 8) & 0xFF;
        location[3] = byteAddress & 0xFF;               /* LS byte */

        if (isRead)
        {
            location[0] = 0x03; /* Read command. */

            buf_p.size  = glSpiPageSize;
            buf_p.count = glSpiPageSize;

            status = CyFxFlashProgSpiWaitForStatus ();
            if (status != CY_U3P_SUCCESS)
                return status;

            CyU3PSpiSetSsnLine (CyFalse);
            status = CyU3PSpiTransmitWords (location, 4);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PDebugPrint (2, "SPI READ command failed\r\n");
                CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }

            CyU3PSpiSetBlockXfer (0, glSpiPageSize);

            status = CyU3PDmaChannelSetupRecvBuffer (&glSpiRxHandle,
                    &buf_p);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }
            status = CyU3PDmaChannelWaitForCompletion (&glSpiRxHandle,
                    CY_FX_FLASH_PROG_TIMEOUT);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }

            CyU3PSpiSetSsnLine (CyTrue);
            CyU3PSpiDisableBlockXfer (CyFalse, CyTrue);
        }
        else /* Write */
        {
            location[0] = 0x02; /* Write command */

            buf_p.size  = glSpiPageSize;
            buf_p.count = glSpiPageSize;

            status = CyFxFlashProgSpiWaitForStatus ();
            if (status != CY_U3P_SUCCESS)
                return status;

            CyU3PSpiSetSsnLine (CyFalse);
            status = CyU3PSpiTransmitWords (location, 4);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PDebugPrint (2, "SPI WRITE command failed\r\n");
                CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }

            CyU3PSpiSetBlockXfer (glSpiPageSize, 0);

            status = CyU3PDmaChannelSetupSendBuffer (&glSpiTxHandle,
                    &buf_p);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }
            status = CyU3PDmaChannelWaitForCompletion(&glSpiTxHandle,
                    CY_FX_FLASH_PROG_TIMEOUT);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }

            CyU3PSpiSetSsnLine (CyTrue);
            CyU3PSpiDisableBlockXfer (CyTrue, CyFalse);
        }

        /* Update the parameters */
        byteAddress  += glSpiPageSize;
        buf_p.buffer += glSpiPageSize;
        pageCount --;

        CyU3PThreadSleep (10);
    }
    return CY_U3P_SUCCESS;
}

/* Function to erase SPI flash sectors. */
static CyU3PReturnStatus_t
CyFxFlashProgEraseSector (
     CyBool_t  isErase,
     uint8_t   sector,
     uint8_t  *wip)
{
    uint32_t temp = 0;
    uint8_t  location[4], rdBuf[2];
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if ((!isErase) && (wip == NULL))
    {
        return CY_U3P_ERROR_BAD_ARGUMENT;
    }

    location[0] = 0x06;  /* Write enable. */

    CyU3PSpiSetSsnLine (CyFalse);
    status = CyU3PSpiTransmitWords (location, 1);
    CyU3PSpiSetSsnLine (CyTrue);
    if (status != CY_U3P_SUCCESS)
        return status;

    if (isErase)
    {
        location[0] = 0xD8; /* Sector erase. */
        temp        = sector * 0x10000;
        location[1] = (temp >> 16) & 0xFF;
        location[2] = (temp >> 8) & 0xFF;
        location[3] = temp & 0xFF;

        CyU3PSpiSetSsnLine (CyFalse);
        status = CyU3PSpiTransmitWords (location, 4);
        CyU3PSpiSetSsnLine (CyTrue);
    }
    else
    {
        location[0] = 0x05; /* Read status */

        CyU3PSpiSetSsnLine (CyFalse);
        status = CyU3PSpiTransmitWords (location, 1);
        if (status != CY_U3P_SUCCESS)
        {
            CyU3PSpiSetSsnLine (CyTrue);
            return status;
        }

        status = CyU3PSpiReceiveWords (rdBuf, 2);
        CyU3PSpiSetSsnLine (CyTrue);
        *wip = rdBuf[0] & 0x1;
    }

    return status;
}

CyBool_t
CyFxUSBSetupCB (
        uint32_t setupdat0,
        uint32_t setupdat1)
{
    /* Fast enumeration is used. Only requests addressed to the interface, class,
     * vendor and unknown control requests are received by this function. */

    uint8_t  i2cAddr = 0;
    uint32_t *addr = NULL;
    int32_t offset = 0;
    uint8_t  bRequest, bReqType;
    uint8_t  bType, bTarget;
    uint16_t wValue, wIndex, wLength;
    CyBool_t isHandled = CyFalse;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Decode the fields from the setup request. */
    bReqType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
    bType    = (bReqType & CY_U3P_USB_TYPE_MASK);
    bTarget  = (bReqType & CY_U3P_USB_TARGET_MASK);
    bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
    wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
    wIndex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);
    wLength   = ((setupdat1 & CY_U3P_USB_LENGTH_MASK)   >> CY_U3P_USB_LENGTH_POS);

    if (bType == CY_U3P_USB_STANDARD_RQT)
    {
        /* Handle SET_FEATURE(FUNCTION_SUSPEND) and CLEAR_FEATURE(FUNCTION_SUSPEND)
         * requests here. It should be allowed to pass if the device is in configured
         * state and failed otherwise. */
        if ((bTarget == CY_U3P_USB_TARGET_INTF) && ((bRequest == CY_U3P_USB_SC_SET_FEATURE)
                    || (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) && (wValue == 0))
        {
            if (glIsApplnActive)
                CyU3PUsbAckSetup ();
            else
                CyU3PUsbStall (0, CyTrue, CyFalse);

            isHandled = CyTrue;
        }
    }

    /* Handle supported vendor requests. */
    if (bType == CY_U3P_USB_VENDOR_RQT)
    {
        isHandled = CyTrue;

        switch (bRequest)
        {
            case CY_FX_RQT_ID_CHECK:
                CyU3PUsbSendEP0Data (8, (uint8_t *)glFirmwareID);
                break;

            case CY_FX_RQT_I2C_EEPROM_WRITE:
                i2cAddr = 0xA0 | ((wValue & 0x0007) << 1);
                status  = CyU3PUsbGetEP0Data(wLength, glEp0Buffer, NULL);
                if (status == CY_U3P_SUCCESS)
                {
                    CyFxFlashProgI2cTransfer (wIndex, i2cAddr, wLength,
                            glEp0Buffer, CyFalse);
                }
                break;

            case CY_FX_RQT_I2C_EEPROM_READ:
                i2cAddr = 0xA0 | ((wValue & 0x0007) << 1);
                CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
                status = CyFxFlashProgI2cTransfer (wIndex, i2cAddr, wLength,
                        glEp0Buffer, CyTrue);
                if (status == CY_U3P_SUCCESS)
                {
                    status = CyU3PUsbSendEP0Data(wLength, glEp0Buffer);
                }
                break;

            case CY_FX_RQT_SYS_MEM_READ:
                addr = (uint32_t *)((wIndex << 16) | wValue);
                offset = 0;
                if (wLength)
                {
                    while (offset < (wLength / 4))
                    {
                        ((uint32_t *)glEp0Buffer)[offset++] = *addr;
                        addr++;
                    }

                    status = CyU3PUsbSendEP0Data (wLength, glEp0Buffer);
                }
                else
                {
                    /* Zero length read. Just ACK.*/
                    CyU3PUsbAckSetup ();
                }
                break;

            case CY_FX_RQT_SPI_FLASH_WRITE:
                status = CyU3PUsbGetEP0Data (wLength, glEp0Buffer, NULL);
                if (status == CY_U3P_SUCCESS)
                {
                    status = CyFxFlashProgSpiTransfer (wIndex, wLength,
                            glEp0Buffer, CyFalse);
                }
                break;

            case CY_FX_RQT_SPI_FLASH_READ:
                CyU3PMemSet (glEp0Buffer, 0, sizeof (glEp0Buffer));
                status = CyFxFlashProgSpiTransfer (wIndex, wLength,
                        glEp0Buffer, CyTrue);
                if (status == CY_U3P_SUCCESS)
                {
                    status = CyU3PUsbSendEP0Data (wLength, glEp0Buffer);
                }
                break;

            case CY_FX_RQT_SPI_FLASH_ERASE_POLL:
                status = CyFxFlashProgEraseSector ((wValue) ? CyTrue : CyFalse,
                        (wIndex & 0xFF), glEp0Buffer);
                if (status == CY_U3P_SUCCESS)
                {
                    if (wValue == 0)
                    {
                        status = CyU3PUsbSendEP0Data (wLength, glEp0Buffer);
                    }
                    else
                    {
                        CyU3PUsbAckSetup ();
                    }
                }
                break;

            default:
                /* This is unknown request. */
                isHandled = CyFalse;
                break;
        }

        /* If there was any error, return not handled so that the library will
         * stall the request. Alternatively EP0 can be stalled here and return
         * CyTrue. */
        if (status != CY_U3P_SUCCESS)
        {
            isHandled = CyFalse;
        }
    }

    return isHandled;
}

/* This is the callback function to handle the USB events. */
void
CyFxUSBEventCB (
    CyU3PUsbEventType_t evtype, /* Event type */
    uint16_t            evdata  /* Event data */
    )
{
    switch (evtype)
    {
        case CY_U3P_USB_EVENT_SETCONF:
            glIsApplnActive = CyTrue;
            break;

        case CY_U3P_USB_EVENT_RESET:
        case CY_U3P_USB_EVENT_DISCONNECT:
            glIsApplnActive = CyFalse;
            /* Reset the I2C and SPI DMA channels. */
            CyU3PDmaChannelReset (&glI2cTxHandle);
            CyU3PDmaChannelReset (&glI2cRxHandle);
            CyU3PDmaChannelReset (&glSpiTxHandle);
            CyU3PDmaChannelReset (&glSpiRxHandle);
            break;

        default:
            break;
    }
}

/* Callback function to handle LPM requests from the USB 3.0 host. This function is invoked by the API
   whenever a state change from U0 -> U1 or U0 -> U2 happens. If we return CyTrue from this function, the
   FX3 device is retained in the low power state. If we return CyFalse, the FX3 device immediately tries
   to trigger an exit back to U0.

   This application does not have any state in which we should not allow U1/U2 transitions; and therefore
   the function always return CyTrue.
 */
CyBool_t
CyFxApplnLPMRqtCB (
        CyU3PUsbLinkPowerMode link_mode)
{
    return CyTrue;
}

/* Initialize all interfaces for the application. */
CyU3PReturnStatus_t
CyFxFlashProgInit (
        void)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Initialize the I2C interface for the EEPROM of page size 64 bytes. */
    status = CyFxFlashProgI2cInit (0x40);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Initialize the SPI interface for flash of page size 256 bytes. */
    status = CyFxFlashProgSpiInit (0x100);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Start the USB functionality. */
    status = CyU3PUsbStart();
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* The fast enumeration is the easiest way to setup a USB connection,
     * where all enumeration phase is handled by the library. Only the
     * class / vendor requests need to be handled by the application. */
    CyU3PUsbRegisterSetupCallback(CyFxUSBSetupCB, CyTrue);

    /* Setup the callback to handle the USB events. */
    CyU3PUsbRegisterEventCallback(CyFxUSBEventCB);

    /* Register a callback to handle LPM requests from the USB 3.0 host. */
    CyU3PUsbRegisterLPMRequestCallback(CyFxApplnLPMRqtCB);    
    
    /* Set the USB Enumeration descriptors */

    /* Super speed device descriptor. */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, NULL, (uint8_t *)CyFxUSB30DeviceDscr);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* High speed device descriptor. */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, NULL, (uint8_t *)CyFxUSB20DeviceDscr);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* BOS descriptor */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, NULL, (uint8_t *)CyFxUSBBOSDscr);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Device qualifier descriptor */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, NULL, (uint8_t *)CyFxUSBDeviceQualDscr);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Super speed configuration descriptor */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, NULL, (uint8_t *)CyFxUSBSSConfigDscr);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* High speed configuration descriptor */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, NULL, (uint8_t *)CyFxUSBHSConfigDscr);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Full speed configuration descriptor */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, NULL, (uint8_t *)CyFxUSBFSConfigDscr);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* String descriptor 0 */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* String descriptor 1 */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* String descriptor 2 */
    status = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)CyFxUSBProductDscr);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Connect the USB Pins with super speed operation enabled. */
    status = CyU3PConnectState(CyTrue, CyTrue);

    return status;
}

/*
 * Entry function for the application thread. This function performs
 * the initialization of the Debug, I2C, SPI and USB modules and then
 * executes in a loop printing out heartbeat messages through the UART.
 * All of the flash programming functionality is implemented in callbacks.
 */
void
AppThread_Entry (
        uint32_t input)
{
    uint8_t count = 0;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Initialize the debug interface. */
    status = CyFxDebugInit ();
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_error;
    }

    /* Initialize the application. */
    status = CyFxFlashProgInit ();
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_error;
    }

    for (;;)
    {
        CyU3PDebugPrint (4, "%x: Device initialized. Firmware ID: %x %x %x %x %x %x %x %x\r\n",
                count++, glFirmwareID[3], glFirmwareID[2], glFirmwareID[1], glFirmwareID[0],
                glFirmwareID[7], glFirmwareID[6], glFirmwareID[5], glFirmwareID[4]);
        CyU3PThreadSleep (1000);
    }

handle_error:
    CyU3PDebugPrint (4, "%x: Application failed to initialize. Error code: %d.\n", status);
    while (1);
}

/* Application define function which creates the application threads. */
void
CyFxApplicationDefine (
        void)
{
    void *ptr = NULL;
    uint32_t retThrdCreate = CY_U3P_SUCCESS;

    /* Allocate the memory for the threads and create threads */
    ptr = CyU3PMemAlloc (APPTHREAD_STACK);
    retThrdCreate = CyU3PThreadCreate (&appThread, /* Thread structure. */
            "21:AppThread",                        /* Thread ID and name. */        
            AppThread_Entry,                       /* Thread entry function. */
            0,                                     /* Thread input parameter. */
            ptr,                                   /* Pointer to the allocated thread stack. */
            APPTHREAD_STACK,                       /* Allocated thread stack size. */
            APPTHREAD_PRIORITY,                    /* Thread priority. */
            APPTHREAD_PRIORITY,                    /* Thread pre-emption threshold: No preemption. */
            CYU3P_NO_TIME_SLICE,                   /* No time slice. Thread will run until task is
                                                      completed or until the higher priority 
                                                      thread gets active. */
            CYU3P_AUTO_START                       /* Start the thread immediately. */
            );

    /* Check the return code */
    if (retThrdCreate != 0)
    {
        /* Thread creation failed with the error code retThrdCreate */

        /* Add custom recovery or debug actions here */

        /* Application cannot continue. Loop indefinitely */
        while(1);
    }
}

/*
 * Main function
 */
int
main (void)
{
    CyU3PIoMatrixConfig_t io_cfg;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Initialize the device */
    status = CyU3PDeviceInit (NULL);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Initialize the caches. Enable both Instruction and Data caches. */
    status = CyU3PDeviceCacheControl (CyTrue, CyTrue, CyTrue);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

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
    /* No GPIOs are enabled. */
    io_cfg.gpioSimpleEn[0]  = 0;
    io_cfg.gpioSimpleEn[1]  = 0;
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    status = CyU3PDeviceConfigureIOMatrix (&io_cfg);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* This is a non returnable call for initializing the RTOS kernel */
    CyU3PKernelEntry ();

    /* Dummy return to make the compiler happy */
    return 0;

handle_fatal_error:

    /* Cannot recover from this error. */
    while (1);
}

/* [ ] */

