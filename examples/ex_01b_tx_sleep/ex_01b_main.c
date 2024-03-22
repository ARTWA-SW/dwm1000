/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   TX with sleep example code
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#ifdef EX_01B_DEF
#include "deca_device_api.h"
#include "deca_regs.h"
#include "lcd.h"
#include "deca_spi.h"
#include "port.h"

/* Example application name and version to display on LCD screen. */
#define APP_NAME "TX SLEEP v1.2"

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. Used in TX only. */
    DWT_PLEN_1024,   /* Preamble length. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 *     - byte 10/11: frame check-sum, automatically set by DW1000.  */
static uint8 tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0, 0};
/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 1000

/* Dummy buffer for DW1000 wake-up SPI read. See NOTE 2 below. */
#define DUMMY_BUFFER_LEN 600
static uint8 dummy_buffer[DUMMY_BUFFER_LEN];

/**
 * Application entry point.
 */
int dw_main(void)
{
    /* Display application name on LCD. */
    lcd_display_str(APP_NAME);

    /* Issue a wake-up in case DW1000 is asleep.
     * Since DW1000 is not woken by the reset line, we could get here with it asleep. Note that this may be true in other examples but we pay special
     * attention here because this example is precisely about sleeping. */
    dwt_spicswakeup(dummy_buffer, DUMMY_BUFFER_LEN);

    /* Reset and initialise DW1000. See NOTE 3 below.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    port_set_dw1000_slowrate();
    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
    {
        lcd_display_str("INIT FAILED");
        while (1)
        { };
    }
    port_set_dw1000_fastrate();

    /* Configure DW1000. See NOTE 4 below. */
    dwt_configure(&config);

    /* Configure sleep and wake-up parameters. */
    dwt_configuresleep(DWT_PRESRV_SLEEP | DWT_CONFIG, DWT_WAKE_CS | DWT_SLP_EN);

    /* Loop forever sending frames periodically. */
    while (1)
    {
        /* Write frame data to DW1000 and prepare transmission. See NOTE 5 below. */
        dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

        /* Start transmission. */
        dwt_starttx(DWT_START_TX_IMMEDIATE);

        /* Poll DW1000 until TX frame sent event set. See NOTE 6 below.
         * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
         * function to access it.*/
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
        { };

        /* Clear TX frame sent event. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

        /* Put DW1000 to sleep. */
        dwt_entersleep();

        /* Execute a delay between transmissions. */
        Sleep(TX_DELAY_MS);

        /* Wake DW1000 up. See NOTE 2 below. */
        dwt_spicswakeup(dummy_buffer, DUMMY_BUFFER_LEN);

        /* Increment the blink frame sequence number (modulo 256). */
        tx_msg[BLINK_FRAME_SN_IDX]++;
    }
}
#endif
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The device ID is a hard coded constant in the blink to keep the example simple but for a real product every device should have a unique ID.
 *    For development purposes it is possible to generate a DW1000 unique ID by combining the Lot ID & Part Number values programmed into the
 *    DW1000 during its manufacture. However there is no guarantee this will not conflict with someone else�s implementation. We recommended that
 *    customers buy a block of addresses from the IEEE Registration Authority for their production items. See "EUI" in the DW1000 User Manual.
 * 2. The chosen method for waking the DW1000 up here is by maintaining SPI chip select line low for at least 500 us. This means that we need a buffer
 *    to collect the data that DW1000 outputs during this dummy SPI transaction. The length of the transaction, and then the time for which the SPI
 *    chip select is held low, is determined by the buffer length given to dwt_spicswakeup() so this length must be chosen high enough so that the
 *    DW1000 has enough time to wake up.
 * 3. In this example, LDE microcode is not loaded upon calling dwt_initialise(). This will prevent the IC from generating an RX timestamp. If
 *    time-stamping is required, DWT_LOADUCODE parameter should be used. See two-way ranging examples (e.g. examples 5a/5b).
 * 4. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 5. dwt_writetxdata() takes the full size of tx_msg as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our tx_msg could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 6. We use polled mode of operation here to keep the example as simple as possible but the TXFRS status event can be used to generate an interrupt.
 *    Please refer to DW1000 User Manual for more details on "interrupts".
 * 7. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *    DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/
