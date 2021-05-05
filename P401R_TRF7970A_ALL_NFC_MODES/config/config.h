//*****************************************************************************
//
// config.h - MCU Pin Configuration to interface with the TRF7970A
//
// Copyright (c) 2015 Texas Instruments Incorporated.  All rights reserved.
// TI Information - Selective Disclosure
//
//*****************************************************************************
#ifndef __CONFIG_H__
#define __CONFIG_H__

#ifdef      MSP432P401R_LAUNCHPAD_ENABLED

//
// TRF79790A Hardware Configuration
//
#define SPI_MODULE_BASE_ADDR    EUSCI_A3_MODULE


#define SPI_PORT                GPIO_PORT_P9
#define SPI_CLK                 GPIO_PIN5
#define SPI_MOSI                GPIO_PIN7
#define SPI_MISO                GPIO_PIN6
#define SPI_SS_PORT             GPIO_PORT_P9
#define SPI_SS_POUT             P9OUT
#define SPI_SS                  GPIO_PIN4
#define TRF_EN_PORT             GPIO_PORT_P7
#define TRF_EN                  GPIO_PIN0


//THIS IRQ PIN IS MISLEADING: IT IS THE XOUT, MOVE XOUT WIRE NOT IRQ
#define TRF_IRQ_PORT            GPIO_PORT_P6
#define TRF_IRQ_PIN             P6IN
#define TRF_IRQ                 GPIO_PIN0

//
// P2.0 - Displays if the external RF field is enabled.
//
#define NFC_RF_FIELD_LED_BIT            BIT0
#define NFC_RF_FIELD_LED_POUT           P2OUT
#define NFC_RF_FIELD_LED_DIR            P2DIR

//
// P1.0 - Displays if the NFC host is connected to the controller.
//
#define NFC_HOST_LED_BIT                BIT0
#define NFC_HOST_LED_POUT               P1OUT
#define NFC_HOST_LED_DIR                P1DIR

//
// P - Toggles during the RX of a NDEF packet via P2P.
//
#define NFC_RX_LED_BIT
#define NFC_RX_LED_POUT
#define NFC_RX_LED_PDIR

//
// P - Toggles during the TX of a NDEF packet via P2P.
//
#define NFC_TX_LED_BIT
#define NFC_TX_LED_POUT
#define NFC_TX_LED_PDIR

//
// LED1 P3.6 - Displays if the RW link is established.
//
#define NFC_RW_LED_BIT                  BIT1
#define NFC_RW_LED_POUT                 P7OUT
#define NFC_RW_LED_PDIR                 P7DIR

//
// LED2 P5.2 - Displays if the CE link is established.
//
#define NFC_P2P_LED_BIT                 BIT2
#define NFC_P2P_LED_POUT                P7OUT
#define NFC_P2P_LED_PDIR                P7DIR

//
// LED3 P5.0 - Displays if the P2P link is established.
//
#define NFC_CE_LED_BIT                  BIT3
#define NFC_CE_LED_POUT                 P7OUT
#define NFC_CE_LED_PDIR                 P7DIR

#endif

#endif
