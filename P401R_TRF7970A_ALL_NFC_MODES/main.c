//*****************************************************************************
//
// main.c
//
//XRTIC Gripper Example Code
//Authors: Matthew Bocharnikov, Zhiheng Luo, Cody Luong, Amarchand Niranjan, William Ogle
//Date Created: 5/4/2021
//Description: Demonstrates NFC Tag Detection with the RFID TRF7907A Sensor
//
//*****************************************************************************
#include <driverlib.h>
#include "nfc_controller.h"

/* Standard Includes */
#include <stdint.h>

#include <stdbool.h>

#ifdef MSP432P401R_LAUNCHPAD_ENABLED
#include "lp_buttons.h"
#endif

#define DOGS102x6_DRAW_NORMAL 0x00
#define DOGS102x6_DRAW_INVERT 0x01

#define UART_BASE               EUSCI_A0_BASE

#include "tag_header.h"

//
// Buffer to store incoming packets from NFC host
//
uint8_t g_ui8SerialBuffer[265];

//
// Number of bytes received from the host
//
volatile uint16_t g_ui16BytesReceived = 0x00;

bool g_bEnableAutoSDD;
bool g_bExtAmplifier;
bool g_bTRF5VSupply;
tTRF79x0_Version g_eTRFVersion;
bool g_bSupportCertification;
uint16_t g_ui16ListenTime;

//////////////////////////////////////////////////
tT5TStateMachine g_eT5TState;

static uint16_t g_ui16T5TBlockNumber;
static uint16_t g_ui16T5TBlockCount;
static uint8_t g_pui8T5TRxBuffer[30];

bool g_bT5TWaitForRsp;

uint8_t * g_pui8T5TBuffer;

uint16_t g_ui16T5TNdefLen;

uint16_t g_ui16T5TMaxNdefLen;

uint16_t g_ui16T5TNdefIndex;

uint16_t g_ui16T5TTLVRemaining;

uint8_t g_ui8T5TCurrentTlv;

uint16_t g_ui16T5TSize;

bool g_bT5TTLVSelected;

bool g_bT5TTLVLengthKnown;

uint8_t g_ui8T5TTLVLengthRemainBytes;

uint16_t g_ui16T5TReadIndex;

bool g_bT5TFormatting;

//////////////////////////////////////////////////

#if NFC_READER_WRITER_ENABLED
    t_sNfcRWMode g_sRWSupportedModes;
    t_sNfcRWCommBitrate g_sRWSupportedBitrates;
    t_sIsoDEP_RWSetup g_sRWSetupOptions;
    uint8_t g_ui8IsoDepInitiatorDID;
#endif


void NFC_configuration(void);
void Serial_processCommand(void);

void turnOn_LaunchpadLED1();
void turnOff_LaunchpadLED1();
void initialize_LaunchpadLED1();
void toggle_LaunchpadLED1();

void initialize_LaunchpadLED2_green();
void turnOn_LaunchpadLED2_green();
void turnOff_LaunchpadLED2_green();
void toggle_LaunchpadLED2_green();

void transmitString(uint8_t*);

uint8_t g_ui8TxBuffer[256];
uint8_t g_ui8TxLength;

void NFC_initIDs(void)
{
    // NFC ID's
    uint8_t pui8NfcAId[10] = {0x08,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};   // Generic ISO14443 T4TA Tag
    uint8_t pui8NfcBId[4] = {0x08,0x0A, 0xBE,0xEF}; // Generic ISO14443 T4TB Tag
    uint8_t pui8NfcFId[8] = {0x01,0xFE,0x88,0x77,0x66,0x55,0x44,0x33};  // Type F ID for P2P

    // Set the NFC Id's for Type A, Type B, and Type F
    NFC_A_setNfcAId(pui8NfcAId,4);
    NFC_B_setNfcBId(pui8NfcBId,4);
    NFC_F_setNfcId2(pui8NfcFId,8);
}
const eUSCI_UART_Config uartConfig2 =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        0x68,                                    // BRDIV = 208
        0,                                       // UCxBRF = 0
        2,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // MSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION  // Low Frequency Mode
};

//Initializing MSP432 Launchpad LED's
void initialize_LaunchpadLED1()
{
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
}

void turnOn_LaunchpadLED1()
{
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
}

void turnOff_LaunchpadLED1()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
}

void toggle_LaunchpadLED1()
{
    GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
}

void initialize_LaunchpadLED2_green()
{
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
}
void turnOn_LaunchpadLED2_green()
{
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
}
void turnOff_LaunchpadLED2_green()
{
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
}
void toggle_LaunchpadLED2_green()
{
    GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
}

void main(void)
{
    tNfcState eTempNFCState;
    tNfcState eCurrentNFCState;

    // CE Variables
    t_sNfcCEMode sCEMode;

    t_sNfcP2PMode sP2PMode;
    t_sNfcP2PCommBitrate sP2PBitrate;

    // Reader/Writer RX Status
    t_sNfcRWMode sRWMode;
    t_sNfcRWCommBitrate sRWBitrate;

    // Initialize MCU
    MCU_init();

    // Initialize Debug Pins as output
    NFC_RF_FIELD_LED_DIR |= NFC_RF_FIELD_LED_BIT;
    NFC_HOST_LED_DIR |= NFC_HOST_LED_BIT;

    NFC_RW_LED_PDIR |= NFC_RW_LED_BIT;
    NFC_P2P_LED_PDIR |= NFC_P2P_LED_BIT;
    NFC_CE_LED_PDIR |= NFC_CE_LED_BIT;

    // Clear NFC pins
    NFC_RF_FIELD_LED_POUT &= ~NFC_RF_FIELD_LED_BIT;
    NFC_HOST_LED_POUT &= ~NFC_HOST_LED_BIT;

    NFC_RW_LED_POUT &= ~NFC_RW_LED_BIT;
    NFC_P2P_LED_POUT &= ~NFC_P2P_LED_BIT;
    NFC_CE_LED_POUT &= ~NFC_CE_LED_BIT;

    //Enable interrupts globally
    __enable_interrupt();

    // Initialize USB Communication
    Serial_init();

    // Initialize TRF7970
    TRF79x0_init();

    Buttons_init(BUTTON_ALL);
    Buttons_interruptEnable(BUTTON_ALL);

    TRF79x0_idleMode();

    // Initialize the NFC Controller
    NFC_init();

    // This function will configure all the settings for each protocol
    NFC_configuration();

    // Initialize Type 4 Tag RTD Message
    T4T_CE_initNDEF();

    // Initialize IDs for NFC-A, NFC-B and NFC-F
    NFC_initIDs();

#if NFC_READER_WRITER_ENABLED
    // Initialize the RW T2T, T3T, T4T and T5 state machines
    T2T_init(g_ui8TxBuffer,256);
    T3T_init(g_ui8TxBuffer,256);
    T4T_init(g_ui8TxBuffer,256);
    T5T_init(g_ui8TxBuffer,256);
#endif

    uint8_t count = 2;
    bool tag_present = false;

    while(1)
    {
        eTempNFCState = NFC_run();

        if(eTempNFCState == NFC_DATA_EXCHANGE_PROTOCOL)
        {
            if(count != 2)
            {
                count = 2;
            }
            if(NFC_RW_getModeStatus(&sRWMode,&sRWBitrate))
            {
#if NFC_READER_WRITER_ENABLED
                NFC_RW_LED_POUT |= NFC_RW_LED_BIT;

                if( sRWMode.bits.bNfcA == 1)
                {
                    if(NFC_A_getSAK() == 0x00)
                    {
                        // T2T Tag Detection
                        if (tag_present == false){
                            turnOn_LaunchpadLED2_green();
                            tag_present = true;
                            transmitString(" Type 2 NFC tag detected \n\r");
                        }
                    }
                    else if(NFC_A_getSAK() & 0x20)
                    {
                        // T4T Tag State Machine
                        T4T_stateMachine();
                    }
                }
                else if(sRWMode.bits.bNfcB == 1)
                {
                    if(NFC_B_isISOCompliant())
                    {
                        // T4T Tag State Machine
                        T4T_stateMachine();
                    }
                }
                else if(sRWMode.bits.bNfcF == 1)
                {
                    // T3T Tag State Machine
                    T3T_stateMachine();
                }
                else if(sRWMode.bits.bISO15693 == 1)
                {
                    // T5T Tag State Machine
                    if (tag_present == false){
                        turnOn_LaunchpadLED1();
                        tag_present = true;
                        transmitString(" Type 5 NFC tag detected \n\r");

                    }
                    //T5T_stateMachine();
                }
#endif
            }
            else if(NFC_P2P_getModeStatus(&sP2PMode,&sP2PBitrate))
            {

            }
            else if(NFC_CE_getModeStatus(&sCEMode))
            {

            }
        }
        else
        {
            // Clear LEDs (RX & TX)
            if(count != 0)
                count--;

            if (count == 0)
            {
                turnOff_LaunchpadLED1();
                turnOff_LaunchpadLED2_green();


                NFC_RW_LED_POUT &= ~NFC_RW_LED_BIT;
                NFC_P2P_LED_POUT &= ~NFC_P2P_LED_BIT;
                NFC_CE_LED_POUT &= ~NFC_CE_LED_BIT;
                tag_present = false;
            }
        }

        // Update Current State if it has changed.
        if(eCurrentNFCState != eTempNFCState)
        {
            __no_operation();

            if(eCurrentNFCState != NFC_TARGET_WAIT_FOR_ACTIVATION
                && eCurrentNFCState != NFC_STATE_IDLE
                && (eTempNFCState == NFC_PROTOCOL_ACTIVATION
                    || eTempNFCState == NFC_DISABLED))
            {
                eCurrentNFCState = eTempNFCState;

#if NFC_READER_WRITER_ENABLED
                // Initialize the RW T2T, T3T, T4T and T5 state machines
                T2T_init(g_ui8TxBuffer,256);
                T3T_init(g_ui8TxBuffer,256);
                T4T_init(g_ui8TxBuffer,256);
                T5T_init(g_ui8TxBuffer,256);
#endif

                // Clear RW, P2P and CE LEDs
                NFC_RW_LED_POUT &= ~NFC_RW_LED_BIT;
                NFC_P2P_LED_POUT &= ~NFC_P2P_LED_BIT;
                NFC_CE_LED_POUT &= ~NFC_CE_LED_BIT;

                buttonDebounce = 1;

                Serial_printf("DC",NFC_MODE_LOST);
            }
            else
            {
                eCurrentNFCState = eTempNFCState;
            }

        }

        // Check if any packets have been received from the NFC host.
        if(g_ui16BytesReceived > 0)
        {
            Serial_processCommand();
        }
    }
}


void T5T_init(uint8_t * pui8Ndef, uint16_t ui16NdefMaxSize)
{
    g_eT5TState = T5T_INVENTORY;
    g_ui16T5TBlockNumber = 0;
    g_ui16T5TBlockCount = 0;
    g_bT5TWaitForRsp = false;

    g_pui8T5TBuffer = pui8Ndef;

    g_ui16T5TMaxNdefLen = ui16NdefMaxSize;

    g_ui16T5TNdefIndex = 0;

    g_ui16T5TNdefLen = 0;

    g_ui16T5TTLVRemaining = 0;

    g_ui8T5TCurrentTlv = 0;

    g_ui16T5TSize = 0;

    g_bT5TTLVSelected = false;

    g_bT5TTLVLengthKnown = false;

    g_ui8T5TTLVLengthRemainBytes = 0;

    g_ui16T5TReadIndex = 0;

    g_bT5TFormatting = false;
}

void T5T_writeNDEF(uint8_t * pui8Data, uint16_t ui16NdefLen)
{
    uint8_t ui8Offset;

    ui8Offset = 0;

    if(g_eT5TState == T5T_SELECTED_IDLE)
    {
        // Total Size of tag - 3 bytes (of TLV overhead Type, Length and Terminator TLV) - 4 bytes (Capability Container)
        if(g_ui16T5TSize > 7 && (g_ui16T5TSize-7) >= ui16NdefLen)
        {
            g_eT5TState = T5T_WRITE_NDEF;

            g_ui16T5TNdefIndex = 0;

            // Start Writing at Block 1
            g_ui16T5TBlockNumber = 0x01;

            // NDEF Msg TLV - Type
            g_pui8T5TBuffer[ui8Offset++] = TLV_NDEF_MSG;
            // NDEF Msg TLV - Length
            if(ui16NdefLen < 0xFF)
            {
                g_pui8T5TBuffer[ui8Offset++] = (uint8_t) ui16NdefLen;
            }

            g_ui16T5TTLVRemaining = (uint16_t) ui8Offset+ui16NdefLen+1;

            memcpy(&g_pui8T5TBuffer[ui8Offset],pui8Data,ui16NdefLen);

            g_pui8T5TBuffer[ui8Offset+ui16NdefLen] = TLV_TERMINATOR;
        }
        else
        {
            Serial_printf("T5T Write Fail: NDEF message size not supported by tag.\n",RW_STATUS_DATA);
        }

    }
    else
    {
        Serial_printf("T5T Write Fail: Busy.\n",RW_STATUS_DATA);
    }
}

void T5T_formatTag(void)
{
    uint8_t ui8TagBlockSize;

    Serial_printf("T5T Formatting...\n",RW_STATUS_DATA);

    ui8TagBlockSize = NFC_RW_T5T_getVICCBlockSize();

    if(g_eT5TState == T5T_SELECTED_IDLE)
    {
        g_eT5TState = T5T_WRITE_NDEF;

        g_bT5TFormatting = true;

        g_ui16T5TNdefIndex = 0;

        g_ui16T5TBlockNumber = 0;

        // Check for Extended Memory Tags
        if (g_ui16T5TBlockCount > 255)
        {
            g_ui16T5TTLVRemaining = 12;         // Need 2 Write Blocks to finish writing CC

            g_pui8T5TBuffer[0] = 0xE2;          // Tags with more than 256 blocks have Magic Number = 0xE2
            g_pui8T5TBuffer[1] = 0x40;          // NFC Major Version = 1, NFC Minor Version = 0, Read Access = Always, Write Access = Always
            g_pui8T5TBuffer[2] = 0x00;

            if (g_ui16T5TBlockCount == 0x0A)
            {
                g_pui8T5TBuffer[3] = 0x10;      // TI Tag-It Pro/Standard needs Option Flag
            }
            else
            {
                g_pui8T5TBuffer[3] = 0x00;
            }

            g_pui8T5TBuffer[4] = 0x00;          // RFU
            g_pui8T5TBuffer[5] = 0x00;          // RFU
            g_pui8T5TBuffer[6] = (uint8_t) ((g_ui16T5TBlockCount & 0xFF00) >> 8);
            g_pui8T5TBuffer[7] = (uint8_t) (g_ui16T5TBlockCount && 0x00FF);

            // Write an empty NDEF to tag
            g_pui8T5TBuffer[8] = 0x03;
            g_pui8T5TBuffer[9] = 0x00;
            g_pui8T5TBuffer[10] = 0xFE;
            g_pui8T5TBuffer[11] = 0x00;
        }
        else
        {
            g_ui16T5TTLVRemaining = 8;          // Need 1 Write Block for CC

            g_pui8T5TBuffer[0] = 0xE1;          // All other tags have Magic Number = 0xE1
            g_pui8T5TBuffer[1] = 0x40;          // NFC Major Version = 1, NFC Minor Version = 0, Read Access = Always, Write Access = Always

            if (g_ui16T5TBlockCount == 0x0A)
            {
                g_pui8T5TBuffer[2] = 0x04;
                g_pui8T5TBuffer[3] = 0x10;      // TI Tag-It Pro/Standard needs Option Flag
            }
            else
            {
                g_pui8T5TBuffer[2] = (uint8_t) (((g_ui16T5TBlockCount+1)*(ui8TagBlockSize+1))>>3);
                g_pui8T5TBuffer[3] = 0x00;
            }

            // Write an empty NDEF to tag
            g_pui8T5TBuffer[4] = 0x03;
            g_pui8T5TBuffer[5] = 0x00;
            g_pui8T5TBuffer[6] = 0xFE;
            g_pui8T5TBuffer[7] = 0x00;
        }
    }
    else
    {
        Serial_printf("T5T Write Fail: Busy./n",RW_STATUS_DATA);
    }
}

void T5T_stateMachine(void)
{
    uint8_t ui8RxLength;
    uint8_t * pui8RxData;
    tNfcRwT5TConnectionStatus eT5TStatus;
    char pui8LenBuffer[5];
    uint8_t ui8Temp;
    uint8_t ui8LocalReadIndex;

    // Waiting for a response
    if(g_bT5TWaitForRsp == true)
    {
        switch(g_eT5TState)
        {
        case T5T_INVENTORY:
            eT5TStatus = NFC_RW_T5T_getInventoryStatus();
            if(eT5TStatus == NFC_RW_T5T_CONNECTION_INVENTORY_SUCCESS)
            {
                g_eT5TState = T5T_GET_SYS_INFO_EXT;
            }
            else
            {
                g_eT5TState = T5T_SELECTED_IDLE;
            }
            g_bT5TWaitForRsp = false;
            break;
        case T5T_GET_SYS_INFO_EXT:
            eT5TStatus = NFC_RW_T5T_getGetSysInfoStatus();
            if(eT5TStatus == NFC_RW_T5T_CONNECTION_GET_SYS_INFO_SUCCESS)
            {
                NFC_RW_T5T_getPacketStatus(&pui8RxData,&ui8RxLength);

                memcpy(g_pui8T5TRxBuffer,pui8RxData,ui8RxLength);

                g_eT5TState = T5T_READ_CC;

                g_ui16T5TBlockCount = NFC_RW_T5T_getVICCBlockCount();
            }
            else if(eT5TStatus == NFC_RW_T5T_CONNECTION_GET_SYS_INFO_FAIL)
            {
                g_eT5TState = T5T_GET_SYS_INFO_1;
            }
            g_bT5TWaitForRsp = false;
            break;
        case T5T_GET_SYS_INFO_1:
            eT5TStatus = NFC_RW_T5T_getGetSysInfoStatus();
            if(eT5TStatus == NFC_RW_T5T_CONNECTION_GET_SYS_INFO_SUCCESS)
            {
                NFC_RW_T5T_getPacketStatus(&pui8RxData,&ui8RxLength);

                memcpy(g_pui8T5TRxBuffer,pui8RxData,ui8RxLength);

                g_eT5TState = T5T_READ_CC;

                g_ui16T5TBlockCount = NFC_RW_T5T_getVICCBlockCount();
            }
            else if(eT5TStatus == NFC_RW_T5T_CONNECTION_GET_SYS_INFO_FAIL)
            {
                g_eT5TState = T5T_GET_SYS_INFO_2;
            }
            g_bT5TWaitForRsp = false;
            break;
        case T5T_GET_SYS_INFO_2:
            eT5TStatus = NFC_RW_T5T_getGetSysInfoStatus();
            if(eT5TStatus == NFC_RW_T5T_CONNECTION_GET_SYS_INFO_SUCCESS)
            {
                g_eT5TState = T5T_READ_CC;

                g_ui16T5TBlockCount = NFC_RW_T5T_getVICCBlockCount();
            }
            else if(eT5TStatus == NFC_RW_T5T_CONNECTION_GET_SYS_INFO_FAIL)
            {
                g_eT5TState = T5T_READ_CC;

                g_ui16T5TBlockCount = 0x0A;
            }
            g_bT5TWaitForRsp = false;
            break;
        case T5T_READ_CC:
            eT5TStatus = NFC_RW_T5T_getReadSingleStatus();
            if(eT5TStatus == NFC_RW_T5T_CONNECTION_READ_SINGLE_SUCCESS)
            {
                NFC_RW_T5T_getPacketStatus(&pui8RxData,&ui8RxLength);

                memcpy(g_pui8T5TRxBuffer,pui8RxData+1,ui8RxLength-1);

                // Check if NDEF message formatted
                if(g_pui8T5TRxBuffer[0] == 0xE1)
                {
                    // Switch to the read NDEF data
                    g_eT5TState = T5T_READ_NDEF;

                    g_ui16T5TSize = g_pui8T5TRxBuffer[2] << 3;
                }
                else
                {
                    // Switch to the read NDEF data
                    g_eT5TState = T5T_READ_DATA;

                    Serial_printf("T5T Not NDEF Formatted",RW_STATUS_DATA);

                    // Print New Line
                    Serial_printf("\n",RW_STATUS_DATA);

                    Serial_printf("Block ",RW_STATUS_DATA);

                    // Print Block Number
                    convertWordToAscii(g_ui16T5TBlockNumber,(uint8_t *) pui8LenBuffer);

                    Serial_printf(pui8LenBuffer,RW_STATUS_DATA);

                    Serial_printf(" ",RW_STATUS_DATA);

                    for(ui8Temp = 0; ui8Temp < (ui8RxLength-1); ui8Temp++)
                    {
                        convertByteToAscii(g_pui8T5TRxBuffer[ui8Temp],(uint8_t *) pui8LenBuffer);

                        Serial_printf("0x",RW_STATUS_DATA);

                        Serial_printf(pui8LenBuffer,RW_STATUS_DATA);

                        Serial_printf(" ",RW_STATUS_DATA);
                    }

                    // Print New Line
                    Serial_printf("\n",RW_STATUS_DATA);

                    g_ui16T5TSize = 0;
                }

                // Incrment Block Number
                g_ui16T5TBlockNumber++;
            }
            else if(eT5TStatus == NFC_RW_T5T_CONNECTION_READ_SINGLE_FAIL)
            {
                g_eT5TState = T5T_SELECTED_IDLE;
            }
            g_bT5TWaitForRsp = false;
            break;
        case T5T_READ_DATA:
            eT5TStatus = NFC_RW_T5T_getReadSingleStatus();
            if(eT5TStatus == NFC_RW_T5T_CONNECTION_READ_SINGLE_SUCCESS)
            {
                NFC_RW_T5T_getPacketStatus(&pui8RxData,&ui8RxLength);

                memcpy(g_pui8T5TRxBuffer,pui8RxData+1,ui8RxLength-1);

                Serial_printf("Block ",RW_STATUS_DATA);

                // Print Block Number
                convertWordToAscii(g_ui16T5TBlockNumber,(uint8_t *) pui8LenBuffer);

                Serial_printf(pui8LenBuffer,RW_STATUS_DATA);

                Serial_printf(" ",RW_STATUS_DATA);

                for(ui8Temp = 0; ui8Temp < (ui8RxLength-1); ui8Temp++)
                {
                    convertByteToAscii(g_pui8T5TRxBuffer[ui8Temp],(uint8_t *) pui8LenBuffer);

                    Serial_printf("0x",RW_STATUS_DATA);

                    Serial_printf(pui8LenBuffer,RW_STATUS_DATA);

                    Serial_printf(" ",RW_STATUS_DATA);
                }

                // Print New Line
                Serial_printf("\n",RW_STATUS_DATA);

                g_ui16T5TBlockNumber++;

                if(g_ui16T5TBlockNumber > g_ui16T5TBlockCount)
                {
                    g_eT5TState = T5T_SELECTED_IDLE;
                }
            }
            else if(eT5TStatus == NFC_RW_T5T_CONNECTION_READ_SINGLE_FAIL)
            {
                g_eT5TState = T5T_SELECTED_IDLE;
            }
            g_bT5TWaitForRsp = false;
            break;
        case T5T_READ_NDEF:
            eT5TStatus = NFC_RW_T5T_getReadSingleStatus();
            if(eT5TStatus == NFC_RW_T5T_CONNECTION_READ_SINGLE_SUCCESS)
            {
                NFC_RW_T5T_getPacketStatus(&pui8RxData,&ui8RxLength);

                memcpy(g_pui8T5TRxBuffer,pui8RxData+1,ui8RxLength-1);

                ui8RxLength = ui8RxLength - 1;

                ui8LocalReadIndex = 0x00;

                // Processes all the bytes returned from read block #
                while(ui8LocalReadIndex < ui8RxLength && g_eT5TState == T5T_READ_NDEF)
                {

                    // TLV handling

                    //  No TLV is selected
                    if(g_bT5TTLVSelected == false)
                    {
                        // Read TLV
                        g_ui8T5TCurrentTlv =  g_pui8T5TRxBuffer[ui8LocalReadIndex++];

                        // NULL TLV just increment the read index
                        if(g_ui8T5TCurrentTlv == TLV_NULL)
                        {
                            // Do nothing
                        }
                        // TERMINATOR TLV we are done reading
                        else if(g_ui8T5TCurrentTlv == TLV_TERMINATOR)
                        {
                            g_eT5TState = T5T_SELECTED_IDLE;
                        }
                        else
                        {
                            // Go read length
                            g_bT5TTLVSelected = true;

                            g_bT5TTLVLengthKnown = false;
                            // Default
                            g_ui8T5TTLVLengthRemainBytes = 0xFF;
                        }

                    }
                    // TLV is selected and Length not known
                    else if(g_bT5TTLVLengthKnown == false)
                    {
                        // No length bytes have been read - (0xFF is default value)
                        if(g_ui8T5TTLVLengthRemainBytes == 0xFF)
                        {
                            // Large NDEF - NDEF length is stored in two subsequent bytes
                            if(g_pui8T5TRxBuffer[ui8LocalReadIndex] == 0xFF)
                            {
                                g_ui8T5TTLVLengthRemainBytes = 2;
                            }
                            else
                            {
                                g_bT5TTLVLengthKnown = true;
                                g_ui16T5TNdefLen = g_pui8T5TRxBuffer[ui8LocalReadIndex];
                                g_ui16T5TTLVRemaining = g_ui16T5TNdefLen;
                                g_ui16T5TReadIndex = 0;
                            }
                        }
                        // Read 1st length byte
                        else if(g_ui8T5TTLVLengthRemainBytes == 2)
                        {
                            g_ui16T5TNdefLen = (uint16_t) (g_pui8T5TRxBuffer[ui8LocalReadIndex] << 8);
                            g_ui8T5TTLVLengthRemainBytes = 1;
                        }
                        else if(g_ui8T5TTLVLengthRemainBytes == 1)
                        {
                            g_bT5TTLVLengthKnown = true;
                            g_ui16T5TNdefLen |= ((uint16_t) g_pui8T5TRxBuffer[ui8LocalReadIndex] & 0xFF);
                            g_ui16T5TTLVRemaining = g_ui16T5TNdefLen;
                            g_ui8T5TTLVLengthRemainBytes = 0;
                            g_ui16T5TReadIndex = 0;
                        }
                        else
                        {
                            // Do nothing
                        }
                        // Increment read index
                        ui8LocalReadIndex++;
                    }
                    // Parse the selected TLV data
                    else
                    {
                        if(g_ui8T5TCurrentTlv == TLV_NDEF_MSG)
                        {
                            // If the current TLV remaining bytes are more than the available bytes
                            if(g_ui16T5TTLVRemaining > (ui8RxLength - ui8LocalReadIndex))
                            {
//                              Serial_printBuffer((char *)&g_pui8T5TRxBuffer[ui8ReadIndex],(15 - ui8ReadIndex),RW_PAYLOAD_DATA);

                                // Store Incoming NDEF Message into g_pui8T5TBuffer
                                if(g_ui16T5TMaxNdefLen > (g_ui16T5TReadIndex + (ui8RxLength - ui8LocalReadIndex)))
                                {
                                    memcpy(&g_pui8T5TBuffer[g_ui16T5TReadIndex],&g_pui8T5TRxBuffer[ui8LocalReadIndex],(uint16_t) ui8RxLength - ui8LocalReadIndex);
                                }
                                else
                                {
                                    // NDEF message is larger than the size of the g_pui8T5TBuffer buffer
                                }

                                g_ui16T5TReadIndex += ui8RxLength - ui8LocalReadIndex;


                            }
                            else
                            {
                                // Store Incoming NDEF Message into g_pui8T5TBuffer
                                if(g_ui16T5TMaxNdefLen > (g_ui16T5TReadIndex + g_ui16T5TTLVRemaining))
                                {
                                    memcpy(&g_pui8T5TBuffer[g_ui16T5TReadIndex],&g_pui8T5TRxBuffer[ui8LocalReadIndex],g_ui16T5TTLVRemaining);

                                    Serial_printBuffer((char *)&g_pui8T5TBuffer[0],(uint8_t) (g_ui16T5TReadIndex+g_ui16T5TTLVRemaining),RW_PAYLOAD_DATA);
                                }
                                else
                                {
                                    // NDEF message is larger than the size of the g_pui8T5TBuffer buffer
                                }

                                g_ui16T5TReadIndex += g_ui16T5TTLVRemaining;
                            }
                        }

                        // Incrementing Index

                        // If the current TLV remaining bytes are more than the available bytes
                        if(g_ui16T5TTLVRemaining > (ui8RxLength - ui8LocalReadIndex))
                        {
                            // Update Remaining
                            g_ui16T5TTLVRemaining -= (uint16_t) ( ui8RxLength - ui8LocalReadIndex);
                            // Increase Index to 16 to continue reading next block
                            ui8LocalReadIndex = ui8RxLength;
                        }
                        else
                        {
                            // Increase read index by remaining number of bytes
                            ui8LocalReadIndex += (uint8_t) g_ui16T5TTLVRemaining;
                            g_ui16T5TTLVRemaining = 0;
                            // Finished reading TLV
                            g_bT5TTLVSelected = false;
                        }
                    }
                }

                if(g_eT5TState != T5T_SELECTED_IDLE)
                {
                    // Incrment Block Number
                    g_ui16T5TBlockNumber++;

                    // EOF Not Found
                    if(g_ui16T5TBlockNumber > g_ui16T5TBlockCount)
                    {
                        Serial_printf("Error: T5T Terminator TLV not found.",RW_STATUS_DATA);

                        // Print New Line
                        Serial_printf("\n",RW_STATUS_DATA);

                        g_eT5TState = T5T_SELECTED_IDLE;
                    }
                }
            }
            else if(eT5TStatus == NFC_RW_T5T_CONNECTION_READ_SINGLE_FAIL)
            {
                g_eT5TState = T5T_SELECTED_IDLE;
            }
            g_bT5TWaitForRsp = false;
            break;
        case T5T_WRITE_NDEF:
            eT5TStatus = NFC_RW_T5T_getWriteSingleStatus();

            if(eT5TStatus == NFC_RW_T5T_CONNECTION_WRITE_SINGLE_SUCCESS)
            {
                g_ui16T5TBlockNumber++;
                g_ui16T5TNdefIndex = g_ui16T5TNdefIndex + 4;
                if(g_ui16T5TTLVRemaining > 4)
                {
                    g_ui16T5TTLVRemaining = g_ui16T5TTLVRemaining - 4;
                }
                else
                {
                    g_ui16T5TTLVRemaining = 0;
                    g_eT5TState = T5T_SELECTED_IDLE;

                    Serial_printf("T5T Write Successful!",RW_STATUS_DATA);

                    // Print New Line
                    Serial_printf("\n",RW_STATUS_DATA);

                    if (g_bT5TFormatting)
                    {
                        Serial_printf("Re-present the Tag to Write to it.",RW_STATUS_DATA);

                        // Print New Line
                        Serial_printf("\n",RW_STATUS_DATA);
                    }
                }
            }
            else if(eT5TStatus == NFC_RW_T5T_CONNECTION_WRITE_SINGLE_FAIL)
            {
                if (NFC_RW_T5T_getT5TErrorCode() == 0x12)
                {
                    Serial_printf("T5T Write Fail! Target Block Is Locked",RW_STATUS_DATA);
                }

                g_eT5TState = T5T_SELECTED_IDLE;
            }
            g_bT5TWaitForRsp = false;
            break;
        case T5T_SELECTED_IDLE:
            g_bT5TWaitForRsp = false;
            break;
        }
    }

    // Sending a command
    if(g_bT5TWaitForRsp == false)
    {
        switch(g_eT5TState)
        {
        case T5T_INVENTORY:
            if(NFC_RW_T5T_sendInventoryCmd(0x26,0x00,false) == STATUS_SUCCESS)
            {
                g_bT5TWaitForRsp = true;
            }
            break;
        case T5T_GET_SYS_INFO_EXT:
            if(NFC_RW_T5T_sendGetSysInfoCmd(0x02,0x3B) == STATUS_SUCCESS)
            {
                g_bT5TWaitForRsp = true;
            }
            break;
        case T5T_GET_SYS_INFO_1:
            if(NFC_RW_T5T_sendGetSysInfoCmd(0x0A,0x2B) == STATUS_SUCCESS)
            {
                g_bT5TWaitForRsp = true;
            }
            break;
        case T5T_GET_SYS_INFO_2:
            if(NFC_RW_T5T_sendGetSysInfoCmd(0x02,0x2B) == STATUS_SUCCESS)
            {
                g_bT5TWaitForRsp = true;
            }
            break;
        case T5T_READ_CC:
            if(NFC_RW_T5T_sendReadSingleCmd(0x22,g_ui16T5TBlockNumber) == STATUS_SUCCESS)
            {
                g_bT5TWaitForRsp = true;
            }
            break;
        case T5T_READ_DATA:
            if(NFC_RW_T5T_sendReadSingleCmd(0x22,g_ui16T5TBlockNumber) == STATUS_SUCCESS)
            {
                g_bT5TWaitForRsp = true;
            }
            break;
        case T5T_READ_NDEF:
            if(NFC_RW_T5T_sendReadSingleCmd(0x22,g_ui16T5TBlockNumber) == STATUS_SUCCESS)
            {
                g_bT5TWaitForRsp = true;
            }
            break;
        case T5T_WRITE_NDEF:
            if(g_ui16T5TTLVRemaining > 0)
            {
                if(NFC_RW_T5T_sendWriteSingleCmd(T5T_REQ_FLAG_HIGH_DATA|T5T_REQ_FLAG_OPTION,g_ui16T5TBlockNumber,&g_pui8T5TBuffer[g_ui16T5TNdefIndex],4) == STATUS_SUCCESS)
                {
                    g_bT5TWaitForRsp = true;
                }
                else
                {
                    g_eT5TState = T5T_SELECTED_IDLE;
                }
            }
            break;
        case T5T_SELECTED_IDLE:
            break;
        }
    }
}

//*****************************************************************************
//
//! NFC_configuration - Handles the initial NFC configuration.
//!
//! Setup all NFC Mode Parameters.
//!
//! Current modes enabled: Card Emulation
//! Current modes supported: Card Emulation and Peer 2 Peer
//! Reader/Writer is NOT supported yet.
//!
//*****************************************************************************

void NFC_configuration(void)
{
#if NFC_READER_WRITER_ENABLED
    g_sRWSupportedModes.ui8byte = 0x00;
    g_sRWSupportedBitrates.ui16byte = 0x0000;
    g_sRWSetupOptions.ui16byte = 0x0000;
#endif

    // Set the TRF7970 Version being used
    g_eTRFVersion = TRF7970_A;

    // External Amplifer (disconnected by default)
    g_bExtAmplifier = false;

    // Configure TRF External Amplifier for the transceiver
    TRF79x0_setExtAmplifer(g_bExtAmplifier);

    // Configure TRF Power Supply (5V = true, 3V = false)

    g_bTRF5VSupply = false;


    // Configure TRF Power Supply
    TRF79x0_setPowerSupply(g_bTRF5VSupply);

    // Milliseconds the NFC stack will be in listen mode
    g_ui16ListenTime = 10;

    // Set the time the NFC stack will be with the RF field disabled (listen mode)
    NFC_setListenTime(g_ui16ListenTime);

    // Enable (1) or disable (0) the Auto SDD Anti-collision function of the TRF7970A
    g_bEnableAutoSDD = 0;

#if NFC_READER_WRITER_ENABLED
    // Enable Reader Writer Supported Modes
    g_sRWSupportedModes.bits.bNfcA = 1;
    g_sRWSupportedModes.bits.bNfcB = 1;
    g_sRWSupportedModes.bits.bNfcF = 1;
    g_sRWSupportedModes.bits.bISO15693 = 1;

    // NFC-A Bitrates
    g_sRWSupportedBitrates.bits.bNfcA_106kbps = 1;      // Must be enabled if bNfcA is set
    g_sRWSupportedBitrates.bits.bNfcA_212kbps = 0;
    g_sRWSupportedBitrates.bits.bNfcA_424kbps = 0;
    g_sRWSupportedBitrates.bits.bNfcA_848kbps = 1;
    // NFC-B Bitrates
    g_sRWSupportedBitrates.bits.bNfcB_106kbps = 1;      // Must be enabled if bNfcB is set
    g_sRWSupportedBitrates.bits.bNfcB_212kbps = 0;
    g_sRWSupportedBitrates.bits.bNfcB_424kbps = 0;
    g_sRWSupportedBitrates.bits.bNfcB_848kbps = 1;
    // NFC-F Bitrates
    g_sRWSupportedBitrates.bits.bNfcF_212kbps = 1;
    g_sRWSupportedBitrates.bits.bNfcF_424kbps = 1;
    // ISO15693 Bitrates
    g_sRWSupportedBitrates.bits.bISO15693_26_48kbps = 1;

    // Default Max number of WTX 2
    g_sRWSetupOptions.bits.ui3RWMaxWTX = 2;
    // Default Max number of ACK 2
    g_sRWSetupOptions.bits.ui3RWMaxACK = 2;
    // Default Max number of NACK 2
    g_sRWSetupOptions.bits.ui3RWMaxNACK = 2;
    // Default Max number of DSL 2
    g_sRWSetupOptions.bits.ui3RWMaxDSL = 2;

    g_ui8IsoDepInitiatorDID = 0x00;
#endif



#if NFC_READER_WRITER_ENABLED
    if (g_sRWSupportedModes.ui8byte != 0x00)
    {
        // To be made shortly
        NFC_RW_configure(g_sRWSupportedModes,g_sRWSupportedBitrates);
    }
    ISODEP_configure_RW(g_sRWSetupOptions,g_ui8IsoDepInitiatorDID);
#endif

    // Set the Auto SDD flag within nfc_a.c
    NFC_A_setAutoSDD(g_bEnableAutoSDD);

    // Set the current TRF version within trf79x0.c
    TRF79x0_setVersion(g_eTRFVersion);

    // Set Certification Support for all Protocols - Required for NFC Forum Certification
    NFC_setSupportCertification(g_bSupportCertification);

    // Set Test Enable flag within trf79x0.c - Required for NFC Forum Certification
    TRF79x0_testFlag(g_bSupportCertification);

}

//*****************************************************************************
//
//! Serial_processCommand - Process incoming commands from NFC host.
//!
//! Checks for the SOF (0xFE) from the host, and processes the commands. The
//! possible commands are:
//!     COMMUNICATION_START - NFC Host connected
//!     START_P2P_CMD - Enable the P2P modes included in the command
//!     STOP_P2P_CMD - Disable the P2P stack.
//!     ACK_CMD - Not currently used
//!     NDEF_PAYLOAD - NDEF data to send via the P2P stack.
//!     COMMUNICATION_END - Disconnect form the NFC Host.
//!
//! \return None.
//
//*****************************************************************************
void Serial_processCommand(void)
{

    tNFCControllerCommands eHostCommand;
#if NFC_READER_WRITER_ENABLED
    t_sNfcRWMode sRWMode;
    t_sNfcRWCommBitrate sRWBitrate;
#endif
    uint8_t ui8CurrentConfiguration[19];

    // When SOF and length are received, but still missing data
    if(g_ui16BytesReceived > 2 && ((g_ui8SerialBuffer[2] + 3) > g_ui16BytesReceived))
    {
        // Wait until full packet has been received
    }
    // Waiting for Length Byte
    else if(g_ui16BytesReceived < 3)
    {
        // Wait until Length Byte received
    }
    else if(g_ui8SerialBuffer[2] + 3 == g_ui16BytesReceived)    // Length
    {
        eHostCommand = (tNFCControllerCommands) g_ui8SerialBuffer[1];
        if(eHostCommand == COMM_START && g_ui16BytesReceived == 3)
        {
            // Initialize the Current Configuration variables to zero
            //
            memset(ui8CurrentConfiguration,0x00,19);

            // Turn on LED
            g_bSerialConnectionEstablished = true;
            NFC_HOST_LED_POUT |= NFC_HOST_LED_BIT;
            Serial_printf(NFC_FW_VERSION,FW_VERSION_CMD);

#if NFC_READER_WRITER_ENABLED
            ui8CurrentConfiguration[0] |= READER_WRITER_FW_ENABLED;
            ui8CurrentConfiguration[4] |= g_sRWSupportedModes.ui8byte;
            ui8CurrentConfiguration[5] |= (uint8_t) (g_sRWSupportedBitrates.ui16byte & 0xFF);
            ui8CurrentConfiguration[6] |= (uint8_t) ((g_sRWSupportedBitrates.ui16byte >> 8) & 0xFF);
#endif

            ui8CurrentConfiguration[7] = (uint8_t) (g_ui16ListenTime & 0xFF);
            ui8CurrentConfiguration[8] = (uint8_t) ((g_ui16ListenTime >> 8) & 0xFF);

            ui8CurrentConfiguration[9] = g_bSupportCertification;

#if NFC_READER_WRITER_ENABLED
            ui8CurrentConfiguration[12] = (uint8_t) (g_sRWSetupOptions.ui16byte & 0xFF);

            ui8CurrentConfiguration[13] = (uint8_t) ((g_sRWSetupOptions.ui16byte >> 8) & 0xFF);

            ui8CurrentConfiguration[14] = g_ui8IsoDepInitiatorDID;
#endif

            ui8CurrentConfiguration[15] = g_bEnableAutoSDD;

            ui8CurrentConfiguration[16] = g_bExtAmplifier;

            ui8CurrentConfiguration[17] = g_bTRF5VSupply;

            ui8CurrentConfiguration[18] = (uint8_t) g_eTRFVersion;

            Serial_printBuffer((char *)ui8CurrentConfiguration,19,NFC_CONFIGURATION);

        }
        else if(eHostCommand == COMM_END && g_ui16BytesReceived == 3)
        {
            // Turn off LED
            NFC_HOST_LED_POUT &= ~NFC_HOST_LED_BIT;
            g_bSerialConnectionEstablished = false;
        }
        else if(eHostCommand == NFC_TEST_CONFIG && g_ui16BytesReceived == 9)
        {
            // NFC Certification
            g_bSupportCertification = g_ui8SerialBuffer[3];

            // Set Certification Support for all Protocols - Required for NFC Forum Certification
            NFC_setSupportCertification(g_bSupportCertification);

            // Set Test Enable flag within trf79x0.c - Required for NFC Forum Certification
            TRF79x0_testFlag(g_bSupportCertification);


 #if NFC_READER_WRITER_ENABLED
            // RW Options
            g_sRWSetupOptions.ui16byte =  (uint16_t) (g_ui8SerialBuffer[6]) + ((uint16_t) g_ui8SerialBuffer[7] << 8);

            // ISO DEP DID
            g_ui8IsoDepInitiatorDID = g_ui8SerialBuffer[8];

            ISODEP_configure_RW(g_sRWSetupOptions,g_ui8IsoDepInitiatorDID);
#endif
        }
        else if(eHostCommand == TRF_SETTINGS && g_ui16BytesReceived == 9)
        {
            // TRF Version Number
            g_eTRFVersion = (tTRF79x0_Version) g_ui8SerialBuffer[3];

            // Set the current TRF version within trf79x0.c
            TRF79x0_setVersion(g_eTRFVersion);

            // Listen Time
            g_ui16ListenTime = (uint16_t) (g_ui8SerialBuffer[4]) + ((uint16_t) g_ui8SerialBuffer[5] << 8);

            // Set the time the NFC stack will be with the RF field disabled (listen mode)
            NFC_setListenTime(g_ui16ListenTime);

            // 5V Power Supply
            g_bTRF5VSupply = g_ui8SerialBuffer[6];

            // Configure TRF Power Supply
            TRF79x0_setPowerSupply(g_bTRF5VSupply);

            // External Amplifier
            g_bExtAmplifier = g_ui8SerialBuffer[7];

            // Configure TRF External Amplifier for the transceiver
            TRF79x0_setExtAmplifer(g_bExtAmplifier);

            // Auto-SDD Setting
            g_bEnableAutoSDD = g_ui8SerialBuffer[8];

            // Set the Auto SDD flag within nfc_a.c
            NFC_A_setAutoSDD(g_bEnableAutoSDD);
        }
#if (NFC_READER_WRITER_ENABLED)
        else if(eHostCommand == RW_START_CMD && g_ui16BytesReceived == 6)
        {
            g_sRWSupportedBitrates.ui16byte = (g_ui8SerialBuffer[4] << 8) + g_ui8SerialBuffer[3];
            g_sRWSupportedModes.ui8byte = g_ui8SerialBuffer[5];

            NFC_RW_configure(g_sRWSupportedModes,g_sRWSupportedBitrates);
        }
        else if(eHostCommand == RW_STOP_CMD && g_ui16BytesReceived == 3)
        {
            NFC_RW_disable();
            g_sRWSupportedModes.ui8byte = 0x00;
        }
        else if(eHostCommand == RW_WRITE_TAG)
        {
            if(NFC_RW_getModeStatus(&sRWMode,&sRWBitrate))
            {
                if( sRWMode.bits.bNfcA == 1)
                {
                    if(NFC_A_getSAK() == 0x00)
                    {
                        // T2T Tag Write State Machine
                        T2T_writeNDEF(&g_ui8SerialBuffer[3],(uint16_t) (g_ui16BytesReceived-3));
                    }
                    else if(NFC_A_getSAK() & 0x20)
                    {
                        // T4T Tag Write State Machine
                        T4T_writeNDEF(&g_ui8SerialBuffer[3],(uint16_t) (g_ui16BytesReceived-3));
                    }
                }
                else if(sRWMode.bits.bNfcB == 1)
                {
                    if(NFC_B_isISOCompliant())
                    {
                        // T4T Tag Write State Machine
                        T4T_writeNDEF(&g_ui8SerialBuffer[3],(uint16_t) (g_ui16BytesReceived-3));
                    }
                }
                else if(sRWMode.bits.bNfcF == 1)
                {
                    // T3T Tag State Machine
                    T3T_writeNDEF(&g_ui8SerialBuffer[3],(uint16_t) (g_ui16BytesReceived-3));
                }
                else if(sRWMode.bits.bISO15693 == 1)
                {
                    // T5T Tag State Machine
                    T5T_writeNDEF(&g_ui8SerialBuffer[3],(uint16_t) (g_ui16BytesReceived-3));
                }
            }
        }
        else if (eHostCommand == RW_FORMAT_TAG)
        {
            if(NFC_RW_getModeStatus(&sRWMode,&sRWBitrate))
            {
                if(sRWMode.bits.bISO15693 == 1)
                {
                    T5T_formatTag();
                }
                else
                {
                    Serial_printf("Cannot Format non-T5T Tags! \n",RW_STATUS_DATA);
                }
            }

        }
#endif
        g_ui16BytesReceived = 0;
    }
    else
    {
        // Partial Command Received
        g_ui16BytesReceived = 0x00;
    }
}

/*
* EUSCI A0 UART interrupt handler. Receives data from GUI and sets LED color/blink frequency
*/
void EusciA0_ISR(void)
{
   uint8_t ui8RxByte = UCA0RXBUF;

   g_ui8SerialBuffer[g_ui16BytesReceived] = ui8RxByte;

   // Check if we are receiving a packet
   if(g_ui16BytesReceived > 0)
   {
       g_ui16BytesReceived++;
   }
   // Check for Start of Frame
   else if(ui8RxByte == 0xFE)
   {
       g_ui16BytesReceived++;
   }
   else
   {
       // Do nothing
   }
}

void transmitString(uint8_t* s)
{
    uint8_t x = 0;
    while(s[x] != 0)
    {
        UART_transmitData(UART_BASE, s[x]);
        x++;
    }

}
