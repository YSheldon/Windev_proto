/**
********************************************************************************
\file   main.c

\brief  Ndis minipost for Intel 83573V NIC

The file is the main entry point for the NDIS miniport driver prototype
designed to test performance of NDIS miniport driver.

\ingroup module_miniport
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2014 Kalycito Infotech Private Limited
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of the copyright holders nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/
//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include "ndisInc.h"

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------
#pragma NDIS_INIT_FUNCTION(DriverEntry)

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#ifndef EDRV_MAX_TX_BUFFERS
#define EDRV_MAX_TX_BUFFERS     42
#endif

#ifndef EDRV_MAX_TX_DESCS
#define EDRV_MAX_TX_DESCS       16
#define EDRV_TX_DESC_MASK       (EDRV_MAX_TX_DESCS - 1)
#endif

#ifndef EDRV_MAX_RX_BUFFERS
#define EDRV_MAX_RX_BUFFERS     32
#endif

#ifndef EDRV_MAX_RX_DESCS
#define EDRV_MAX_RX_DESCS       16
#define EDRV_RX_DESC_MASK       (EDRV_MAX_RX_DESCS - 1)
#endif

#define EDRV_MAX_FRAME_SIZE     0x600

#define EDRV_TX_BUFFER_SIZE     (EDRV_MAX_TX_BUFFERS * EDRV_MAX_FRAME_SIZE) // n * (MTU + 14 + 4)
#define EDRV_TX_DESCS_SIZE      (EDRV_MAX_TX_DESCS * sizeof (tEdrvTxDesc))


#define EDRV_RX_BUFFER_SIZE_SHIFT   11  // 2048 Byte
#define EDRV_RX_MAX_FRAME_SIZE      (1 << EDRV_RX_BUFFER_SIZE_SHIFT) 
#define EDRV_RX_BUFFER_SIZE         EDRV_MAX_RX_BUFFERS * EDRV_RX_MAX_FRAME_SIZE // TODO: Review this 
#define EDRV_RX_DESCS_SIZE          (EDRV_MAX_RX_DESCS * sizeof (tEdrvRxDesc))

#define EDRV_AUTO_READ_DONE_TIMEOUT 10  // ms
#define EDRV_MASTER_DISABLE_TIMEOUT 90  // ms
#define EDRV_LINK_UP_TIMEOUT        3000 // ms


// Tx descriptor definitions
#define EDRV_TX_DESC_CMD_RS     0x08000000  // Report Status
#define EDRV_TX_DESC_CMD_IFCS   0x02000000  // Insert Frame Check Sum
#define EDRV_TX_DESC_CMD_EOP    0x01000000  // End of Packet
#define EDRV_TX_DESC_CMD_DEF    (EDRV_TX_DESC_CMD_EOP \
    | EDRV_TX_DESC_CMD_IFCS \
    | EDRV_TX_DESC_CMD_RS)
#define EDRV_TX_DESC_STATUS_DD  0x01        // Descriptor Done
#define EDRV_TX_DESC_STATUS_EC  0x02        // Excess Collisions
#define EDRV_TX_DESC_STATUS_LC  0x04        // Late Collision


// Ethernet Controller Register Definitions
#define EDRV_REGDW_CTRL         0x00000     // Device Control
#define EDRV_REGDW_CTRL_FD      0x00000001  // Full-Duplex
#define EDRV_REGDW_CTRL_MST_DIS 0x00000004  // GIO Master Disable
#define EDRV_REGDW_CTRL_LRST    0x00000008  // Link Reset
#define EDRV_REGDW_CTRL_SLU     0x00000040  // Set Link Up
#define EDRV_REGDW_CTRL_RST     0x04000000  // Reset
#define EDRV_REGDW_CTRL_PHY_RST 0x80000000  // PHY Reset

#define EDRV_REGDW_CTRL_DEF     (EDRV_REGDW_CTRL_LRST \
    | EDRV_REGDW_CTRL_SLU)

#define EDRV_REGDW_STATUS       0x00008     // Device Status
#define EDRV_REGDW_STATUS_LU    0x00000002  // Link Up Indication
#define EDRV_REGDW_STATUS_MST_EN 0x00080000 // GIO Master Enable Status

#define EDRV_REGDW_EEC          0x00010     // EEPROM Control Register
#define EDRV_REGDW_EEC_AUTO_RD  0x00000200  // Auto Read Done

#define EDRV_REGDW_ICR          0x000C0     // Interrupt Cause Read
#define EDRV_REGDW_ITR          0x000C4     // Interrupt Throttling Rate
#define EDRV_REGDW_IMS          0x000D0     // Interrupt Mask Set/Read
#define EDRV_REGDW_IMC          0x000D8     // Interrupt Mask Clear
#define EDRV_REGDW_INT_MASK_ALL 0xFFFFFFFF  // mask all interrupts
#define EDRV_REGDW_INT_TXDW     0x00000001  // Transmit Descriptor Written Back
#define EDRV_REGDW_INT_TXQE     0x00000002  // Transmit Descriptor Queue Empty
#define EDRV_REGDW_INT_LSC      0x00000004  // Link Status Change
#define EDRV_REGDW_INT_RXSEQ    0x00000008  // Receive Sequence Error
#define EDRV_REGDW_INT_RXT0     0x00000080  // Receiver Timer Interrupt
#define EDRV_REGDW_INT_RXDMT0   0x00000010  // Receive Descriptor Minimum Threshold Reached
#define EDRV_REGDW_INT_RXO      0x00000040  // Receiver Overrun
#define EDRV_REGDW_INT_TXD_LOW  0x00008000  // Transmit Descriptor Low Threshold hit
#define EDRV_REGDW_INT_SRPD     0x00010000  // Small Receive Packet Detected
#define EDRV_REGDW_INT_INT_ASSERTED 0x80000000  // PCIe Int. has been asserted

#define EDRV_REGDW_INT_MASK_DEF (EDRV_REGDW_INT_TXDW \
    | EDRV_REGDW_INT_RXT0 \
    | EDRV_REGDW_INT_RXDMT0 \
    | EDRV_REGDW_INT_RXO \
    | EDRV_REGDW_INT_RXSEQ)

#define EDRV_REGDW_TIPG         0x00410     // Transmit Inter Packet Gap
#define EDRV_REGDW_TIPG_DEF     0x00702008  // default according to Intel PCIe GbE Controllers Open Source Software Developer's Manual

#define EDRV_REGDW_RDTR         0x02820     // Receive Interrupt Packet Delay Timer
#define EDRV_REGDW_RADV         0x0282C     // Receive Interrupt Absolute Delay Timer

#define EDRV_REGDW_TXDCTL       0x03828     // Transmit Descriptor Control
#define EDRV_REGDW_TXDCTL_GRAN  0x01000000  // Granularity (1=Descriptor, 0=Cache line)
//#define EDRV_REGDW_TXDCTL_WTHRESH 0x01000000  // Write Back Threshold
#define EDRV_REGDW_TXDCTL_DEF   (EDRV_REGDW_TXDCTL_GRAN)

#define EDRV_REGDW_RXDCTL       0x02828     // Receive Descriptor Control
#define EDRV_REGDW_RXDCTL_GRAN  0x01000000  // Granularity (1=Descriptor, 0=Cache line)
#define EDRV_REGDW_RXDCTL_DEF   (EDRV_REGDW_RXDCTL_GRAN)

#define EDRV_REGDW_TCTL         0x00400     // Transmit Control
#define EDRV_REGDW_TCTL_EN      0x00000002  // Transmit Enable
#define EDRV_REGDW_TCTL_PSP     0x00000008  // Pad Short Packets
#define EDRV_REGDW_TCTL_CT      0x000000F0  // Collision Threshold
#define EDRV_REGDW_TCTL_COLD    0x0003F000  // Collision Distance
#define EDRV_REGDW_TCTL_DEF     (EDRV_REGDW_TCTL_EN \
    | EDRV_REGDW_TCTL_PSP \
    | EDRV_REGDW_TCTL_CT \
    | EDRV_REGDW_TCTL_COLD)

#define EDRV_REGDW_RCTL         0x00100     // Receive Control
#define EDRV_REGDW_RCTL_EN      0x00000002  // Receive Enable
#define EDRV_REGDW_RCTL_SBP     0x00000004  // Store Bad Packets (to recognize collisions)
#define EDRV_REGDW_RCTL_BAM     0x00008000  // Broadcast Accept Mode
#define EDRV_REGDW_RCTL_SECRC   0x04000000  // Strip Ethernet CRC (do not store in host memory)
#define EDRV_REGDW_RCTL_UPE     (1 << 3)    // Unicast Promisious mode
#define EDRV_REGDW_RCTL_MPE     (1 << 4)    // Multicast Promisious mode
#define EDRV_REGDW_RCTL_BSIZE_2048  0x00000000  // buffer size is 2048 byte

// TODO: Remove Promisious enable bits
#define EDRV_REGDW_RCTL_DEF     (EDRV_REGDW_RCTL_EN \
    | EDRV_REGDW_RCTL_SBP \
    | EDRV_REGDW_RCTL_BAM \
    | EDRV_REGDW_RCTL_SECRC \
    | EDRV_REGDW_RCTL_BSIZE_2048 \
    | EDRV_REGDW_RCTL_UPE \
    | EDRV_REGDW_RCTL_MPE)

#define EDRV_REGDW_TDBAL        0x03800     // Transmit Descriptor Base Address Low
#define EDRV_REGDW_TDBAH        0x03804     // Transmit Descriptor Base Address High
#define EDRV_REGDW_TDLEN        0x03808     // Transmit Descriptor Length
#define EDRV_REGDW_TDH          0x03810     // Transmit Descriptor Head
#define EDRV_REGDW_TDT          0x03818     // Transmit Descriptor Tail

#define EDRV_REGDW_RDBAL0       0x02800     // Receive Descriptor Base Address Low
#define EDRV_REGDW_RDBAH0       0x02804     // Receive Descriptor Base Address High
#define EDRV_REGDW_RDLEN0       0x02808     // Receive Descriptor Length
#define EDRV_REGDW_RDH0         0x02810     // Receive Descriptor Head
#define EDRV_REGDW_RDT0         0x02818     // Receive Descriptor Tail

#define EDRV_REGDW_MTA(n)       (0x05200 + 4*n)  // Multicast Table Array

#define EDRV_REGDW_RAL(n)       (0x05400 + 8*n)  // Receive Address Low
#define EDRV_REGDW_RAH(n)       (0x05404 + 8*n)  // Receive Address HIGH
#define EDRV_REGDW_RAH_AV       0x80000000  // Receive Address Valid

#define EDRV_REGDW_CRCERRS      0x04000     // CRC Error Count
#define EDRV_REGDW_LATECOL      0x04020     // Late Collision Count
#define EDRV_REGDW_COLC         0x04028     // Collision Count
#define EDRV_REGDW_SEC          0x04038     // Sequence Error Count
#define EDRV_REGDW_RLEC         0x04040     // Receive Length Error Count

#define EDRV_REGDW_SWSM         0x05B50     // Software Semaphore
#define EDRV_REGDW_SWSM_SWESMBI 0x00000002  // Software EEPROM Semaphore Bit

// defines for the status byte in the receive descriptor
#define EDRV_RXSTAT_DD          0x01        // Descriptor Done (Processed by Hardware)
#define EDRV_RXSTAT_EOP         0x02        // End of Packet
#define EDRV_RXERR_CE           0x01        // CRC Error
#define EDRV_RXERR_SEQ          0x04        // Sequence Error
#define EDRV_RXERR_OTHER        0xE2        // Other Error


#define EDRV_REGDW_WRITE(reg, val)      WRITE_REGISTER_ULONG((PULONG)((ULONG_PTR)instance_l.pIoAddr + reg), val)
#define EDRV_REGDW_READ(reg)            READ_REGISTER_ULONG((PULONG)((ULONG_PTR)instance_l.pIoAddr + reg))


#define EDRV_SAMPLE_NUM         10000

#define DATA_LENGTH             1000
//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    UINT64  bufferAddr_le;
    UINT32  lengthCmd_le;
    UINT32  status_le;
} tEdrvTxDesc;

typedef struct
{
    UINT64  bufferAddr_le;
    UINT16  length_le;
    UINT16  checksum_le;
    UINT8   status;
    UINT8   error;
    UINT16  reserved_le;
} tEdrvRxDesc;

typedef struct
{
    UINT8*                  rxBuffVa;
    NDIS_PHYSICAL_ADDRESS   rxBuffPa;
}tRxUsedBuff;
// Private structure
typedef struct
{
    NDIS_HANDLE             pDriverHandle;       ///< From NdisMRegisterMiniportDriver
    NDIS_HANDLE             pDeviceHandle;       ///< From NdisMRegisterDeviceEx
    NDIS_HANDLE             pAdapterHandle;      ///< Miniport adapter handle
    NDIS_HANDLE             pInterruptHandle;    ///< Interrupt handle
    void*                   pIoAddr;             ///< Pointer to register space of Ethernet controller
    UINT                    ioLength;

    tEdrvRxDesc*            pRxDesc;             ///< Pointer to Rx descriptors
    tRxUsedBuff             aRxBuffUsed[EDRV_MAX_RX_DESCS];
    NDIS_PHYSICAL_ADDRESS   pRxDescDma;          ///< Dma pointer to Rx descriptors
    void*                   pRxBuf;              ///< Pointer to Rx buffer
    NDIS_PHYSICAL_ADDRESS   pRxBufDma;              ///< Pointer to Rx buffer
    INT                     rxBufFreeTop;
    PNDIS_SPIN_LOCK*        pSpinLockRxBufRelease;
    INT                     pageAllocations;

    void*                   pTxBuf;      // pointer to Tx buffer
    NDIS_PHYSICAL_ADDRESS        pTxBufDma;
    tEdrvTxDesc*            pTxDesc;      // pointer to Tx descriptors
//    tEdrvTxBuffer*      apTxBuffer[EDRV_MAX_TX_DESCS];
    NDIS_PHYSICAL_ADDRESS        pTxDescDma;
    BOOLEAN                 afTxBufUsed[EDRV_MAX_TX_BUFFERS];

    UINT                    headTxDesc;
    UINT                    tailTxDesc;
    UINT                    headRxDesc;
    UINT                    tailRxDesc;

    NDIS_HANDLE         pDmaHandle;
//    tEdrvInitParam      initParam;

#if CONFIG_EDRV_USE_DIAGNOSTICS != FALSE
    ULONGLONG           interruptCount;
    INT                 rxBufFreeMin;
    UINT                rxCount[EDRV_SAMPLE_NUM];
    UINT                txCount[EDRV_SAMPLE_NUM];
    UINT                pos;
#endif
} tEdrvInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

tEdrvInstance instance_l;

NDIS_OID supportedOids[6] =
{
    OID_GEN_SUPPORTED_LIST,
    OID_GEN_HARDWARE_STATUS,
    OID_GEN_MEDIA_SUPPORTED,
    OID_GEN_MEDIA_IN_USE,
    OID_802_3_PERMANENT_ADDRESS,
    OID_802_3_CURRENT_ADDRESS,
    //
    // TODO: identify and add addtional OID to be Supported
    // OID_GEN_LINK_PARAMETERS,
    // 

};


UCHAR aframe[DATA_LENGTH] = { 0x01, 0x11, 0x1e, 0x00, 0x00, 0x01, 0x00, 0x27, 0x0e, 0x37, 0x7b, 0xfa, 0x88, 0xab, 0x04, 0xff
, 0xf0, 0xfd, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

BOOLEAN fSend = FALSE;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static NDIS_STATUS acquireResources(PNDIS_RESOURCE_LIST allocatedResources_p);
static NDIS_STATUS initHardware(NDIS_HANDLE adapterHandle_p);
static void initQueues(NDIS_HANDLE adapterHandle_p);
static void freeQueues(NDIS_HANDLE adapterHandle_p);
static void releaseResources(NDIS_HANDLE adapterHandle_p);
static void shutHardware(NDIS_HANDLE adapterHandle_p);
static NDIS_STATUS sendFrame(UINT32 bufferNumber_p, UINT16 size_p);
BOOLEAN irqHandler(IN NDIS_HANDLE interruptContext_p, IN ULONG messageId_p,
    OUT PBOOLEAN queueDefaultInterruptDpc_p, OUT PULONG  targetProcessors_p);
VOID irqDpc(IN NDIS_HANDLE  interruptContext_p, IN ULONG messageId_p,
    IN PVOID dpcContext_p, IN PULONG  reserved1_p, IN PULONG reserved2_p);
static void processInterrupt(void);
//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Entry Point to the Driver

\param  driverObject_p      pointer to the driver object
\param  registryPath_p      pointer to the driver registry path

\return NDIS_STATUS - the value returned by NdisMRegisterMiniport
\retval NDIS_STATUS_SUCCESS

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
NTSTATUS DriverEntry(IN PDRIVER_OBJECT driverObject_p, IN PUNICODE_STRING registryPath_p)
{
    NDIS_STATUS                             status = NDIS_STATUS_SUCCESS;
    NDIS_MINIPORT_DRIVER_CHARACTERISTICS    miniportChar;

    // Prepare miniport charecteristic struture for registration
    NdisZeroMemory(&miniportChar, sizeof(miniportChar));

    DbgPrint("---->Driver Entry 1.2 \n");
    // Header parameters
    miniportChar.Header.Type = NDIS_OBJECT_TYPE_MINIPORT_DRIVER_CHARACTERISTICS;
    miniportChar.Header.Size = sizeof(NDIS_MINIPORT_DRIVER_CHARACTERISTICS);
    miniportChar.Header.Revision = NDIS_MINIPORT_DRIVER_CHARACTERISTICS_REVISION_1;

    // Driver and NDIS version supported by miniport
    miniportChar.MajorDriverVersion = OPLKMP_MAJOR_DRIVER_VERSION;
    miniportChar.MajorNdisVersion = OPLKMP_NDIS_MAJOR_VERSION;
    miniportChar.MinorDriverVersion = OPLKMP_MINOR_DRIVER_VERISON;
    miniportChar.MinorNdisVersion = OPLKMP_NDIS_MINOR_VERSION;

    // Callback entry points for NDIS 
    miniportChar.CancelOidRequestHandler = miniport_cancelOidRequest;
    miniportChar.CancelSendHandler = miniport_cancelSendNetBufferLists;
    miniportChar.CheckForHangHandlerEx = miniport_checkForHang;
    miniportChar.DevicePnPEventNotifyHandler = miniport_pnPEventNotify  ;
    miniportChar.HaltHandlerEx = miniport_halt;
    miniportChar.InitializeHandlerEx = miniport_initialize;
    miniportChar.OidRequestHandler = miniport_oidRequest;
    miniportChar.PauseHandler = miniport_pause;
    miniportChar.ResetHandlerEx = miniport_reset;
    miniportChar.RestartHandler = miniport_restart;
    miniportChar.ReturnNetBufferListsHandler = miniport_returnNetBufferLists;
    miniportChar.SetOptionsHandler = miniport_setOptions;
    miniportChar.ShutdownHandlerEx = miniport_shutdown;
    miniportChar.UnloadHandler = miniport_unload;
    miniportChar.SendNetBufferListsHandler = miniport_sendNetBufferLists;

    status = NdisMRegisterMiniportDriver(driverObject_p, registryPath_p, NULL, &miniportChar, &instance_l.pDriverHandle);
    if (status != NDIS_STATUS_SUCCESS)
    {
        DbgPrint("Miniport Registration Failed %x\n", status);
        miniport_unload(driverObject_p);
    }
    DbgPrint("<----Driver Entry \n");
    return status;
}

//------------------------------------------------------------------------------
/**
\brief  

NOTE: Prototype does not require this now.

\param  driverHandle_p      pointer to the driver object
\param  driverContext_p      pointer to the driver registry path

\return NDIS_STATUS - the value returned by NdisMRegisterMiniport
\retval NDIS_STATUS_SUCCESS

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
NDIS_STATUS miniport_setOptions( IN NDIS_HANDLE driverHandle_p, IN NDIS_HANDLE driverContext_p )
{
    UNREFERENCED_PARAMETER(driverHandle_p);
    UNREFERENCED_PARAMETER(driverContext_p);

    return NDIS_STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief  

NOTE: Prototype does not require this now.

\param  adapterHandle_p
\param  driverContext_p
\param  initParameters_p

\return NDIS_STATUS - the value returned by NdisMRegisterMiniport
\retval NDIS_STATUS_SUCCESS

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
NDIS_STATUS miniport_initialize(IN  NDIS_HANDLE adapterHandle_p, IN  NDIS_HANDLE driverContext_p,
                                IN  PNDIS_MINIPORT_INIT_PARAMETERS initParameters_p)
{
    NDIS_STATUS                                     status = NDIS_STATUS_SUCCESS;
    NDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES   registrationAttributes;
    NDIS_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES        generalAttributes;
    NDIS_SG_DMA_DESCRIPTION                         dmaDescription;
    NDIS_MINIPORT_INTERRUPT_CHARACTERISTICS         intrChars;
    INT                                             i;
    // store the adapter handle for future references
    instance_l.pAdapterHandle = adapterHandle_p;
    DbgPrint("---->Miniport Initialize \n");
    // Initialize the registration attributes structures
    NdisZeroMemory(&registrationAttributes, sizeof(NDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES));
    NdisZeroMemory(&generalAttributes, sizeof(NDIS_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES));

    // setting registration attributes
    registrationAttributes.Header.Type = NDIS_OBJECT_TYPE_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES;
    registrationAttributes.Header.Revision = NDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES_REVISION_1;
    registrationAttributes.Header.Size = sizeof(NDIS_MINIPORT_ADAPTER_REGISTRATION_ATTRIBUTES);

    registrationAttributes.MiniportAdapterContext = (NDIS_HANDLE) &instance_l;
    registrationAttributes.AttributeFlags = (NDIS_MINIPORT_ATTRIBUTES_HARDWARE_DEVICE |
                                             NDIS_MINIPORT_ATTRIBUTES_BUS_MASTER);

    registrationAttributes.CheckForHangTimeInSeconds = 4;
    registrationAttributes.InterfaceType = NdisInterfacePci;

    status = NdisMSetMiniportAttributes(adapterHandle_p,
        (PNDIS_MINIPORT_ADAPTER_ATTRIBUTES) &registrationAttributes);

    if (status != NDIS_STATUS_SUCCESS)
    {
        DbgPrint(" NdisMSetMiniportAttributes failed 0x%x\n", status);
        goto Exit;
    }

    // Get IO and interrupt resources
    acquireResources(initParameters_p->AllocatedResources);

    // Initialize hardware 
    initHardware(adapterHandle_p);

    // General attributes of hardware
    generalAttributes.Header.Type = NDIS_OBJECT_TYPE_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES;
    generalAttributes.Header.Revision = NDIS_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES_REVISION_1;
    generalAttributes.Header.Size = sizeof(NDIS_MINIPORT_ADAPTER_GENERAL_ATTRIBUTES);

    generalAttributes.MediaType = NdisMedium802_3;

    generalAttributes.MtuSize = 1500;
    generalAttributes.MaxXmitLinkSpeed = 100;
    generalAttributes.MaxRcvLinkSpeed = 100;
    generalAttributes.XmitLinkSpeed = 100;
    generalAttributes.RcvLinkSpeed = 100;
    generalAttributes.MediaConnectState = MediaConnectStateUnknown;
    generalAttributes.MediaDuplexState = MediaDuplexStateUnknown;
    generalAttributes.LookaheadSize = 1500;

    generalAttributes.PowerManagementCapabilities = NULL;
    generalAttributes.MacOptions = NDIS_MAC_OPTION_TRANSFERS_NOT_PEND |
                                   NDIS_MAC_OPTION_NO_LOOPBACK;

    generalAttributes.SupportedPacketFilters = NDIS_PACKET_TYPE_DIRECTED |
                                               NDIS_PACKET_TYPE_MULTICAST |
                                               NDIS_PACKET_TYPE_ALL_MULTICAST |
                                               NDIS_PACKET_TYPE_BROADCAST;

    generalAttributes.MaxMulticastListSize = 32;
    generalAttributes.MacAddressLength = ETH_LENGTH_OF_ADDRESS;


    //NdisMoveMemory(generalAttributes.PermanentMacAddress, Mac, ETH_LENGTH_OF_ADDRESS);

    //NdisMoveMemory(generalAttributes.CurrentMacAddress, Mac, ETH_LENGTH_OF_ADDRESS);

    generalAttributes.PhysicalMediumType = NdisPhysicalMedium802_3;
    generalAttributes.RecvScaleCapabilities = NULL;
    generalAttributes.AccessType = NET_IF_ACCESS_BROADCAST; // NET_IF_ACCESS_BROADCAST for a typical ethernet adapter
    generalAttributes.DirectionType = NET_IF_DIRECTION_SENDRECEIVE; // NET_IF_DIRECTION_SENDRECEIVE for a typical ethernet adapter
    generalAttributes.ConnectionType = NET_IF_CONNECTION_DEDICATED;  // NET_IF_CONNECTION_DEDICATED for a typical ethernet adapter
    generalAttributes.IfType = IF_TYPE_ETHERNET_CSMACD; // IF_TYPE_ETHERNET_CSMACD for a typical ethernet adapter (regardless of speed)
    generalAttributes.IfConnectorPresent = TRUE; // RFC 2665 TRUE if physical adapter

    generalAttributes.SupportedStatistics = NDIS_STATISTICS_XMIT_OK_SUPPORTED |
                                            NDIS_STATISTICS_RCV_OK_SUPPORTED;
    generalAttributes.SupportedPauseFunctions = NdisPauseFunctionsUnsupported;
    generalAttributes.AutoNegotiationFlags = NDIS_LINK_STATE_DUPLEX_AUTO_NEGOTIATED;
    generalAttributes.SupportedOidList = supportedOids;
    generalAttributes.SupportedOidListLength = sizeof(supportedOids);



    status = NdisMSetMiniportAttributes(adapterHandle_p,
        (PNDIS_MINIPORT_ADAPTER_ATTRIBUTES) &generalAttributes);

    if (status != NDIS_STATUS_SUCCESS)
    {
        DbgPrint("NdisMSetminiportAttributes Failed Status 0x%X\n", status);
        goto Exit;
    }

    // Initialze interrupt 
    // TODO: Add interrupt initialization
    NdisZeroMemory(&intrChars, sizeof(NDIS_MINIPORT_INTERRUPT_CHARACTERISTICS));

    intrChars.Header.Type = NDIS_OBJECT_TYPE_MINIPORT_INTERRUPT;
    intrChars.Header.Revision = NDIS_MINIPORT_INTERRUPT_REVISION_1;
    intrChars.Header.Size = sizeof(NDIS_MINIPORT_INTERRUPT_CHARACTERISTICS);
    intrChars.MsiSyncWithAllMessages = TRUE;
    intrChars.MessageInterruptHandler = irqHandler;
    intrChars.MessageInterruptDpcHandler = irqDpc;
    intrChars.DisableMessageInterruptHandler = NULL;
    intrChars.EnableMessageInterruptHandler = NULL;

    status = NdisMRegisterInterruptEx(adapterHandle_p,
                                      &instance_l,
                                      &intrChars,
                                      &instance_l.pInterruptHandle
                                     );


    if (status != NDIS_STATUS_SUCCESS)
    {
        DbgPrint("NdisMRegisterInterrupt failed Status 0x%x\n", status);
        goto Exit;
    }

    //
    // Check If the driver MSI is Identified 
    //
    if (intrChars.InterruptType == NDIS_CONNECT_MESSAGE_BASED)
    {
        DbgPrint("Using MSI interrupt\n");
    }
    else
    {
        DbgPrint("Not Using Message Interrupts\n");
    }

    // Register DMA channel
    NdisZeroMemory(&dmaDescription, sizeof(NDIS_SG_DMA_DESCRIPTION));
    dmaDescription.Header.Type = NDIS_OBJECT_TYPE_SG_DMA_DESCRIPTION;
    dmaDescription.Header.Revision = NDIS_SG_DMA_DESCRIPTION_REVISION_1;
    dmaDescription.Header.Size = NDIS_SIZEOF_SG_DMA_DESCRIPTION_REVISION_1;

    dmaDescription.Flags = 0;
    dmaDescription.MaximumPhysicalMapping = (ULONG)-1;
    dmaDescription.SharedMemAllocateCompleteHandler = NULL;
    dmaDescription.ScatterGatherListSize = 0;

    status = NdisMRegisterScatterGatherDma(adapterHandle_p, &dmaDescription, &instance_l.pDmaHandle);
    //status = NdisMRegisterDmaChannel(&instance_l.pDmaHandle, adapterHandle_p, 0, FALSE, &dmaDescription, (ULONG)-1);
    if (status != NDIS_STATUS_SUCCESS)
    {
        DbgPrint("NdisMRegisterDmaChannel unable to register dma channel 0x%X\n", status);
        status = NDIS_STATUS_SUCCESS;
        goto Exit;
    }

    // initialize Multicast Table Array to 0
    for (i = 0; i < 128; i++)
    {
        EDRV_REGDW_WRITE(EDRV_REGDW_MTA(i), 0);
    }
    
    // Allocate TX and RX resources
    initQueues(adapterHandle_p);

    DbgPrint("%s waiting for link up...\n", __FUNCTION__);
    for (i = EDRV_LINK_UP_TIMEOUT; i > 0; i -= 100)
    {
        if ((EDRV_REGDW_READ(EDRV_REGDW_STATUS) & EDRV_REGDW_STATUS_LU) != 0)
        {
            break;
        }

        NdisMSleep(100000);
    }
    if (i == 0)
    {
        status = NDIS_STATUS_ADAPTER_NOT_READY;
        goto ExitFail;
    }


    // enable interrupts
    EDRV_REGDW_WRITE(EDRV_REGDW_IMS, EDRV_REGDW_INT_MASK_DEF);

    // Prepare frame for send test
    NdisMoveMemory(instance_l.pTxBuf, aframe, DATA_LENGTH);
    goto Exit;

ExitFail:
    // remove here

Exit:
    DbgPrint("<----MIniport Initialize \n");
    return status;
}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
VOID miniport_halt(IN  NDIS_HANDLE adapterContext_p, IN NDIS_HALT_ACTION haltAction_p)
{
    UNREFERENCED_PARAMETER(haltAction_p);
    UNREFERENCED_PARAMETER(adapterContext_p);
    DbgPrint("---->%s \n",__FUNCTION__);

    if (instance_l.pAdapterHandle == NULL)
    {
        // nothing to do
        return;
    }
    // Stop hardware
    shutHardware(instance_l.pAdapterHandle);
    // Free queues
    freeQueues(instance_l.pAdapterHandle);

    if (instance_l.pInterruptHandle != NULL)
    {
        NdisMDeregisterInterruptEx(instance_l.pInterruptHandle);
        instance_l.pInterruptHandle = NULL;
    }

    // De-Register the DMA channel
    if (instance_l.pDmaHandle != NULL)
    {
        NdisMDeregisterScatterGatherDma(instance_l.pDmaHandle);
    }
    // Release resources
    releaseResources(instance_l.pAdapterHandle);
    DbgPrint("<----%s \n",__FUNCTION__);
}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
VOID miniport_unload(IN PDRIVER_OBJECT driverObject_p)
{
    NdisMDeregisterMiniportDriver(instance_l.pDriverHandle);
}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
NDIS_STATUS miniport_pause(IN NDIS_HANDLE adapterContext_p,
                           IN PNDIS_MINIPORT_PAUSE_PARAMETERS pauseParameters_p)
{

    UNREFERENCED_PARAMETER(adapterContext_p);
    UNREFERENCED_PARAMETER(pauseParameters_p);

    return NDIS_STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
NDIS_STATUS miniport_restart(IN  NDIS_HANDLE adapterContext_p,
                             PNDIS_MINIPORT_RESTART_PARAMETERS restartParameters_p)
{
    UNREFERENCED_PARAMETER(adapterContext_p);
    UNREFERENCED_PARAMETER(restartParameters_p);

    return NDIS_STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
BOOLEAN miniport_checkForHang(IN NDIS_HANDLE adapterContext_p)
{
    UNREFERENCED_PARAMETER(adapterContext_p);
    return FALSE;
}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
NDIS_STATUS miniport_oidRequest(IN NDIS_HANDLE adapterContext_p,
                                IN PNDIS_OID_REQUEST   request_p)
{
    return NDIS_STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
VOID miniport_sendNetBufferLists(IN  NDIS_HANDLE adapterContext_p,
                                 IN  PNET_BUFFER_LIST netBufferList_p,
                                 IN  NDIS_PORT_NUMBER portNumber_p,
                                 IN  ULONG sendFlags_p)
{

}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
VOID miniport_shutdown(IN NDIS_HANDLE adapterContext_p,
                       IN NDIS_SHUTDOWN_ACTION shutdownAction_p)
{
    UNREFERENCED_PARAMETER(adapterContext_p);
    UNREFERENCED_PARAMETER(shutdownAction_p);
}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
VOID miniport_returnNetBufferLists(IN NDIS_HANDLE adapterContext_p,
                                   IN PNET_BUFFER_LIST netBufferLists_p,
                                   IN ULONG returnFlags_p)
{

}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
VOID miniport_cancelSendNetBufferLists(IN NDIS_HANDLE adapterContext_p, IN PVOID cancelId_p)
{
    UNREFERENCED_PARAMETER(adapterContext_p);
    UNREFERENCED_PARAMETER(cancelId_p);
}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
VOID miniport_pnPEventNotify(IN NDIS_HANDLE adapterContext_p,
                             IN PNET_DEVICE_PNP_EVENT netDevicePnPEvent_p)
{

}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
VOID miniport_cancelOidRequest(IN NDIS_HANDLE adapterContext_p, IN PVOID requestId_p)
{
    UNREFERENCED_PARAMETER(adapterContext_p);
    UNREFERENCED_PARAMETER(requestId_p);
}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
NDIS_STATUS miniport_reset(IN NDIS_HANDLE adapterContext_p, OUT PBOOLEAN addressingReset_p)
{

    UNREFERENCED_PARAMETER(adapterContext_p);
    UNREFERENCED_PARAMETER(addressingReset_p);

    return NDIS_STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
NTSTATUS miniport_ioDispatch(IN PDEVICE_OBJECT deviceObject_p, IN PIRP irp_p)
{
    return NDIS_STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
NTSTATUS miniport_deviceIoControl(IN PDEVICE_OBJECT deviceObject_p, IN PIRP Irp_p)
{
    return NDIS_STATUS_SUCCESS;
}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
VOID miniport_sgListHandler(IN PDEVICE_OBJECT pDO_p, IN PVOID reserved_p, IN PSCATTER_GATHER_LIST pSGL_p,
                            IN PVOID context_p)
{
    DbgPrint("---->%s", __FUNCTION__);
    UNREFERENCED_PARAMETER(pDO_p);
    UNREFERENCED_PARAMETER(reserved_p);
    UNREFERENCED_PARAMETER(pSGL_p);
    UNREFERENCED_PARAMETER(context_p);
    DbgPrint("<----%s", __FUNCTION__);

}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief Interrupt Service routine for MSI

NOTE: Prototype does not require this now.

\param  interruptContext_p          Pointer to the interrupt context.
\param  messageId_p                 
\param  queueDefaultInterruptDpc_p  TRUE on return if MiniportHandleInterrupt 
                                    should be called on default CPU.
\param  targetProcessors_p          Pointer to a bitmap specifying 
                                    Target processors which should run the DPC

\returns TRUE    The miniport recognizes the interrupt
         FALSE   Otherwise
\ingroup module_miniport
*/
//------------------------------------------------------------------------------
BOOLEAN irqHandler(IN NDIS_HANDLE interruptContext_p, IN ULONG messageId_p,
                   OUT PBOOLEAN queueDefaultInterruptDpc_p, OUT PULONG  targetProcessors_p)
{

    UNREFERENCED_PARAMETER(interruptContext_p);
    UNREFERENCED_PARAMETER(messageId_p);
    UNREFERENCED_PARAMETER(targetProcessors_p);

   // DbgPrint("Interrupt\n");
    processInterrupt();
    *queueDefaultInterruptDpc_p = FALSE;

    return TRUE;
}

//------------------------------------------------------------------------------
/**
\brief DPC for the ISR

NOTE: Prototype does not require this now.

\param  interruptContext_p          Pointer to the interrupt context.
\param  messageId_p
\param  dpcContext_p                Pointer to the DPC context.
\param  targetProcessors_p          Pointer to a bitmap specifying
Target processors which should run the DPC

\returns void

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
void irqDpc(IN NDIS_HANDLE  interruptContext_p, IN ULONG messageId_p,
            IN PVOID dpcContext_p, IN PULONG  reserved1_p, IN PULONG reserved2_p)
{

    //Do nothing
}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
static void processInterrupt(void)
{
    UINT32          status;

    //DbgPrint("Interrupt Handler Called\n");
    // Read the interrupt status
    status = EDRV_REGDW_READ(EDRV_REGDW_ICR);

    if ((status & EDRV_REGDW_INT_MASK_DEF) == 0)
    {
        goto Exit;
    }

    if ((status & EDRV_REGDW_INT_INT_ASSERTED) == 0)
    {   // Manual acknowledge required
        EDRV_REGDW_WRITE(EDRV_REGDW_ICR, status);
    }



    if ((status & (EDRV_REGDW_INT_RXT0 | EDRV_REGDW_INT_SRPD | EDRV_REGDW_INT_RXDMT0 | // Receive interrupt
        EDRV_REGDW_INT_TXDW)) != 0)                                         // Transmit interrupt
    {
        UINT            headRxDescOrg;

        headRxDescOrg = instance_l.headRxDesc;

        if (instance_l.pTxBuf == NULL)
        {
            DbgPrint("%s Tx buffers currently not allocated\n", __FUNCTION__);
            goto Exit;
        }
        do
        {
            tEdrvRxDesc*    pRxDesc;
            tEdrvTxDesc*    pTxDesc;
            // Process receive descriptors
            pRxDesc = &instance_l.pRxDesc[instance_l.headRxDesc];

            while (pRxDesc->status != 0)
            {
                // Rx frame available
                UINT8                   rxStatus;
                UINT8                   rxError;
                UINT8*                  rxBuffer;

                rxStatus = pRxDesc->status;
                rxError = pRxDesc->error;

                if ((rxStatus & EDRV_RXSTAT_DD) != 0)
                {   // Descriptor is valid

                    if ((rxStatus & EDRV_RXSTAT_EOP) == 0)
                    {   // Multiple descriptors used for one packet
                        //Do Nothin;
                    }
                    else if ((rxError & EDRV_RXERR_CE) != 0)
                    {   // CRC error
                        //Do Nothin;
                    }
                    else if ((rxError & EDRV_RXERR_SEQ) != 0)
                    {   // Packet sequence error
                        //Do Nothin;
                    }
                    else if ((rxError & EDRV_RXERR_OTHER) != 0)
                    {   // Other error
                        //Do Nothin;
                    }
                    else
                    {
                        // Packet Ok
                        rxBuffer = instance_l.aRxBuffUsed[instance_l.headRxDesc].rxBuffVa;

                        if (rxBuffer[14] == 3)
                        {
                            //DbgPrint("SoC Received \n");
                            sendFrame(0, DATA_LENGTH);
                        }

                    }
                }
                pRxDesc->status = 0;

                instance_l.headRxDesc = (instance_l.headRxDesc + 1) & EDRV_RX_DESC_MASK;
                pRxDesc = &instance_l.pRxDesc[instance_l.headRxDesc];
            }

            // Process one transmit descriptor
            pTxDesc = &instance_l.pTxDesc[instance_l.headTxDesc];

            if ((pTxDesc->status_le & EDRV_TX_DESC_STATUS_DD) != 0)
            {

                UINT32          txStatus;

                //DbgPrint("Tx Interrupt\n");
                txStatus = pTxDesc->status_le;

                // Delete DD flag
                pTxDesc->status_le = 0;

                // Increment Tx descriptor queue head pointer
                instance_l.headTxDesc = (instance_l.headTxDesc + 1) & EDRV_TX_DESC_MASK;

                if ((txStatus & EDRV_TX_DESC_STATUS_EC) != 0)
                {
                    //EDRV_COUNT_TX_COL_RL;
                }
                else if ((txStatus & EDRV_TX_DESC_STATUS_LC) != 0)
                {
                    //EDRV_COUNT_LATECOLLISION;
                }
                else
                {
                    //EDRV_COUNT_TX;
                }
            }
            else
            {
                break;
            }

        } while (instance_l.headTxDesc != instance_l.tailTxDesc);

        if (instance_l.headRxDesc != headRxDescOrg)
        {
            // Release receive descriptors
            if (instance_l.headRxDesc == 0)
            {
                instance_l.tailRxDesc = EDRV_MAX_RX_DESCS - 1;
            }
            else
            {
                instance_l.tailRxDesc = instance_l.headRxDesc - 1;
            }

            EDRV_REGDW_WRITE(EDRV_REGDW_RDT0, instance_l.tailRxDesc);
        }
    }


    if ((status & (EDRV_REGDW_INT_RXSEQ | EDRV_REGDW_INT_RXO)) != 0)
    {   // Receive error interrupt

        if ((status & (EDRV_REGDW_INT_RXSEQ)) != 0)
        {   // Ethernet frame sequencing error
            DbgPrint("Sequence Erorr\n");
        }

        if ((status & (EDRV_REGDW_INT_RXO)) != 0)
        {   // Receive queue overrun
            DbgPrint("OverRun\n");
        }
    }
Exit:
    return;
}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
static NDIS_STATUS acquireResources(PNDIS_RESOURCE_LIST allocatedResources_p)
{
    NDIS_STATUS                         status = NDIS_STATUS_SUCCESS;
    PCM_PARTIAL_RESOURCE_DESCRIPTOR     resDescriptor;
    ULONG                               resCount;
    BOOLEAN                             fBarResistered = FALSE;

    for (resCount = 0; resCount < allocatedResources_p->Count; resCount++)
    {
        resDescriptor = &allocatedResources_p->PartialDescriptors[resCount];
        if (resDescriptor == NULL)
        {
            DbgPrint("No resources Returned from Hardware\n");
            return STATUS_DEVICE_CONFIGURATION_ERROR;
        }

        switch (resDescriptor->Type)
        {
            case CmResourceTypePort:
                // we don't use this ignore
                break;
            case CmResourceTypeMemory:
                //register bar0 to access control registers
                if (!fBarResistered)
                {
                    if (resDescriptor->u.Memory.Length == 0x20000)
                    {
                        // remap IO registers
                        fBarResistered = TRUE;
                        status = NdisMMapIoSpace(&instance_l.pIoAddr, instance_l.pAdapterHandle, resDescriptor->u.Memory.Start,
                            resDescriptor->u.Memory.Length);
                        if (status != NDIS_STATUS_SUCCESS)
                        {
                            DbgPrint("Error Remapping Memory\n");
                            return status;;
                        }
                        else
                        {
                            DbgPrint("IO memory Mapped at :%p-%d\n", instance_l.pIoAddr, resDescriptor->u.Memory.Length);
                            instance_l.ioLength = resDescriptor->u.Memory.Length;
                        }

                    }
                }
                break;
            case CmResourceTypeInterrupt:
                // nothing to do now
                //TODO: register interrupts here
                break;
            default:
                DbgPrint("Unspecified resource found %x\n", resDescriptor->Type);
                break;
        }
    }

    return status;

}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
static NDIS_STATUS initHardware(NDIS_HANDLE adapterHandle_p)
{
    NDIS_STATUS     status = NDIS_STATUS_SUCCESS;
    UINT32          temp;
    INT             i;

    // disable GIO Master accesses
    temp = EDRV_REGDW_READ(EDRV_REGDW_CTRL);
    temp |= EDRV_REGDW_CTRL_MST_DIS;
    EDRV_REGDW_WRITE(EDRV_REGDW_CTRL, temp);

    // wait until master is disabled
    for (i = EDRV_MASTER_DISABLE_TIMEOUT; i > 0; i--)
    {
        if ((EDRV_REGDW_READ(EDRV_REGDW_STATUS) & EDRV_REGDW_STATUS_MST_EN) == 0)
        {
            break;
        }

        NdisMSleep(1000);
    }
    if (i == 0)
    {
        status = NDIS_STATUS_DEVICE_FAILED;
        goto ExitFail;
    }

    // disable interrupts
    EDRV_REGDW_WRITE(EDRV_REGDW_IMC, EDRV_REGDW_INT_MASK_ALL);

    // reset controller
    temp |= EDRV_REGDW_CTRL_RST;
    EDRV_REGDW_WRITE(EDRV_REGDW_CTRL, temp);

    // wait until reset has finished and configuration from EEPROM was read
    for (i = EDRV_AUTO_READ_DONE_TIMEOUT; i > 0; i--)
    {
        if ((EDRV_REGDW_READ(EDRV_REGDW_EEC) & EDRV_REGDW_EEC_AUTO_RD) != 0)
        {
            break;
        }

        NdisMSleep(1000);
    }
    if (i == 0)
    {
        status = NDIS_STATUS_DEVICE_FAILED;
        goto ExitFail;
    }

    // disable interrupts
    EDRV_REGDW_WRITE(EDRV_REGDW_IMC, EDRV_REGDW_INT_MASK_ALL);
    temp = EDRV_REGDW_READ(EDRV_REGDW_ICR);

    // set global configuration
    EDRV_REGDW_WRITE(EDRV_REGDW_CTRL, EDRV_REGDW_CTRL_DEF);

    // PHY reset by software
    // 1. Obtain the Software/Firmware semaphore (SWSM.SWESMBI). Set it to 1b.
    temp = EDRV_REGDW_READ(EDRV_REGDW_SWSM);
    temp |= EDRV_REGDW_SWSM_SWESMBI;
    EDRV_REGDW_WRITE(EDRV_REGDW_SWSM, temp);
    // 2. Drive PHY reset (CTRL.PHY_RST, write 1b, wait 100 us, and then write 0b).
    temp = EDRV_REGDW_READ(EDRV_REGDW_CTRL);
    temp |= EDRV_REGDW_CTRL_PHY_RST;
    EDRV_REGDW_WRITE(EDRV_REGDW_CTRL, temp);
    NdisMSleep(1000);
    temp &= ~EDRV_REGDW_CTRL_PHY_RST;
    EDRV_REGDW_WRITE(EDRV_REGDW_CTRL, temp);
    // 3. Delay 10 ms
    NdisMSleep(10000);
    // 4. Start configuring the PHY.

    // 5. Release the Software/Firmware semaphore
    temp = EDRV_REGDW_READ(EDRV_REGDW_SWSM);
    temp &= ~EDRV_REGDW_SWSM_SWESMBI;
    EDRV_REGDW_WRITE(EDRV_REGDW_SWSM, temp);

    // Clear statistical registers
    temp = EDRV_REGDW_READ(EDRV_REGDW_CRCERRS);
    temp = EDRV_REGDW_READ(EDRV_REGDW_LATECOL);
    temp = EDRV_REGDW_READ(EDRV_REGDW_COLC);
    temp = EDRV_REGDW_READ(EDRV_REGDW_SEC);
    temp = EDRV_REGDW_READ(EDRV_REGDW_RLEC);

    // set TIPG
    EDRV_REGDW_WRITE(EDRV_REGDW_TIPG, EDRV_REGDW_TIPG_DEF);

    goto Exit;

ExitFail:
    // Clean and deregister resources

Exit:

    return status;

}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
static void shutHardware(NDIS_HANDLE adapterHandle_p)
{
    UNREFERENCED_PARAMETER(adapterHandle_p);
    return;
}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
static void initQueues(NDIS_HANDLE adapterHandle_p)
{
    INT         i;
    UINT64      descAddress;
    void*       pRxBuffVa;
    UINT64      pRxBuffPa;
    // Tx Buffer 
    NdisMAllocateSharedMemory(adapterHandle_p, EDRV_TX_BUFFER_SIZE, FALSE, &instance_l.pTxBuf, &instance_l.pTxBufDma);
    if (instance_l.pTxBuf == NULL)
    {
        DbgPrint("Failed to allocate shared memory");
    }
    // Tx Descriptor
    NdisMAllocateSharedMemory(adapterHandle_p, EDRV_TX_DESCS_SIZE, FALSE, (PVOID *)&instance_l.pTxDesc, &instance_l.pTxDescDma);

    // Rx Buffer 
    NdisMAllocateSharedMemory(adapterHandle_p, EDRV_RX_BUFFER_SIZE, FALSE, &instance_l.pRxBuf, &instance_l.pRxBufDma);

    // Rx Descriptor
    NdisMAllocateSharedMemory(adapterHandle_p, EDRV_RX_DESCS_SIZE, FALSE, (PVOID *) &instance_l.pRxDesc, &instance_l.pRxDescDma);

    for (i = 0; i < EDRV_MAX_RX_DESCS; i++)
    {
        if (instance_l.pRxBuf != NULL && (instance_l.pRxBufDma.QuadPart != 0LL))
        {
            pRxBuffVa = (void*) (((ULONG_PTR) instance_l.pRxBuf + (i * EDRV_RX_MAX_FRAME_SIZE)));
            pRxBuffPa = instance_l.pRxBufDma.QuadPart + (i * EDRV_RX_MAX_FRAME_SIZE);

            instance_l.pRxDesc[i].bufferAddr_le = pRxBuffPa;
            instance_l.pRxDesc[i].status = 0;

            instance_l.aRxBuffUsed[i].rxBuffPa.QuadPart = pRxBuffPa;
            instance_l.aRxBuffUsed[i].rxBuffVa = pRxBuffVa;

        }

    }

    // Initialize the queues in hardware
    EDRV_REGDW_WRITE(EDRV_REGDW_RXDCTL, EDRV_REGDW_RXDCTL_DEF);
    // Rx buffer size is set to 2048 by default
    // Rx descriptor typ is set to legacy by default
    descAddress = instance_l.pRxDescDma.QuadPart;
    EDRV_REGDW_WRITE(EDRV_REGDW_RDBAL0, (descAddress & 0xFFFFFFFF));
    EDRV_REGDW_WRITE(EDRV_REGDW_RDBAH0, (descAddress >> 32));
    EDRV_REGDW_WRITE(EDRV_REGDW_RDLEN0, EDRV_RX_DESCS_SIZE);
    instance_l.headRxDesc = 0;
    EDRV_REGDW_WRITE(EDRV_REGDW_RDH0, 0);
    instance_l.tailRxDesc = EDRV_MAX_RX_DESCS - 1;
    EDRV_REGDW_WRITE(EDRV_REGDW_RDT0, EDRV_MAX_RX_DESCS - 1);

    // enable receiver
    EDRV_REGDW_WRITE(EDRV_REGDW_RCTL, EDRV_REGDW_RCTL_DEF);

    // initialize Tx descriptors
    // NdisZeroMemory(instance_l.apTxBuffer, 0, sizeof(edrvInstance_l.apTxBuffer));
    EDRV_REGDW_WRITE(EDRV_REGDW_TXDCTL, EDRV_REGDW_TXDCTL_DEF);
    descAddress = instance_l.pTxDescDma.QuadPart;
    EDRV_REGDW_WRITE(EDRV_REGDW_TDBAL, (descAddress & 0xFFFFFFFF));
    EDRV_REGDW_WRITE(EDRV_REGDW_TDBAH, (descAddress >> 32));
    EDRV_REGDW_WRITE(EDRV_REGDW_TDLEN, EDRV_TX_DESCS_SIZE);
    EDRV_REGDW_WRITE(EDRV_REGDW_TDH, 0);
    EDRV_REGDW_WRITE(EDRV_REGDW_TDT, 0);

    // enable transmitter
    EDRV_REGDW_WRITE(EDRV_REGDW_TCTL, EDRV_REGDW_TCTL_DEF);

}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
static void freeQueues(NDIS_HANDLE adapterHandle_p)
{
    if (instance_l.pTxBuf != NULL)
    {
        NdisMFreeSharedMemory(adapterHandle_p, EDRV_TX_BUFFER_SIZE, FALSE, instance_l.pTxBuf, instance_l.pTxBufDma);
        instance_l.pTxBuf = NULL;
    }

    if (instance_l.pTxDesc != NULL)
    {
        NdisMFreeSharedMemory(adapterHandle_p, EDRV_TX_DESCS_SIZE, FALSE, (void *) instance_l.pTxDesc, instance_l.pTxDescDma);
        instance_l.pTxDesc = NULL;
    }

    if (instance_l.pRxBuf != NULL)
    {
        NdisMFreeSharedMemory(adapterHandle_p, EDRV_RX_BUFFER_SIZE, FALSE, instance_l.pRxBuf, instance_l.pRxBufDma);
        instance_l.pRxBuf = NULL;
    }

    if (instance_l.pRxDesc != NULL)
    {
        NdisMFreeSharedMemory(adapterHandle_p, EDRV_RX_DESCS_SIZE, FALSE, (void *) instance_l.pRxDesc, instance_l.pRxDescDma);
        instance_l.pRxDesc = NULL;
    }

}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
static void releaseResources(NDIS_HANDLE adapterHandle_p)
{
    if (instance_l.pIoAddr != NULL)
    {
        NdisMUnmapIoSpace(adapterHandle_p, instance_l.pIoAddr, instance_l.ioLength);
        instance_l.pIoAddr = NULL;
    }

    // Release interrupt here 
}

//------------------------------------------------------------------------------
/**
\brief

NOTE: Prototype does not require this now.

\param  adapterContext_p
\param  haltAction_p

\ingroup module_miniport
*/
//------------------------------------------------------------------------------
static NDIS_STATUS sendFrame(UINT32 bufferNumber_p, UINT16 size_p)
{
    tEdrvTxDesc*    pTxDesc;
    static  UINT32  count = 0;
    // one descriptor has to be left empty for distinction between full and empty
    if (((instance_l.tailTxDesc + 1) & EDRV_TX_DESC_MASK) == instance_l.headTxDesc)
    {
        return NDIS_STATUS_RESOURCES;
    }

    pTxDesc = &instance_l.pTxDesc[instance_l.tailTxDesc];
    pTxDesc->bufferAddr_le = instance_l.pTxBufDma.QuadPart + (bufferNumber_p * EDRV_MAX_FRAME_SIZE);
    pTxDesc->status_le = 0;
    pTxDesc->lengthCmd_le = ((UINT32) size_p) | EDRV_TX_DESC_CMD_DEF;
    *((UINT32*) &((UINT8*) instance_l.pTxBuf)[24]) = count;
    // increment Tx descriptor queue tail pointer
    instance_l.tailTxDesc = (instance_l.tailTxDesc + 1) & EDRV_TX_DESC_MASK;

    // start transmission
    EDRV_REGDW_WRITE(EDRV_REGDW_TDT, instance_l.tailTxDesc);
    count++;
    return NDIS_STATUS_SUCCESS;

}