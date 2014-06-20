/*++

Module Name:

    device.h

Abstract:

    This file contains the device definitions.

Environment:

    Kernel-mode Driver Framework

--*/

#include "public.h"

//
// The device context performs the same job as
// a WDM device extension in the driver frameworks
//
typedef struct _DEVICE_CONTEXT
{
    ULONG           PrivateDeviceData;  ///< just a placeholder
    void*           pIoAddr;            ///< IoAddr for the device
    WDFINTERRUPT    interrupt;
} DEVICE_CONTEXT, *PDEVICE_CONTEXT;

//
// This macro will generate an inline function called DeviceGetContext
// which will be used to get a pointer to the device context memory
// in a type safe manner.
//
WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(DEVICE_CONTEXT, DeviceGetContext)


/* some definitions for time conversion */
#define FSEC_PER_NSEC                   1000000LL
#define FSEC_PER_10PSEC                 10000LL
#define FSEC_PER_SEC                    1000000000000000LL
#define NSEC_PER_SEC                    1000000000LL
#define NSEC_PER_10PSEC                 100LL

//============================================================================//
// Function declarations                                                      //
//============================================================================//

EVT_WDF_INTERRUPT_ISR hpetIrqHandler;
EVT_WDF_INTERRUPT_DPC EvtInterruptDpc;
//
// Function to initialize the device and its callbacks
//
NTSTATUS
HpetProtoWdfCreateDevice(
    _Inout_ PWDFDEVICE_INIT DeviceInit
    );


