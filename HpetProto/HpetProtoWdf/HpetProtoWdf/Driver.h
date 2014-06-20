/*++

Module Name:

    driver.h

Abstract:

    This file contains the driver definitions.

Environment:

    Kernel-mode Driver Framework

--*/

#define INITGUID

#include <ntddk.h>
#include <wdf.h>

#include "device.h"
#include "queue.h"
#include "trace.h"
#include "hpetTimer.h"
//
// WDFDRIVER Events
//

DRIVER_INITIALIZE DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD HpetProtoWdfEvtDeviceAdd;
EVT_WDF_OBJECT_CONTEXT_CLEANUP HpetProtoWdfEvtDriverContextCleanup;
EVT_WDF_DRIVER_DEVICE_ADD hpetEvtDeviceAdd;
EVT_WDF_DEVICE_D0_ENTRY hpetEvtDeviceD0Entry;
EVT_WDF_DEVICE_PREPARE_HARDWARE hpetEvtDevicePrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE hpetEvtDeviceReleaseHardware;
EVT_WDF_DEVICE_D0_EXIT hpetEvtDeviceD0Exit;


void hpet_writel(unsigned int uiAddress_p, unsigned int uiData_p);
unsigned int hpet_readl(unsigned int uiAddress_p);
