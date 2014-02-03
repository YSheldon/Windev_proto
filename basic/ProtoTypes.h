
#pragma warning(push)
#pragma warning(disable:4115)  // named typedef in parenthesis
#pragma warning(disable:4200)  // nameless struct/union
#pragma warning(disable:4201)  // nameless struct/union
#pragma warning(disable:4214)  // bit field types other than int

#include "ntddk.h"
#include "wdf.h"

#pragma warning(pop)

#define __DRIVER_NAME "BASIC: "

/*declaration of the device context. we have to declare the type of the
  device context, and an accessor function name that will return to us
  a pointer to the device context.*/
typedef struct _DEVICE_CONTEXT {
  WDFQUEUE          IoDefaultQueue;
} DEVICE_CONTEXT, *PDEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(DEVICE_CONTEXT, GetDeviceContext);
DRIVER_INITIALIZE DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD HpmnEvtDeviceAdd;
EVT_WDF_DEVICE_D0_ENTRY HpmnEvtDeviceD0Entry;
EVT_WDF_DEVICE_PREPARE_HARDWARE HpmnEvtDevicePrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE hpmnEvtDeviceReleaseHardware;
EVT_WDF_DEVICE_D0_EXIT HpmnEvtDeviceD0Exit;
EVT_WDF_IO_QUEUE_IO_DEFAULT HpmnEvtDeviceIoDefault;
NTSTATUS
DriverEntry(
    IN PDRIVER_OBJECT  DriverObject, 
    IN PUNICODE_STRING  RegistryPath
    );





NTSTATUS
HpmnEvtDeviceAdd(
    IN WDFDRIVER        Driver,
    IN PWDFDEVICE_INIT  DeviceInit
    );

NTSTATUS
HpmnEvtDevicePrepareHardware(
    IN WDFDEVICE    Device,
    IN WDFCMRESLIST ResourceList,
    IN WDFCMRESLIST ResourceListTranslated
    );

VOID
HpmnEvtDeviceIoDefault(
    IN WDFQUEUE  Queue,
    IN WDFREQUEST  Request
    );

NTSTATUS
HpmnEvtDeviceD0Entry(
    IN WDFDEVICE  Device,
    IN WDF_POWER_DEVICE_STATE  PreviousState
    );

NTSTATUS
HpmnEvtDeviceD0Exit(
    IN WDFDEVICE  Device,
    IN WDF_POWER_DEVICE_STATE  TargetState
    );
NTSTATUS
HpmnEvtDeviceReleaseHardware(
    IN  WDFDEVICE    Device,
    IN  WDFCMRESLIST ResourcesTranslated
    );


