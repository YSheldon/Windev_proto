
#pragma warning(push)
#pragma warning(disable:4115)  // named typedef in parenthesis
#pragma warning(disable:4200)  // nameless struct/union
#pragma warning(disable:4201)  // nameless struct/union
#pragma warning(disable:4214)  // bit field types other than int

#include "ntddk.h"
#include "wdf.h"
#define NTSTRSAFE_LIB
#include <ntstrsafe.h>
#include <wdmsec.h> // for SDDLs

#pragma warning(pop)

#define __DRIVER_NAME "BASIC: "
#define HPMNDEVICE_NAME_STRING	L"\\Device\\HPMN"
#define SYMBOLIC_NAME_STRING     L"\\DosDevices\\HPMN"

#define HPMN_BAR_LENGTH 0x1000000

/*declaration of the device context. we have to declare the type of the
  device context, and an accessor function name that will return to us
  a pointer to the device context.*/
typedef struct _DEVICE_CONTEXT
 {
  WDFDEVICE              	WdfDevice;
  WDFQUEUE             		IoDefaultQueue;
  WDFINTERRUPT           	WdfInterrupt;
  PHYSICAL_ADDRESS       	MemPhysAddressBar0;
  PUCHAR                 	CSRAddressBar0;
  PHYSICAL_ADDRESS       	MemPhysAddressBar1;
  PUCHAR                 	CSRAddressBar1;
  PHYSICAL_ADDRESS       	MemPhysAddressBar2;
  PUCHAR                 	CSRAddressBar2;
  BUS_INTERFACE_STANDARD 	BusInterface;
} DEVICE_CONTEXT, *PDEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(DEVICE_CONTEXT, GetDeviceContext);

typedef struct _INTERRUPT_DATA {
    PVOID Context; // Keep here the context information accessed by the ISR
                    // or with ISR lock held. Not used in this sample.
} INTERRUPT_DATA, *PINTERRUPT_DATA;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(INTERRUPT_DATA, HpmnGetInterruptData)


//////********Function Prototypes**********//////////


DRIVER_INITIALIZE DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD HpmnEvtDeviceAdd;
EVT_WDF_DEVICE_D0_ENTRY HpmnEvtDeviceD0Entry;
EVT_WDF_DEVICE_PREPARE_HARDWARE HpmnEvtDevicePrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE hpmnEvtDeviceReleaseHardware;
EVT_WDF_DEVICE_D0_EXIT HpmnEvtDeviceD0Exit;
EVT_WDF_IO_QUEUE_IO_DEFAULT HpmnEvtDeviceIoDefault;
EVT_WDF_IO_QUEUE_IO_READ  HpmnEvtIoRead;
EVT_WDF_IO_QUEUE_IO_WRITE  HpmnEvtIoWrite;
EVT_WDF_IO_QUEUE_IO_DEVICE_CONTROL  HpmnEvtIoDeviceControl;




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

VOID
  HpmnEvtIoDeviceControl (
    IN WDFQUEUE  Queue,
    IN WDFREQUEST  Request,
    IN size_t  OutputBufferLength,
    IN size_t  InputBufferLength,
    IN ULONG  IoControlCode
    );
VOID
  HpmnEvtIoRead (
    IN WDFQUEUE  Queue,
    IN WDFREQUEST  Request,
    IN size_t  Length
    );

VOID
  HpmnEvtIoWrite (
    IN WDFQUEUE  Queue,
    IN WDFREQUEST  Request,
    IN size_t  Length
    );
	
VOID
HpmnEvtDeviceIoDefault(
    IN WDFQUEUE  Queue,
    IN WDFREQUEST  Request
    );
	
NTSTATUS
HpmnEvtInterruptEnable(
 WDFINTERRUPT Interrupt, 
 WDFDEVICE AssociatedDevice);
 
NTSTATUS 
HpmnEvtInterruptDisable(
WDFINTERRUPT Interrupt,
 WDFDEVICE AssociatedDevice);
 
BOOLEAN 
HpmnIsr (
WDFINTERRUPT Interrupt,
 ULONG MessageID);
 
VOID HpmnDpc(
WDFINTERRUPT Interrupt,
WDFOBJECT Device);


 VOID
PrintChars(
    __in_ecount(CountChars) PCHAR BufferAddress,
    __in size_t CountChars
    );
