#include "ProtoTypes.h"
#include <initguid.h>
#include "public.h"
#include <ntddk.h>  
#pragma alloc_text(INIT, DriverEntry)
#pragma alloc_text (PAGE, HpmnEvtDeviceAdd)
#pragma alloc_text (PAGE, HpmnEvtDevicePrepareHardware)
#pragma alloc_text (PAGE, HpmnEvtDeviceReleaseHardware)
#pragma alloc_text (PAGE, HpmnEvtDeviceIoDefault)
#pragma alloc_text (PAGE, HpmnEvtDeviceD0Entry)
#pragma alloc_text (PAGE, HpmnEvtDeviceD0Exit)

KDEFERRED_ROUTINE CustomTimerDpc;

VOID CustomTimerDpc(
 struct _KDPC *Dpc,
  PVOID DeferredContext,
  PVOID SystemArgument1,
  PVOID SystemArgument2
);

KSTART_ROUTINE ThreadStart;

VOID ThreadStart(
  PVOID StartContext
);


KDPC timerDpc;

KTIMER Timer;
LARGE_INTEGER		   dueTime;
LARGE_INTEGER		   counter;
LARGE_INTEGER		   Freq;
	
HANDLE  kThreadHandle;
OBJECT_ATTRIBUTES ObjectAttributes;
static unsigned int flag = 0;
static BOOLEAN fkillThread = FALSE;
static BOOLEAN fwaitThread = FALSE;
NTSTATUS
DriverEntry(
    PDRIVER_OBJECT  DriverObject,
  PUNICODE_STRING RegistryPath
    )
{
    NTSTATUS               status = STATUS_SUCCESS;
   WDF_DRIVER_CONFIG      config;
    WDF_OBJECT_ATTRIBUTES  attrib;
    WDFDRIVER              driver;

      UNREFERENCED_PARAMETER(DriverObject);
	 UNREFERENCED_PARAMETER(RegistryPath);
   WDF_DRIVER_CONFIG_INIT(&config, HpmnEvtDeviceAdd);

    status = WdfDriverCreate(
                      DriverObject,
                      RegistryPath,
                      WDF_NO_OBJECT_ATTRIBUTES,
                      &config,
                      WDF_NO_HANDLE);
    if(status!=STATUS_SUCCESS)
	{
	 DbgPrint("Wdf Failed to Create Device\n");
    }
	else
	DbgPrint("WDFCREATE SUCCESSFULL \n"); 

	//while(flag < 10000)
	//{
	//	counter = KeQueryPerformanceCounter(&Freq);
		//DbgPrint("%lld\n",counter.QuadPart);
		//flag++;
	//}
	DbgPrint("Initialize Timer\n");
	//KeInitializeTimer(&Timer);
	//KeInitializeDpc(&timerDpc,CustomTimerDpc,NULL);
    //driverContext = GetDriverContext(driver);
    DbgPrint("[HPMN]:Driver Entry\n");
    return status;

}

NTSTATUS HpmnEvtDeviceAdd(
                      IN WDFDRIVER  Driver,
                      IN PWDFDEVICE_INIT  DeviceInit
                      )
{
  NTSTATUS status;
  WDFDEVICE device;
  ULONG DesiredAccess = DELETE;
  PDEVICE_CONTEXT devCtx = NULL;
  WDF_OBJECT_ATTRIBUTES attributes;
  WDF_PNPPOWER_EVENT_CALLBACKS pnpPowerCallbacks;
  WDF_IO_QUEUE_CONFIG ioQConfig;

  UNREFERENCED_PARAMETER(Driver);

  DbgPrint("--> EvtDeviceAdd\n");

  /*set the callback functions that will be executed on PNP and Power events*/
  WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpPowerCallbacks);
  pnpPowerCallbacks.EvtDevicePrepareHardware = HpmnEvtDevicePrepareHardware;
  pnpPowerCallbacks.EvtDeviceReleaseHardware = HpmnEvtDeviceReleaseHardware;
  pnpPowerCallbacks.EvtDeviceD0Entry = HpmnEvtDeviceD0Entry;
  pnpPowerCallbacks.EvtDeviceD0Exit = HpmnEvtDeviceD0Exit;
  WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnpPowerCallbacks);

  /*initialize storage for the device context*/
  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, DEVICE_CONTEXT);

  /*create a device instance.*/
  status = WdfDeviceCreate(&DeviceInit, &attributes, &device);  
  if(!NT_SUCCESS(status))
  {
    DbgPrint("WdfDeviceCreate failed with status 0x%08x\n");
    return status;
  }
  //dueTime.QuadPart = -10000LL;
 // KeSetTimer(&Timer,dueTime,&timerDpc);
 DbgPrint("Create Thread...");
 InitializeObjectAttributes(&ObjectAttributes, NULL, OBJ_KERNEL_HANDLE, NULL, NULL);
 status = PsCreateSystemThread(&kThreadHandle,DesiredAccess,&ObjectAttributes,NULL,
		  NULL,
          ThreadStart,
		  NULL);
if(status == STATUS_SUCCESS) 
{
	DbgPrint("Successful\n");
	counter = KeQueryPerformanceCounter(&Freq);
	DbgPrint("%lld\n",counter.QuadPart);
}

  devCtx = GetDeviceContext(device);

  DbgPrint("<-- EvtDeviceAdd\n");
  return status;
}

/*............................................................................*/
/* call-back function that will be called by the pnp/power manager after the  */
/* Plug and Play manager has assigned hardware resources to the device and    */
/* after the device has entered its uninitialized D0 state. The framework     */
/* calls the driver's EvtDevicePrepareHardware callback function before       */
/* calling the driver's EvtDeviceD0Entry callback function.                   */
/*............................................................................*/
NTSTATUS
HpmnEvtDevicePrepareHardware(
    IN WDFDEVICE    Device,
    IN WDFCMRESLIST ResourceList,
    IN WDFCMRESLIST ResourceListTranslated
    )
{
  NTSTATUS status = STATUS_SUCCESS;

  DbgPrint("--> HpmnEvtDevicePrepareHardware\n");

  UNREFERENCED_PARAMETER(Device);
  UNREFERENCED_PARAMETER(ResourceList);
  UNREFERENCED_PARAMETER(ResourceListTranslated);

 DbgPrint("<-- HpmnEvtDevicePrepareHardware\n");

  return status;
}

VOID
HpmnEvtDeviceIoDefault(
    IN WDFQUEUE  Queue,
    IN WDFREQUEST  Request
    )
{
  UNREFERENCED_PARAMETER(Queue);

  DbgPrint("--> EvtDeviceIoDefault\n");

  WdfRequestComplete(Request, STATUS_NOT_IMPLEMENTED);

  DbgPrint("<-- EvtDeviceIoDefault\n");

}

/*............................................................................*/
/* this function is called when the device is either started or woken up.     */
/*............................................................................*/
NTSTATUS
HpmnEvtDeviceD0Entry(
    IN WDFDEVICE  Device,
    IN WDF_POWER_DEVICE_STATE  PreviousState
    )
{
  NTSTATUS status = STATUS_SUCCESS;

  UNREFERENCED_PARAMETER(Device);
  UNREFERENCED_PARAMETER(PreviousState);

  DbgPrint("[HPMN]--> EvtDeviceD0Entry\n");

  DbgPrint("<-- EvtDeviceD0Entry\n");

  return status;
}

/*............................................................................*/
/* this function is called when the device is powered down.                   */
/* the current IO is left pending, because otherwise the continuous interrupt */
/* read IO will also be cancelled, and it would have to be reconfigured.      */
/*............................................................................*/
NTSTATUS
HpmnEvtDeviceD0Exit(
    IN WDFDEVICE  Device,
    IN WDF_POWER_DEVICE_STATE  TargetState
    )
{
  NTSTATUS status = STATUS_SUCCESS;

  UNREFERENCED_PARAMETER(Device);
  UNREFERENCED_PARAMETER(TargetState);

 DbgPrint("[HPMN]--> EvtDeviceD0Exit\n");

 DbgPrint("<-- EvtDeviceD0Exit\n");

  return status;
}

NTSTATUS
HpmnEvtDeviceReleaseHardware(
    IN  WDFDEVICE    Device,
    IN  WDFCMRESLIST ResourcesTranslated
    )
{
    	LARGE_INTEGER delay;
	DbgPrint("-->[HPMN]:HpmnEvtDeviceReleaseHardware");
    UNREFERENCED_PARAMETER(Device);
	UNREFERENCED_PARAMETER(ResourcesTranslated);
	
	fkillThread = TRUE;
	//delay.QuadPart = 1000LL;
	//KeDelayExecutionThread(KernelMode ,FALSE, &delay);
	// wait here for timer thread to exit
	while(!fwaitThread);
	ZwClose(kThreadHandle);
	DbgPrint("<--[HPMN]:HpmnEvtDeviceReleaseHardware");
	return STATUS_SUCCESS;
}


VOID CustomTimerDpc(
 struct _KDPC *Dpc,
   PVOID DeferredContext,
   PVOID SystemArgument1,
   PVOID SystemArgument2
)
{
	UNREFERENCED_PARAMETER(Dpc);
	UNREFERENCED_PARAMETER(DeferredContext);
	UNREFERENCED_PARAMETER(SystemArgument1);
	UNREFERENCED_PARAMETER(SystemArgument2);
	
	//DbgPrint("[HPMN]--> Timer Interrupt\n");
	counter = KeQueryPerformanceCounter(&Freq);
	DbgPrint("%lld %lld\n",counter.QuadPart,Freq.QuadPart);
	if(flag < 1000)
	{
		dueTime.QuadPart = -25000LL;
		KeSetTimer(&Timer,dueTime,&timerDpc);
		flag++;
	}
}

VOID ThreadStart(
  PVOID StartContext
)
{
	int count = 0;
	unsigned long long old = 0, new,dif,total = 0;
	LARGE_INTEGER delay;
	
	UNREFERENCED_PARAMETER(StartContext);
	DbgPrint("In Kernel Thread new\n");
	//delay.QuadPart = LL;
	while(!fkillThread)
	{
		counter = KeQueryPerformanceCounter(&Freq);
		new = counter.QuadPart;
		if(old)
		{
			dif = new - old;
			total += dif ;
		}
		old = new;
		//DbgPrint("old:%lld New :%lld dif:%lld us\n",old,new,dif);
		count++;
		//KeDelayExecutionThread(KernelMode ,FALSE, &delay);
	}
	
	DbgPrint("Total:%lld Average:%lld us\n",total,(total/count));
	fwaitThread = TRUE;
}