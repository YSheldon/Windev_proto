#include "ProtoTypes.h"
#include <initguid.h>
#include "public.h"
#include<wdmguid.h>
#pragma alloc_text(INIT, DriverEntry)
#pragma alloc_text (PAGE, HpmnEvtDeviceAdd)
#pragma alloc_text (PAGE, HpmnEvtDevicePrepareHardware)
#pragma alloc_text (PAGE, HpmnEvtDeviceReleaseHardware)
#pragma alloc_text (PAGE, HpmnEvtDeviceIoDefault)
#pragma alloc_text (PAGE, HpmnEvtDeviceD0Entry)
#pragma alloc_text (PAGE, HpmnEvtDeviceD0Exit)


NTSTATUS
DriverEntry(
    IN PDRIVER_OBJECT  DriverObject,
    IN PUNICODE_STRING RegistryPath
    )

{


    NTSTATUS               status = STATUS_SUCCESS;
    WDF_DRIVER_CONFIG      config;
    // WDF_OBJECT_ATTRIBUTES  attrib;
    //WDFDRIVER              driver;
    
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
DbgPrint("[HPMN]:WDFCREATE SUCCESSFULL");    // driverContext = GetDriverContext(driver);
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
  PDEVICE_CONTEXT devCtx = NULL;
  WDF_OBJECT_ATTRIBUTES attributes;
  WDF_PNPPOWER_EVENT_CALLBACKS pnpPowerCallbacks;
  WDF_IO_QUEUE_CONFIG ioQConfig;
  WDF_INTERRUPT_CONFIG interruptConfig;
  DECLARE_CONST_UNICODE_STRING(ntDeviceName, HPMNDEVICE_NAME_STRING) ;
  DECLARE_CONST_UNICODE_STRING(symbolicLinkName, SYMBOLIC_NAME_STRING) ;
	
  DECLSPEC_ALIGN(MEMORY_ALLOCATION_ALIGNMENT) UCHAR buffer[0x40];
  PPCI_COMMON_CONFIG  pPciConfig = (PPCI_COMMON_CONFIG) buffer;
  ULONG  bytesRead =0;
  //ULONG  i =0;
  UNREFERENCED_PARAMETER(Driver);

  DbgPrint("--> [HPMN]:EvtDeviceAdd\n");

  /*set the callback functions that will be executed on PNP and Power events*/
  WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpPowerCallbacks);
  pnpPowerCallbacks.EvtDevicePrepareHardware = HpmnEvtDevicePrepareHardware;
  pnpPowerCallbacks.EvtDeviceReleaseHardware = HpmnEvtDeviceReleaseHardware;
  pnpPowerCallbacks.EvtDeviceD0Entry = HpmnEvtDeviceD0Entry;
  pnpPowerCallbacks.EvtDeviceD0Exit = HpmnEvtDeviceD0Exit;
  WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnpPowerCallbacks);

  WdfDeviceInitSetIoType(DeviceInit, WdfDeviceIoBuffered);
  
  /*initialize storage for the device context*/
  WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, DEVICE_CONTEXT);
  attributes.SynchronizationScope = WdfSynchronizationScopeDevice;
  
  status = WdfDeviceInitAssignName(DeviceInit, &ntDeviceName);
   if(!NT_SUCCESS(status))
  {
    DbgPrint("WdfDeviceInitAssignName failed with status 0x%08x\n");
  }
	
  /*create a device instance.*/
  \
  status = WdfDeviceCreate(&DeviceInit, &attributes, &device);  
  if(!NT_SUCCESS(status))
  {
    DbgPrint("[HPMN]:WdfDeviceCreate failed with status 0x%08x\n");
    return status;
  }
  
  status = WdfDeviceCreateSymbolicLink(device, &symbolicLinkName);
  if(!NT_SUCCESS(status))
  {
    DbgPrint("[HPMN]:WdfDeviceInitAssignName failed with status 0x%08x\n");
  }
  devCtx = GetDeviceContext(device);

  /*create the default IO queue. this one will be used for ioctl requests*/
  WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&ioQConfig,
                                          WdfIoQueueDispatchSequential);
  ioQConfig.EvtIoDefault = HpmnEvtDeviceIoDefault;
  ioQConfig.EvtIoRead=HpmnEvtIoRead;
  ioQConfig.EvtIoWrite=HpmnEvtIoWrite;
  ioQConfig.EvtIoDeviceControl=HpmnEvtIoDeviceControl;
  
  status = WdfIoQueueCreate(device,
                            &ioQConfig,
                            WDF_NO_OBJECT_ATTRIBUTES,
                            &devCtx->IoDefaultQueue);
  if(!NT_SUCCESS(status))
  {
    
      DbgPrint("[HPMN]:WdfIoQueueCreate failed \n");
    return status;
  }
//////Interrupt//////

WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
WDF_OBJECT_ATTRIBUTES_SET_CONTEXT_TYPE(&attributes, INTERRUPT_DATA);

WDF_INTERRUPT_CONFIG_INIT(&interruptConfig,
                              HpmnIsr,
                              HpmnDpc);

    interruptConfig.AutomaticSerialization = TRUE;

    interruptConfig.EvtInterruptEnable  = HpmnEvtInterruptEnable;
    interruptConfig.EvtInterruptDisable = HpmnEvtInterruptDisable;

    status = WdfInterruptCreate(device,
                                &interruptConfig,
                                &attributes,
                                &devCtx->WdfInterrupt);
   devCtx->WdfDevice=device;
    status = WdfFdoQueryForInterface(devCtx->WdfDevice,
                                   &GUID_BUS_INTERFACE_STANDARD,
                                   (PINTERFACE) &devCtx->BusInterface,
                                   sizeof(BUS_INTERFACE_STANDARD),
                                   1, // Version
                                   NULL); //InterfaceSpecificData
								   
		bytesRead = devCtx->BusInterface.GetBusData(
                        devCtx->BusInterface.Context,
                         PCI_WHICHSPACE_CONFIG, //READ
                         buffer,
                         FIELD_OFFSET(PCI_COMMON_CONFIG, VendorID),
                         0x40);
	if(bytesRead==0)
	{
	  DbgPrint("[HPMN]:No Configuration Data Read From the Device");
	}
//for(i=0;i<bytesRead;i=i+2)
//{

//DbgPrint("%x%x",buffer[i+1],buffer[i]);
DbgPrint("[HPMN]:Vendor Id:%x\nDevice Id:%x",pPciConfig->VendorID,pPciConfig->DeviceID);
//}								   
///////////////////Device Interface////////////////////
 // status = WdfDeviceCreateDeviceInterface(device, &GUID_DEV_IF_BASIC, NULL);
  //if(!NT_SUCCESS(status))
 // {
  // DbgPrint("[HPMN]:WdfDeviceCreateDeviceInterface\n");
  //  return status;
  //}
  DbgPrint("<--[HPMN]: EvtDeviceAdd\n");
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
  UNREFERENCED_PARAMETER(ResourceList);
  
  DbgPrint("--> [HPMN]:HpmnEvtDevicePrepareHardware\n");
  return status;   
}
 DbgPrint("<-- [HPMN]:HpmnEvtDevicePrepareHardware\n");
 return status;
}

NTSTATUS
HpmnEvtDeviceReleaseHardware(
    IN  WDFDEVICE    Device,
    IN  WDFCMRESLIST ResourcesTranslated
    )
{
    PDEVICE_CONTEXT devCtx = GetDeviceContext(Device);
	
    UNREFERENCED_PARAMETER(ResourcesTranslated);
	DbgPrint("-->[HPMN]:HpmnEvtDeviceReleaseHardware");
	if (devCtx->CSRAddressBar0)
    {
        MmUnmapIoSpace(devCtx->CSRAddressBar0, HPMN_BAR_LENGTH);
        devCtx->CSRAddressBar0 = NULL;
    }

    if (devCtx->CSRAddressBar1)
    {
        MmUnmapIoSpace(devCtx->CSRAddressBar1, HPMN_BAR_LENGTH);
        devCtx->CSRAddressBar1 = NULL;
    }
	
	if (devCtx->CSRAddressBar2)
    {
        MmUnmapIoSpace(devCtx->CSRAddressBar2, HPMN_BAR_LENGTH);
        devCtx->CSRAddressBar2 = NULL;
    }

DbgPrint("<--[HPMN]:HpmnEvtDeviceReleaseHardware");
	return STATUS_SUCCESS;
}
 
VOID
HpmnEvtDeviceIoDefault(
    IN WDFQUEUE  Queue,
    IN WDFREQUEST  Request
    )
{
  UNREFERENCED_PARAMETER(Queue);

  DbgPrint("-->[HPMN]: EvtDeviceIoDefault\n");

  WdfRequestComplete(Request, STATUS_NOT_IMPLEMENTED);

  DbgPrint("<-- [HPMN]:EvtDeviceIoDefault\n");

}
VOID
  HpmnEvtIoDeviceControl (
    IN WDFQUEUE  Queue,
    IN WDFREQUEST  Request,
    IN size_t  OutputBufferLength,
    IN size_t  InputBufferLength,
    IN ULONG  IoControlCode
    )
{

NTSTATUS            status = STATUS_SUCCESS;// Assume success
    PCHAR               inBuf = NULL; // pointer to Input and output buffer
    PCHAR               data = "this String is from Device Driver !!!";
    ULONG               datalen = (ULONG) strlen(data)+1;//Length of data including null
    PCHAR               buffer = NULL;
    size_t               bufSize;
	
	UNREFERENCED_PARAMETER( Queue );
    if(!OutputBufferLength || !InputBufferLength)
      {
        WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
        return;
      }
	DbgPrint("-->[HPMN]: NewEvtDeviceIoControl\n");

	//
    // Determine which I/O control code was specified.
    //

    switch (IoControlCode)
    {
	
	  case IOCTL_HPMN_METHOD_TX:
	  DbgPrint("Check TX IOCTL %x\n", IoControlCode);
	  status = WdfRequestRetrieveInputBuffer(Request, 0, &inBuf, &bufSize);
	
		if(!NT_SUCCESS(status)) {
            status = STATUS_INSUFFICIENT_RESOURCES;
			DbgPrint("STATUS_INSUFFICIENT_RESOURCES");
            break;
        }
		status = WdfRequestRetrieveInputBuffer(Request, 0, &inBuf, &bufSize);
        if(!NT_SUCCESS(status)) {
            status = STATUS_INSUFFICIENT_RESOURCES;
            break;
        }

        ASSERT(bufSize == InputBufferLength);
		PrintChars(inBuf, InputBufferLength);
        status = WdfRequestRetrieveOutputBuffer(Request, 0, &buffer, &bufSize);
        if(!NT_SUCCESS(status)) {
		DbgPrint("WDFRequestRetrieveOutBuffer failed");
		break;
        }

        ASSERT(bufSize == OutputBufferLength);
        
        WdfRequestSetInformation(Request, OutputBufferLength);
		break;
	case IOCTL_HPMN_METHOD_RX:
	    DbgPrint("Check RX IOCTL %x\n", IoControlCode);
	   status = WdfRequestRetrieveInputBuffer(Request, 0, &inBuf, &bufSize);
        if(!NT_SUCCESS(status)) {
            status = STATUS_INSUFFICIENT_RESOURCES;
            break;
        }

        ASSERT(bufSize == InputBufferLength);
        status = WdfRequestRetrieveOutputBuffer(Request, 0, &buffer, &bufSize);
        if(!NT_SUCCESS(status)) {
            break;
        }

        ASSERT(bufSize == OutputBufferLength);

        //
        // Write data to be sent to the user in this buffer
        //
        RtlCopyMemory(buffer, data, OutputBufferLength);

		PrintChars(buffer, datalen);

        WdfRequestSetInformation(Request,
                    OutputBufferLength < datalen? OutputBufferLength: datalen);

        break;
		
		default:

        //
        // The specified I/O control code is unrecognized by this driver.
        //
        status = STATUS_INVALID_DEVICE_REQUEST;
        DbgPrint("ERROR: unrecognized IOCTL %x\n", IoControlCode);
        break;
    }
		WdfRequestComplete( Request, status);
		DbgPrint("<-- [HPMN]:NewEvtDeviceIoControl\n");
 }


VOID
  HpmnEvtIoRead (
    IN WDFQUEUE  Queue,
    IN WDFREQUEST  Request,
    IN size_t  Length
    )
{
  UNREFERENCED_PARAMETER(Queue);
  UNREFERENCED_PARAMETER(Length);

  DbgPrint("--> [HPMN]:EvtIoRead\n");

  WdfRequestComplete(Request, STATUS_NOT_IMPLEMENTED);

  DbgPrint("<--[HPMN]: EvtIoRead\n");

}

VOID
  HpmnEvtIoWrite (
    IN WDFQUEUE  Queue,
    IN WDFREQUEST  Request,
    IN size_t  Length
    )
{
UNREFERENCED_PARAMETER(Queue);
  UNREFERENCED_PARAMETER(Length);

  DbgPrint("-->[HPMN]: EvtIoWrite\n");

  WdfRequestComplete(Request, STATUS_NOT_IMPLEMENTED);

  DbgPrint("<-- [HPMN]:EvtIoWrite\n");
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

  DbgPrint("[HPMN]:<-- EvtDeviceD0Entry\n");

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
//To call Disable Interrupt Routine here before the device shuts down
 DbgPrint("[HPMN]:<-- EvtDeviceD0Exit\n");

  return status;
}




NTSTATUS
HpmnEvtInterruptEnable(
    IN WDFINTERRUPT Interrupt,
    IN WDFDEVICE    Device
    )
{
    DbgPrint("[HPMN]:Enable Interrupt");
	UNREFERENCED_PARAMETER(Device);
	UNREFERENCED_PARAMETER(Interrupt);
	DbgPrint("[HPMN]:Interrupt Enabled");
    return STATUS_SUCCESS;
}

NTSTATUS
HpmnEvtInterruptDisable(
    IN WDFINTERRUPT Interrupt,
    IN WDFDEVICE    Device
    )
{
     DbgPrint("[HPMN]:Disable Interrupt");
    UNREFERENCED_PARAMETER(Device);
	UNREFERENCED_PARAMETER(Interrupt);
	 DbgPrint("[HPMN]:Interrupt Disabled");
    return STATUS_SUCCESS;
}

BOOLEAN
HpmnIsr(
    IN WDFINTERRUPT Interrupt,
    IN ULONG        MessageID
    )
{
  DbgPrint("[HPMN]:Interrupt Isr Is Called");
   UNREFERENCED_PARAMETER(Interrupt);
	UNREFERENCED_PARAMETER(MessageID);
	return TRUE;
}

VOID
HpmnDpc(
    IN WDFINTERRUPT Interrupt,
    IN WDFDEVICE    Device
    )
{
   DbgPrint("[HPMN]:Interrupt Dpc Is Called");
   UNREFERENCED_PARAMETER(Interrupt);
   UNREFERENCED_PARAMETER(Device);
	
 }


VOID
PrintChars(
    __in_ecount(CountChars) PCHAR BufferAddress,
    __in size_t CountChars
    )
{
    if (CountChars) {

        while (CountChars--) {

            if (*BufferAddress > 31
                 && *BufferAddress != 127) {

                KdPrint (( "%c", *BufferAddress) );

            } else {

                KdPrint(( ".") );

            }
            BufferAddress++;
        }
        KdPrint (("\n"));
    }
    return;
}

