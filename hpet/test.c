#include <ntddk.h>     
#include "wdf.h"
NTSTATUS DriverEntry(PDRIVER_OBJECT DriverObject, PUNICODE_STRING RegistryPath)    
{
NTSTATUS               status = STATUS_SUCCESS;
	WDF_DRIVER_CONFIG      config;
    WDF_OBJECT_ATTRIBUTES  attrib;
    WDFDRIVER              driver;
    DbgPrint("Hello World\n");  
	status = WdfDriverCreate(
                      DriverObject,
                      RegistryPath,
                      WDF_NO_OBJECT_ATTRIBUTES,
                      &config,
                      WDF_NO_HANDLE);
    if(status!=STATUS_SUCCESS)
	{
	 DbgPrint("Wdf Failed to Create Device %x\n",status);
    }
	else
	DbgPrint("WDFCREATE SUCCESSFULL \n"); 
    return STATUS_SUCCESS;
}  