//
// UIOTEST.C
//
// Test program for ndisprot.sys
//
// usage: UIOTEST [options] <devicename>
//
// options:
//        -e: Enumerate devices
//        -r: Read
//        -w: Write (default)
//        -l <length>: length of each packet (default: %d)\n", PacketLength
//        -n <count>: number of packets (defaults to infinity)
//        -m <MAC address> (defaults to local MAC)
//

#pragma warning(disable:4201)   // nameless struct/union
#pragma warning(disable:4127)   // conditional expression is constant

#include <windows.h>
#include <winioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <ctype.h>
#include <malloc.h>
#include <conio.h>
#include <winerror.h>
#include <winsock.h>

#include <ntddndis.h>
#include "protuser.h"

// this is needed to prevent comppiler from complaining about 
// pragma prefast statements below
#ifndef _PREFAST_
    #pragma warning(disable:4068)
#endif

#ifndef NDIS_STATUS
#define NDIS_STATUS     ULONG
#endif

#if DBG
#define DEBUGP(stmt)    printf stmt
#else
#define DEBUGP(stmt)
#endif

#define PRINTF(stmt)    printf stmt

#ifndef MAC_ADDR_LEN
#define MAC_ADDR_LEN                    6
#endif

#define MAX_NDIS_DEVICE_NAME_LEN        256
#define DATA_LENGTH 60

CHAR            NdisProtDevice[] = "\\\\.\\\\NdisProt";
CHAR *          pNdisProtDevice = &NdisProtDevice[0];


UCHAR aframe[DATA_LENGTH] = {0x01,0x11,0x1e,0x00,0x00,0x02,0x00,0x27,0x0e,0x37,0x7b,0xfa,0x88,0xab,0x04,0xff
,0x01,0xfd,0x01,0x00,0x00,0x00,0x04,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

BOOLEAN         DoEnumerate = FALSE;
BOOLEAN         DoReads = FALSE,DoWrites = FALSE;
INT             NumberOfPackets = -1;
ULONG           PacketLength = 16;
UCHAR           SrcMacAddr[MAC_ADDR_LEN];
UCHAR           DstMacAddr[MAC_ADDR_LEN];
BOOLEAN         bDstMacSpecified = FALSE;
CHAR *          pNdisDeviceName = "JUNK";
USHORT          EthType = 0xab88;
BOOLEAN         bUseFakeAddress = FALSE;
UCHAR           FakeSrcMacAddr[MAC_ADDR_LEN] = {0};
static ULONGLONG Rxdif,Txdif,TxTotal = 0,RxTotal = 0,txcount = 0, rxcount=0;
LONGLONG multiplier;
	
#include <pshpack1.h>

typedef struct _ETH_HEADER
{
    UCHAR       DstAddr[MAC_ADDR_LEN];
    UCHAR       SrcAddr[MAC_ADDR_LEN];
    USHORT      EthType;
} ETH_HEADER, *PETH_HEADER;

#include <poppack.h>


VOID
PrintUsage()
{
    PRINTF(("usage: PROTTEST [options] <devicename>\n"));
    PRINTF(("options:\n"));
    PRINTF(("       -e: Enumerate devices\n"));
    PRINTF(("       -r: Read\n"));
    PRINTF(("       -w: Write (default)\n"));
    PRINTF(("       -l <length>: length of each packet (default: %d)\n", PacketLength));
    PRINTF(("       -n <count>: number of packets (defaults to infinity)\n"));
    PRINTF(("       -m <MAC address> (defaults to local MAC)\n"));
    PRINTF(("       -f Use a fake address to send out the packets.\n"));
    

}

BOOL
GetOptions(
    INT                             argc,
    __in_ecount(argc) CHAR          *argv[]
)
{
    BOOL        bOkay;
    INT         i, j, increment;
    CHAR        *pOption;
    ULONG       DstMacAddrUlong[MAC_ADDR_LEN];
    INT         RetVal;

    bOkay = TRUE;

    do
    {
        if (argc < 2)
        {
            PRINTF(("Missing <devicename> argument\n"));
            bOkay = FALSE;
            break;
        }

        i = 1;
#pragma prefast(suppress: 12004, "argc is always >= 1")           
        while (i < argc)
        {
            increment = 1;
            pOption = argv[i];

            if ((*pOption == '-') || (*pOption == '/'))
            {
                pOption++;
                if (*pOption == '\0')
                {
                    DEBUGP(("Badly formed option\n"));
                    return (FALSE);
                }
            }
            else
            {
                break;
            }

            switch (*pOption)
            {
                case 'e':
                    DoEnumerate = TRUE;
                    break;
                
                case 'f':
                    bUseFakeAddress = TRUE;
                    break;

                case 'r':
                    DoReads = TRUE;
                    break;

                case 'w':
                    DoWrites = TRUE;
                    break;

                case 'l':

                    if (i+1 < argc-1)
                    {
                        RetVal = atoi(argv[i+1]);
                        if (RetVal != 0)
                        {
                            PacketLength = RetVal;
                            DEBUGP((" Option: PacketLength = %d\n", PacketLength));
                            increment = 2;
                            break;
                        }
                    }    
                    PRINTF(("Option l needs PacketLength parameter\n"));
                    return (FALSE);
                
                case 'n':

                    if (i+1 < argc-1)
                    {
                        RetVal = atoi(argv[i+1]);
                        if (RetVal != 0)
                        {
                            NumberOfPackets = RetVal;
                            DEBUGP((" Option: NumberOfPackets = %d\n", NumberOfPackets));
                            increment = 2;
                            break;
                        }
                    }
                    PRINTF(("Option n needs NumberOfPackets parameter\n"));
                    return (FALSE);

                case 'm':

                    if (i+1 < argc-1)
                    {
                        RetVal = sscanf(argv[i+1], "%2x:%2x:%2x:%2x:%2x:%2x",
                                    &DstMacAddrUlong[0],
                                    &DstMacAddrUlong[1],
                                    &DstMacAddrUlong[2],
                                    &DstMacAddrUlong[3],
                                    &DstMacAddrUlong[4],
                                    &DstMacAddrUlong[5]);

                        if (RetVal == 6)
                        {
                            for (j = 0; j < MAC_ADDR_LEN; j++)
                            {
                                DstMacAddr[j] = (UCHAR)DstMacAddrUlong[j];
                            }
    
                            DEBUGP((" Option: Dest MAC Addr: %02x:%02x:%02x:%02x:%02x:%02x\n",
                                DstMacAddr[0],
                                DstMacAddr[1],
                                DstMacAddr[2],
                                DstMacAddr[3],
                                DstMacAddr[4],
                                DstMacAddr[5]));
                            bDstMacSpecified = TRUE;

                            increment = 2;
                            break;
                        }
                    }
                        
                    PRINTF(("Option m needs MAC address parameter\n"));
                    return (FALSE);
                
                case '?':
                    return (FALSE);

                default:
                    PRINTF(("Unknown option %c\n", *pOption));
                    return (FALSE);
            }

            i+= increment;
        }

        pNdisDeviceName = argv[argc-1];
        
    }
    while (FALSE);

    return (bOkay);
}


HANDLE
OpenHandle(
    __in __nullterminated CHAR    *pDeviceName
)
{
    DWORD   DesiredAccess;
    DWORD   ShareMode;
    LPSECURITY_ATTRIBUTES   lpSecurityAttributes = NULL;

    DWORD   CreationDistribution;
    DWORD   FlagsAndAttributes;
    HANDLE  Handle;
    DWORD   BytesReturned;

    DesiredAccess = GENERIC_READ|GENERIC_WRITE;
    ShareMode = 0;
    CreationDistribution = OPEN_EXISTING;
    FlagsAndAttributes = FILE_ATTRIBUTE_NORMAL;

    Handle = CreateFile(
                pDeviceName,
                DesiredAccess,
                ShareMode,
                lpSecurityAttributes,
                CreationDistribution,
                FlagsAndAttributes,
                NULL
                );
    if (Handle == INVALID_HANDLE_VALUE)
    {
        DEBUGP(("Creating file failed, error %x\n", GetLastError()));
        return Handle;
    }
    //
    //  Wait for the driver to finish binding.
    //
    if (!DeviceIoControl(
                Handle,
                IOCTL_NDISPROT_BIND_WAIT,
                NULL,
                0,
                NULL,
                0,
                &BytesReturned,
                NULL))
    {
        DEBUGP(("IOCTL_NDISIO_BIND_WAIT failed, error %x\n", GetLastError()));
        CloseHandle(Handle);
        Handle = INVALID_HANDLE_VALUE;
    }

    return (Handle);
}


BOOL
OpenNdisDevice(     
    HANDLE                          Handle,
    __in __nullterminated CHAR      *pDeviceName
)
{
    WCHAR   wNdisDeviceName[MAX_NDIS_DEVICE_NAME_LEN];
    INT     wNameLength;
    INT     NameLength = strlen(pDeviceName);
    DWORD   BytesReturned;
    INT     i;


    //
    // Convert to unicode string - non-localized...
    //
    wNameLength = 0;
    for (i = 0; i < NameLength && i < MAX_NDIS_DEVICE_NAME_LEN-1; i++)
    {
        wNdisDeviceName[i] = (WCHAR)pDeviceName[i];
        wNameLength++;
    }
#pragma prefast(suppress: __WARNING_POTENTIAL_BUFFER_OVERFLOW, "wNdisDeviceName is bounded by the check above");    
    wNdisDeviceName[i] = L'\0';

    DEBUGP(("Trying to access NDIS Device: %ws\n", wNdisDeviceName));

    return (DeviceIoControl(
                Handle,
                IOCTL_NDISPROT_OPEN_DEVICE,
                (LPVOID)&wNdisDeviceName[0],
                wNameLength*sizeof(WCHAR),
                NULL,
                0,
                &BytesReturned,
                NULL));

}


BOOL
GetSrcMac(
    HANDLE  Handle,
    PUCHAR  pSrcMacAddr
    )
{
    DWORD       BytesReturned;
    BOOLEAN     bSuccess;
    UCHAR       QueryBuffer[sizeof(NDISPROT_QUERY_OID) + MAC_ADDR_LEN];
    PNDISPROT_QUERY_OID  pQueryOid;

    
    DEBUGP(("Trying to get src mac address\n"));

    pQueryOid = (PNDISPROT_QUERY_OID)&QueryBuffer[0];
    pQueryOid->Oid = OID_802_3_CURRENT_ADDRESS;
    pQueryOid->PortNumber = 0;
    
    bSuccess = (BOOLEAN)DeviceIoControl(
                            Handle,
                            IOCTL_NDISPROT_QUERY_OID_VALUE,
                            (LPVOID)&QueryBuffer[0],
                            sizeof(QueryBuffer),
                            (LPVOID)&QueryBuffer[0],
                            sizeof(QueryBuffer),
                            &BytesReturned,
                            NULL);

    if (bSuccess)
    {
        DEBUGP(("GetSrcMac: IoControl success, BytesReturned = %d\n",
                BytesReturned));

#pragma prefast(suppress:__WARNING_WRITE_OVERRUN __WARNING_BUFFER_OVERRUN_NONSTACK __WARNING_READ_OVERRUN __WARNING_BUFFER_OVERRUN, "enough space allocated in QueryBuffer")
        memcpy(pSrcMacAddr, pQueryOid->Data, MAC_ADDR_LEN);                    
    }
    else
    {
        DEBUGP(("GetSrcMac: IoControl failed: %d\n", GetLastError()));
    }

    return (bSuccess);
}



BOOLEAN
DoReadProc(
    HANDLE  Handle
    )
{
    PUCHAR      pReadBuf = NULL;
    static INT         ReadCount = 0;
    BOOLEAN     bSuccess,ret;
    ULONG       BytesRead;
	LARGE_INTEGER startTime,endTime;
	
  //  DEBUGP(("DoReadProc\n"));

    do
    {
        pReadBuf = malloc(PacketLength);

        if (pReadBuf == NULL)
        {
            PRINTF(("DoReadProc: failed to alloc %d bytes\n", PacketLength));
            break;
        }

        ReadCount = 0; 
        while (TRUE)
        {
#pragma prefast(suppress: 8193, "bSuccess is examined below")

			//QueryPerformanceCounter(&startTime);
            bSuccess = (BOOLEAN)ReadFile(
                                    Handle,
                                    (LPVOID)pReadBuf,
                                    PacketLength,
                                    &BytesRead,
                                    NULL);
			//QueryPerformanceCounter(&endTime);
            if (!bSuccess)
            {
                PRINTF(("DoReadProc: ReadFile failed on Handle %p, error %x\n",
                        Handle, GetLastError()));
                ret = FALSE;
                break;
            }

            if(pReadBuf[5] == 0x01)
            {
            	ret = TRUE;
            	//break;
            }
            else
            {
            	ret = FALSE;
            	///;
            }
			//rxcount++;
			//Rxdif = (endTime.QuadPart - startTime.QuadPart);
			//RxTotal += Rxdif;
           ReadCount++;

        //   DEBUGP(("DoReadProc: read pkt # %d, %d bytes\n", ReadCount, BytesRead));
           break;
           // if ((NumberOfPackets != -1) && (ReadCount == NumberOfPackets))
           // {
           //     break;
           // }
        }
    }
    while (FALSE);

    if (pReadBuf)
    {
        free(pReadBuf);
    }

    return ret;;
    //PRINTF(("DoReadProc finished: read %d packets\n", ReadCount));

}


VOID
DoWriteProc(
    HANDLE  Handle
    )
{
    PUCHAR      pWriteBuf = NULL;
    PUCHAR      pData;
    UINT        i;
    static INT         SendCount;
    PETH_HEADER pEthHeader;
    DWORD       BytesWritten;
    BOOLEAN     bSuccess;
	LARGE_INTEGER startTime,endTime;
   // DEBUGP(("DoWriteProc\n"));
    SendCount = 0;

    do
    {
        pWriteBuf = malloc(PacketLength);

        if (pWriteBuf == NULL)
        {
            DEBUGP(("DoWriteProc: Failed to malloc %d bytes\n", PacketLength));
            break;
        }
        pEthHeader = (PETH_HEADER)pWriteBuf;
#pragma prefast(suppress: __WARNING_POTENTIAL_BUFFER_OVERFLOW, "pWriteBuf is PacketLength(100 bytes long");        
        pEthHeader->EthType = EthType;
        
        if (bUseFakeAddress)
        {
            memcpy(pEthHeader->SrcAddr, FakeSrcMacAddr, MAC_ADDR_LEN);
        }
        else
        {
            memcpy(pEthHeader->SrcAddr, SrcMacAddr, MAC_ADDR_LEN);
        }
        
        memcpy(pEthHeader->DstAddr, DstMacAddr, MAC_ADDR_LEN);

        pData = (PUCHAR)(pEthHeader + 1);
        for (i = 0; i < PacketLength - sizeof(ETH_HEADER); i++)
        {
            *pData++ = (UCHAR)i;
        }

        memcpy(&pWriteBuf[14],&aframe[14],(DATA_LENGTH-15));
        //SendCount = 0;
        
        //while (TRUE)
        {
          //  QueryPerformanceCounter(&startTime);
            bSuccess = (BOOLEAN)WriteFile(
                                    Handle,
                                    pWriteBuf,
                                    PacketLength,
                                    &BytesWritten,
                                    NULL);
									
         //   QueryPerformanceCounter(&endTime);
							
            if (!bSuccess)
            {
                PRINTF(("DoWriteProc: WriteFile failed on Handle %p %x\n", Handle,GetLastError()));
                break;
            }
			//	txcount++;
			//Txdif = (endTime.QuadPart - startTime.QuadPart);
			//TxTotal += Txdif;
            SendCount++;
            
            //DEBUGP(("DoWriteProc: sent %d bytes\n", BytesWritten));

            //if ((NumberOfPackets != -1) && (SendCount == NumberOfPackets))
           // {
           //     break;
          //  }
        }

    }
    while (FALSE);

    if (pWriteBuf)
    {
        free(pWriteBuf);
    }

   // PRINTF(("DoWriteProc: finished sending %d packets of %d bytes each\n",
     //       SendCount, PacketLength));
}

VOID
EnumerateDevices(
    HANDLE  Handle
    )
{
    typedef __declspec(align(MEMORY_ALLOCATION_ALIGNMENT)) QueryBindingCharBuf;        
    QueryBindingCharBuf		Buf[1024];
    DWORD       		BufLength = sizeof(Buf);
    DWORD       		BytesWritten;
    DWORD       		i;
    PNDISPROT_QUERY_BINDING 	pQueryBinding;

    pQueryBinding = (PNDISPROT_QUERY_BINDING)Buf;

    i = 0;
    for (pQueryBinding->BindingIndex = i;
         /* NOTHING */;
         pQueryBinding->BindingIndex = ++i)
    {
#pragma prefast(suppress: __WARNING_READ_OVERRUN, "Okay to read sizeof(NDISPROT_QUERY_BINDING) from pQueryBinding");    
        if (DeviceIoControl(
                Handle,
                IOCTL_NDISPROT_QUERY_BINDING,
                pQueryBinding,
                sizeof(NDISPROT_QUERY_BINDING),
                Buf,
                BufLength,
                &BytesWritten,
                NULL))
        {
            PRINTF(("%2d. %ws\n     - %ws\n",
                pQueryBinding->BindingIndex,
                (WCHAR *)((PUCHAR)pQueryBinding + pQueryBinding->DeviceNameOffset),
                (WCHAR *)((PUCHAR )pQueryBinding + pQueryBinding->DeviceDescrOffset)));

            memset(Buf, 0, BufLength);
        }
        else
        {
            ULONG   rc = GetLastError();
            if (rc != ERROR_NO_MORE_ITEMS)
            {
                PRINTF(("EnumerateDevices: terminated abnormally, error %d\n", rc));
            }
            break;
        }
    }
}




VOID __cdecl
main(
    INT                             argc,
    __in_ecount(argc) LPSTR         *argv
)
{
    HANDLE      DeviceHandle;
    CHAR		cKey = 0;
	LARGE_INTEGER frequency;
	int count = 0;
    DeviceHandle = INVALID_HANDLE_VALUE;
	QueryPerformanceFrequency(&frequency);
	multiplier = 1000000000LL/frequency.QuadPart;
	printf("Frequency %lld Mult:%lld\n",frequency.QuadPart,multiplier);
    do
    {
        if (!GetOptions(argc, argv))
        {
            PrintUsage();
            break;
        }

        DeviceHandle = OpenHandle(pNdisProtDevice);

        if (DeviceHandle == INVALID_HANDLE_VALUE)
        {
            PRINTF(("Failed to open %s\n", pNdisProtDevice));
            break;
        }

        if (DoEnumerate)
        {
            EnumerateDevices(DeviceHandle);
            break;
        }

        if (!OpenNdisDevice(DeviceHandle, pNdisDeviceName))
        {
            PRINTF(("Failed to access %s\n", pNdisDeviceName));
            break;
        }

        DEBUGP(("Opened device %s successfully!\n", pNdisDeviceName));

        if (!GetSrcMac(DeviceHandle, SrcMacAddr))
        {
            PRINTF(("Failed to obtain local MAC address\n"));
            break;
        }


        DEBUGP(("Got local MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
                    SrcMacAddr[0],
                    SrcMacAddr[1],
                    SrcMacAddr[2],
                    SrcMacAddr[3],
                    SrcMacAddr[4],
                    SrcMacAddr[5]));

        if (!bDstMacSpecified)
        {
            memcpy(DstMacAddr, SrcMacAddr, MAC_ADDR_LEN);
        }

        if (DoReads)
        {
            DoReadProc(DeviceHandle);
			printf("Time taken for Rx %lld us in %lld TotalRx:%lld\n", \
														(((RxTotal)/rxcount)*multiplier)/1000, \
														 rxcount, \
														(((RxTotal))*multiplier)/1000);	
        }
        else if (DoWrites)
        {
            DoWriteProc(DeviceHandle);
           // DoReadProc(DeviceHandle);
			//printf("Time taken for Tx %lld us in %lld TTx:%lld\n", \
														(((TxTotal)/txcount)*multiplier)/1000,txcount, \
														(((TxTotal))*multiplier)/1000);	
        }
        else
        {
        	while(cKey != 0x1B)
        	{
        		if(_kbhit())
        		{
        			cKey = (char)_getch();
        		}

				if(DoReadProc(DeviceHandle))
				{
					DoWriteProc(DeviceHandle);
				}
				count++;
				if ((NumberOfPackets != -1) && (count == NumberOfPackets))
			    {
				  break;
				}
        	}
        }

    }
    while (FALSE);

    if (DeviceHandle != INVALID_HANDLE_VALUE)
    {
        CloseHandle(DeviceHandle);
    }
	
	
}

