#include <DriverSpecs.h>
__user_code  

#include <windows.h>

#pragma warning(disable:4201)  // nameless struct/union
#include <winioctl.h>
#pragma warning(default:4201)

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <limits.h>
#include <strsafe.h>
#include "public.h"

#include <conio.h>
#include <fcntl.h>
#include <io.h>

VOID
DoIoctls(
    HANDLE hDevice
    );

VOID __cdecl
main(
    __in ULONG argc,
    __in_ecount(argc) PCHAR argv[]
    )
{
    HANDLE   hDevice;
    DWORD    errNum = 0;
  //  CHAR     driverLocation [MAX_PATH];
  //  BOOL     ok;
    HMODULE  library = NULL;
    LONG     error;
    

  

    //
    // open the device
    //
            hDevice = CreateFile( DEVICE_NAME,
                              GENERIC_READ | GENERIC_WRITE,
                              0,
                              NULL,
                              CREATE_ALWAYS,
                              FILE_ATTRIBUTE_NORMAL,
                              NULL );

        if (hDevice == INVALID_HANDLE_VALUE) {
            printf ( "Error: CreatFile Failed : %d\n", GetLastError());
            return;
        }
    

    DoIoctls(hDevice);

   /* do {

        if(!DoFileReadWrite(hDevice)) {
            break;
        }

        if(!G_fLoop) {
            break;
        }
        Sleep(1000); // sleep for 1 sec.

    } WHILE (TRUE);
*/
    //
    // Close the handle to the device before unloading the driver.
    //
    CloseHandle ( hDevice );
    return;
}


VOID
DoIoctls(
    HANDLE hDevice
    )
{
    char OutputBuffer[100000];
    char InputBuffer[100000];
	char cKey = 0;
    BOOL bRc;
    ULONG bytesReturned;
	LARGE_INTEGER startTime,endTime,frequency;
	ULONGLONG Rxdif,Txdif,TxTotal = 0,RxTotal = 0,count = 0;
	LONGLONG multiplier;
	
	QueryPerformanceFrequency(&frequency);
	multiplier = 1000000000LL/frequency.QuadPart;
	
	printf("Frequency %lld Mult:%lld\n",frequency.QuadPart,multiplier);
    //
    // Printing Input & Output buffer pointers and size
    //

    printf("InputBuffer Pointer = %p, BufLength = %d\n", InputBuffer,
                        sizeof(InputBuffer));
    printf("OutputBuffer Pointer = %p BufLength = %d\n", OutputBuffer,
                                sizeof(OutputBuffer));
    
    //
    // Performing METHOD_TX
    //

    printf("\nCalling DeviceIoControl METHOD_TX\n");

  //  if(FAILED(StringCchCopy(InputBuffer, sizeof(InputBuffer),
    //           "this String is from User Application; using METHOD_TX"))) {
   //     return;
   // }
   
	memset(InputBuffer,0xAF,200);
	//printf("Data in TX:%s",InputBuffer);
	while (cKey != 0x1B && count != 1000000)
	{
		if( _kbhit() )
        {
            cKey    = (BYTE)_getch();
		}
		count++;
		
		QueryPerformanceCounter(&startTime);
		bRc = DeviceIoControl ( hDevice,
                            (DWORD) IOCTL_HPMN_METHOD_TX ,
                            InputBuffer,
                            (DWORD) strlen( InputBuffer )+1,
                            OutputBuffer,
                            sizeof( OutputBuffer),
                            &bytesReturned,
                            NULL
                            );
		QueryPerformanceCounter(&endTime);
		
		Txdif = (endTime.QuadPart - startTime.QuadPart);
		TxTotal += Txdif;
		if ( !bRc )
		{
			printf ( "Error in DeviceIoControl : : %d", GetLastError());
			return;
		}

		//printf("Number of bytes transfered from OutBuffer: %d\n",
										//strlen( InputBuffer )+1);
		
		//printf("Time taken is %lld us\n",dif);									

		//
		// Performing METHOD_RX
		//

		//printf("\nCalling DeviceIoControl METHOD_RX\n");
		//if(FAILED(StringCchCopy(InputBuffer, sizeof(InputBuffer),
		//		   "this String is from User Application; using METHOD_OUT_DIRECT"))){
		//	return;
		//}

		memset(OutputBuffer, 0, sizeof(OutputBuffer));
		QueryPerformanceCounter(&startTime);
		bRc = DeviceIoControl ( hDevice,
								(DWORD) IOCTL_HPMN_METHOD_RX,
								InputBuffer,
								(DWORD) strlen( InputBuffer )+1,
								OutputBuffer,
								sizeof( OutputBuffer),
								&bytesReturned,
								NULL
								);
		QueryPerformanceCounter(&endTime);
		
		Rxdif = (endTime.QuadPart - startTime.QuadPart);
		RxTotal += Rxdif;
		if ( !bRc )
		{
			printf ( "Error in DeviceIoControl : : %d", GetLastError());
			return;
		}

		//	printf("OutBuffer(%d): %x\n", bytesReturned, OutputBuffer[0]);
	
	}
	
    printf("Time taken for Rx %lld us  Tx %lld us in %lld TotalRx:%lld TTx:%lld\n", \
														(((RxTotal)/count)*multiplier)/1000, \
														(((TxTotal)/count)*multiplier)/1000,count, \
														(((RxTotal))*multiplier)/1000, \
														(((TxTotal))*multiplier)/1000);	
    return;

}


/*BOOLEAN
DoFileReadWrite(
    HANDLE HDevice
    )
{
    ULONG bufLength, index;
    PUCHAR readBuf = NULL;
    PUCHAR writeBuf = NULL;
    BOOLEAN ret;
    ULONG   bytesWritten, bytesRead;

    //
    // Seed the random-number generator with current time so that
    // the numbers will be different every time we run.
    //
    srand( (unsigned)time( NULL ) );

    //
    // rand function returns a pseudorandom integer in the range 0 to RAND_MAX
    // (0x7fff)
    //
    bufLength = rand();
    //
    // Try until the bufLength is not zero.
    //
    while(bufLength == 0) {
        bufLength = rand();
    }

    //
    // Allocate a buffer of that size to use for write operation.
    //
    writeBuf = HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, bufLength);
    if(!writeBuf) {
        ret = FALSE;
        goto End;
    }
    //
    // Fill the buffer with randon number less than UCHAR_MAX.
    //
    index = bufLength;
    while(index){
        writeBuf[index-1] = (UCHAR) rand() % UCHAR_MAX;
        index--;
    }

    printf("Write %d bytes to file\n", bufLength);

    //
    // Tell the driver to write the buffer content to the file from the
    // begining of the file.
    //

    if (!WriteFile(HDevice,
                  writeBuf,
                  bufLength,
                  &bytesWritten,
                  NULL)) {

        printf("ReadFile failed with error 0x%x\n", GetLastError());

        ret = FALSE;
        goto End;

    }

    //
    // Allocate another buffer of same size.
    //
    readBuf = HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, bufLength);
    if(!readBuf) {

        ret = FALSE;
        goto End;
    }

    printf("Read %d bytes from the same file\n", bufLength);

    //
    // Tell the driver to read the file from the begining.
    //
    if (!ReadFile(HDevice,
                  readBuf,
                  bufLength,
                  &bytesRead,
                  NULL)) {

        printf("Error: ReadFile failed with error 0x%x\n", GetLastError());

        ret = FALSE;
        goto End;

    }

    //
    // Now compare the readBuf and writeBuf content. They should be the same.
    //

    if(bytesRead != bytesWritten) {
        printf("bytesRead(%d) != bytesWritten(%d)\n", bytesRead, bytesWritten);
        ret = FALSE;
        goto End;
    }

    if(memcmp(readBuf, writeBuf, bufLength) != 0){
        printf("Error: ReadBuf and WriteBuf contents are not the same\n");
        ret = FALSE;
        goto End;
    }

    ret = TRUE;

End:

    if(readBuf){
        HeapFree (GetProcessHeap(), 0, readBuf);
    }

    if(writeBuf){
        HeapFree (GetProcessHeap(), 0, writeBuf);
    }

    return ret;


}
*/

