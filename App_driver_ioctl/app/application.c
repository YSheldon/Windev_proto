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
    char OutputBuffer[100];
    char InputBuffer[200];
    BOOL bRc;
    ULONG bytesReturned;

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

    if(FAILED(StringCchCopy(InputBuffer, sizeof(InputBuffer),
               "this String is from User Application; using METHOD_TX"))) {
        return;
    }
	printf("Data in TX:%s",InputBuffer);
    bRc = DeviceIoControl ( hDevice,
                            (DWORD) IOCTL_HPMN_METHOD_TX ,
                            InputBuffer,
                            (DWORD) strlen( InputBuffer )+1,
                            OutputBuffer,
                            sizeof( OutputBuffer),
                            &bytesReturned,
                            NULL
                            );

    if ( !bRc )
    {
        printf ( "Error in DeviceIoControl : : %d", GetLastError());
        return;
    }

    printf("    Number of bytes transfered from OutBuffer: %d\n",
                                    bytesReturned);

    //
    // Performing METHOD_RX
    //

    printf("\nCalling DeviceIoControl METHOD_RX\n");
    if(FAILED(StringCchCopy(InputBuffer, sizeof(InputBuffer),
               "this String is from User Application; using METHOD_OUT_DIRECT"))){
        return;
    }

    memset(OutputBuffer, 0, sizeof(OutputBuffer));

    bRc = DeviceIoControl ( hDevice,
                            (DWORD) IOCTL_HPMN_METHOD_RX,
                            InputBuffer,
                            (DWORD) strlen( InputBuffer )+1,
                            OutputBuffer,
                            sizeof( OutputBuffer),
                            &bytesReturned,
                            NULL
                            );

    if ( !bRc )
    {
        printf ( "Error in DeviceIoControl : : %d", GetLastError());
        return;
    }

    printf("    OutBuffer (%d): %s\n", bytesReturned, OutputBuffer);

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

