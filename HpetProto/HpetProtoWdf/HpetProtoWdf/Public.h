/*++

Module Name:

    public.h

Abstract:

    This module contains the common declarations shared by driver
    and user applications.

Environment:

    user and kernel

--*/

//
// Define an Interface Guid so that app can find the device and talk to it.
//

DEFINE_GUID (GUID_DEVINTERFACE_HpetProtoWdf,0x6bec47b9,0xdba4,0x4f20,0x85,0x2d,0x9e,0xe2,0x69,0xdd,0x7c,0xae);
// {6bec47b9-dba4-4f20-852d-9ee269dd7cae}
