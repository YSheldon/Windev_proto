/**
********************************************************************************
\file   ndisInc.h

\brief  Ndis minipor driver main include file

Specifies global structure and definition for the routines used by the
NDIS miniport prototype.

*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2014 Kalycito Infotech Private Limited
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of the copyright holders nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/
#ifndef _INC_ndisInc_H_
#define _INC_ndisInc_H_

#include <ndis.h>
#include <ntddk.h>

#define __DRIVER_NAME               "82573 PROTOYPE"

#define OPLKDEV_NAME_STRING          L"\\Device\\HPMN"
#define OPLK_SYMBOLIC_NAME_STRING    L"\\DosDevices\\HPMN"

#define VENDOR_ID                    0x10ee //Xilinx vendor ID //TODO: change this
#define DEVICE_ID                    0x0505 //Xilinx Device ID //todo: change this

#define OPLKMP_MAJOR_DRIVER_VERSION     0x06
#define OPLKMP_MINOR_DRIVER_VERISON     0x00

#define OPLKMP_NDIS_MAJOR_VERSION       6
#define OPLKMP_NDIS_MINOR_VERSION       0

#define MINIPORT_TIMER_TAG  'TxuM'



/////////////////////////////////////////////////////////////////////////////////
//                          FUNCTION PROTOTYPES                                //
/////////////////////////////////////////////////////////////////////////////////


DRIVER_INITIALIZE DriverEntry;

DRIVER_DISPATCH miniport_ioDispatch;

DRIVER_DISPATCH miniport_deviceIoControl;

MINIPORT_SET_OPTIONS miniport_setOptions;

MINIPORT_INITIALIZE miniport_initialize;

MINIPORT_HALT miniport_halt;

MINIPORT_UNLOAD miniport_unload;

MINIPORT_PAUSE miniport_pause;

MINIPORT_RESTART miniport_restart;

MINIPORT_CHECK_FOR_HANG miniport_checkForHang;

MINIPORT_OID_REQUEST miniport_oidRequest;

MINIPORT_SEND_NET_BUFFER_LISTS miniport_sendNetBufferLists;

MINIPORT_RETURN_NET_BUFFER_LISTS miniport_returnNetBufferLists;

MINIPORT_CANCEL_SEND miniport_cancelSendNetBufferLists;

MINIPORT_DEVICE_PNP_EVENT_NOTIFY miniport_pnPEventNotify;

MINIPORT_SHUTDOWN miniport_shutdown;

MINIPORT_CANCEL_OID_REQUEST miniport_cancelOidRequest;

MINIPORT_RESET miniport_reset;

MINIPORT_PROCESS_SG_LIST miniport_sgListHandler;

NDIS_TIMER_FUNCTION timerDpc;
#endif