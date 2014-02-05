
// {EDCCB7CD-86AC-485e-B30C-83AA7C26EF0C}
DEFINE_GUID(GUID_DEV_IF_BASIC, 
0xedccb7cd, 0x86ac, 0x485e, 0xb3, 0xc, 0x83, 0xaa, 0x7c, 0x26, 0xef, 0xc);

#define HPMNIO_TYPE 40001
//
// The IOCTL function codes from 0x800 to 0xFFF are for customer use.
//
#define IOCTL_HPMN_METHOD_TX CTL_CODE( HPMNIO_TYPE, 0x900, METHOD_IN_DIRECT, FILE_ANY_ACCESS  )

#define IOCTL_HPMN_METHOD_RX CTL_CODE( HPMNIO_TYPE, 0x901, METHOD_OUT_DIRECT , FILE_ANY_ACCESS  )

//#define IOCTL_HPMN_METHOD_TX 	CTL_CODE( HPMNIO_TYPE, 0x900, METHOD_BUFFERED, FILE_ANY_ACCESS  )

//#define IOCTL_HPMN_METHOD_RX 	CTL_CODE( HPMNIO_TYPE, 0x901, METHOD_BUFFERED, FILE_ANY_ACCESS  )


#define DRIVER_FUNC_INSTALL     0x01
#define DRIVER_FUNC_REMOVE      0x02

#define DRIVER_NAME       "HPMN"
#define DEVICE_NAME       "\\\\.\\HPMN\\hpmn.log"


