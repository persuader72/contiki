#include <stdio.h>
//#include "usb_printf.h"
#include "USB/USB_API/USB_Common/types.h"
#include "USB/usbConstructs.h"
#include <descriptors.h>

//extern uint8_t usb_printf_state;

int putchar(int c) {
	uint8_t ch = (uint8_t)c;
    //if(usb_printf_state == USB_DISABLED) return -1;
    //if(usb_printf_state == USB_LOCKED) return -1;
    //usb_printf_state = USB_LOCKED;
	cdcSendDataWaitTilDone(&ch,1, CDC0_INTFNUM, 1);
	//usb_printf_state = USB_ENABLED;
	return c;
}
