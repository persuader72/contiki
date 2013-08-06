#include <stdio.h>
//#include "usb_printf.h"
#include "USB/USB_API/USB_Common/types.h"
#include "USB/usbConstructs.h"
#include <descriptors.h>

//extern uint8_t usb_printf_state;
uint8_t buffer[64];
uint8_t bufPos=0;

int putchar(int c) {
	uint8_t ch = (uint8_t)c;
    //if(usb_printf_state == USB_DISABLED) return -1;
    //if(usb_printf_state == USB_LOCKED) return -1;
    //usb_printf_state = USB_LOCKED;
	buffer[bufPos++]=c;
	if(c=='\n' || bufPos == 64){
		cdcSendDataWaitTilDone(buffer,bufPos, CDC0_INTFNUM, 1);
		bufPos=0;
	}
	//usb_printf_state = USB_ENABLED;
	return c;
}
