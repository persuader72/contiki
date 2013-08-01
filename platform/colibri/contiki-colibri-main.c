#include <contiki.h>
#include <random.h>
#include <dev/serial-line.h>
#include <leds.h>
#include <net/rime.h>

#include <watchdog.h>
#include <msp430.h>
#include <stdint.h>

//#include "usb_printf.h"
#include "dev/mrf49xa.h"
#include "dev/infomem.h"
#include "dev/adc.h"

#include <string.h>

#ifdef SERIAL_LINE_USB
#include "USB/USB_API/USB_Common/types.h"               //Basic Type declarations
#include "USB/USB_API/USB_Common/usb.h"                 //USB-specific functions

#include "F5xx_F6xx_Core_Lib/HAL_UCS.h"
#include "F5xx_F6xx_Core_Lib/HAL_PMM.h"
#else
#include <dev/uart1.h>
#endif

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

//Indicates data has been received without an open rcv operation
//volatile BYTE bCDCDataReceived_event = FALSE;

void radio_setup(void){
	/*uint8_t baud = INFOMEM_STRUCT_A->radio.baudRate == 0xFF ? MRF49XA_57600 : INFOMEM_STRUCT_A->radio.baudRate;
	//uint8_t txpwr = INFOMEM_STRUCT_A->radio.txPower == 0xFF ? MRF49XA_TXPWR_0DB : INFOMEM_STRUCT_A->radio.txPower;
	uint8_t txpwr = INFOMEM_STRUCT_A->radio.txPower == 0xFF ? MRF49XA_TXPWR_7DB : INFOMEM_STRUCT_A->radio.txPower;
	//uint8_t rssi = INFOMEM_STRUCT_A->radio.minRssi == 0xFF ? MRF49XA_RSSI_79DB : INFOMEM_STRUCT_A->radio.minRssi;
	uint8_t band = INFOMEM_STRUCT_A->radio.band == 0xFF ? MRF49XA_BAND_868 : INFOMEM_STRUCT_A->radio.band;
	uint8_t channel = INFOMEM_STRUCT_A->radio.channel > MRF49XA_MAX_CHANNEL ? MRF49XA_DEF_CHANNEL : INFOMEM_STRUCT_A->radio.channel;*/

	//mrf49xa_setDataRate(baud,band);
	//mrf49xa_setTxPwr(txpwr);
	//mrf49xa_setRxRssi(MRF49XA_RSSI_79DB,MRF49XA_LNA_0DB);

	//mrf49xa_init(baud,txpwr,band,channel);
	mrf49xa_init(MRF49XA_57600,MRF49XA_TXPWR_7DB,MRF49XA_BAND_868,MRF49XA_DEF_CHANNEL);

}

static void set_rime_addr(void) {
	int i; rimeaddr_t n_addr;

	memcpy(&n_addr, INFOMEM_STRUCT_A->addresses.rimeAddr, sizeof(rimeaddr_t));
	rimeaddr_set_node_addr(&n_addr);

	PRINTF("Rime started with address ");
	for(i = 0; i < sizeof(n_addr.u8) - 1; i++) PRINTF("%d.",n_addr.u8[i]);
	PRINTF("%d\n", n_addr.u8[i]);
}

#ifdef SERIAL_LINE_USB
void init_clock(void) {
    //enable XT2 pins for F5529
    P5SEL |= 0x0C;

    //use REFO for FLL and ACLK
    //UCSCTL3 = (UCSCTL3 & ~(SELREF_7)) | (SELREF__REFOCLK);
    //UCSCTL4 = (UCSCTL4 & ~(SELA_7)) | (SELA__REFOCLK);

    //MCLK will be driven by the FLL (not by XT2), referenced to the REFO
    //Start the FLL, at the freq indicated by the config
    //constant USB_MCLK_FREQ
    //Init_FLL_Settle(USB_MCLK_FREQ / 1000, USB_MCLK_FREQ / 32768);
    //Start the "USB crystal"
    XT2_Start(XT2DRIVE_3);
}

void usb_printf_init(void) {
    SetVCore(3);
    init_clock();
    //Init USB
    USB_init();
    //Enable various USB event handling routines
    USB_setEnabledEvents(kUSB_VbusOnEvent +
                         kUSB_VbusOffEvent +
                         kUSB_receiveCompletedEvent +
                         kUSB_dataReceivedEvent +
                         kUSB_UsbSuspendEvent +
                         kUSB_UsbResumeEvent +
                         kUSB_UsbResetEvent);
    // See if we're already attached physically to USB, and if so,
    // connect to it. Normally applications don't invoke the event
    // handlers, but this is an exception.

    if(USB_connectionInfo() & kUSB_vbusPresent) {
        //usb_printf_state = USB_ENABLED;
        USB_handleVbusOnEvent();
    }

}

/** Initializes modules and pins */
void msp_init(void) {
	//P1DIR |= BIT5|BIT6|BIT7;
    // Stop the watchdog
    //wdt_stop();
    watchdog_stop();

	WDTCTL = WDTPW | WDTHOLD;
    SFRIE1 &= ~WDTIE;

    // Initialize USB interface
    usb_printf_init();
    // Enable global interrupts
    __enable_interrupt();
}
#endif

int main(void) {
	msp430_cpu_init();
	leds_init();
	leds_off(LEDS_ALL);
#ifdef SERIAL_LINE_USB
    msp_init();
#else
#ifdef SERIAL_LINE_OUTPUT_ENABLED
    uart1_init(115200);
#endif
#endif
    clock_init();
    rtimer_init();
    random_init(INFOMEM_STRUCT_A->addresses.nodeId[1]);
    process_init();
    process_start(&etimer_process, NULL);
    ctimer_init();

    adc_init();
    set_rime_addr();
    radio_setup();

#ifdef SERIAL_LINE_INPUT_ENABLED
#ifdef SERIAL_LINE_USB
    serial_line_init();
#else
    uart1_set_input(serial_line_input_byte);
    serial_line_init();
#endif
#endif

    autostart_start(autostart_processes);
    // Main loop
    while(1) {  
        int r;
        do {
          watchdog_periodic();
          r = process_run();
        } while(r > 0);

        if(process_nevents() != 0 /*|| uart1_active()*/) {

        } else {
            watchdog_stop();
            //__bis_SR_register(GIE | LPM3_bits);
            watchdog_start();
        }

        /*if(events_available()) {
            process_events();
        }*/
    }
}

/** Checks if there are events available to be processed
 * The currently supported events are:
 * - There are USB packets available
 *
 *   These events are checked quite frequently because the main
 *   loop is constantly checking them. Anyhow, whenever we need to process
 *   a lengthy task, the main loop will cease to check for an event. 
 *   This doesn't mean that the events will be dropped, but they will 
 *   not be managed instantly.
 */
/*uint8_t events_available(void) {
    return (
        //data received event
        bCDCDataReceived_event
        );
}*/

/** Process then events queried in the \ref events_available() function */
/*void process_events(void) {
    // Data received from USB
    if(bCDCDataReceived_event == TRUE) {
        usb_receive_string();
    }   
}*/
