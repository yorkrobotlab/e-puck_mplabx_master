#include "p30F6014A.h"

#define IR_RECIEVER

#include "stdio.h"
#include "string.h"
#include "math.h"
#include <time.h>
#include <uart/e_uart_char.h>
#include <a_d/advance_ad_scan/e_ad_conv.h>

#include "memory.h"
char buffer[BUFFER_SIZE];
char c;

#ifdef IR_RECIEVER
#include <motor_led/advance_one_timer/e_remote_control.h>
#define SPEED_IR 600
#endif 

#include "Asercom.h"


#define PI 3.14159265358979

#define uart2_send_static_text(msg) { e_send_uart2_char(msg,sizeof(msg)-1); while(e_uart2_sending()); }

int main() {


	//system initialization
	e_init_port();    // configure port pins      
	e_start_agendas_processing();
    e_init_motors();
	e_init_uart2(BAUD115200);   // initialize UART to 115200 Kbaud
	e_init_ad_scan(ALL_ADC);
	//e_init_ad_scan();
	//Reset if Power on (some problem for few robots)
	if (RCONbits.POR) {
		RCONbits.POR=0;
		__asm__ volatile ("reset");
	}
        
	run_asercom();		// advanced sercom protocol

	while(1);
	return 0;
}

