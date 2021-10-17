#include "project_def.h"

#include "gpio_init.h"
#include "xprintf.h"
#include "uart_32f1.h"
#include "can_bus.h"
#include "canhacker_def.h"

#define LED_PORT	GPIOC
#define LED_PIN		13
#define LED_ON		GPIO_SET_RESET(LED_PORT, LED_PIN, 0)	// Low
#define LED_OFF		GPIO_SET_RESET(LED_PORT, LED_PIN, 1)	// Hi
#define LED_TOGGLE	if (LED_PORT->IDR & (1 << LED_PIN)) {LED_ON;} else {LED_OFF;}

volatile uint16_t timestamp = 0;

uint8_t cmd_buf[CMD_BUFFER_LENGTH];	// command buffer
uint8_t cmd_buf_ind = 0;		// command buffer index

uint8_t can_timestamp_enable = 0;

// one byte as 2 ASCII chars
void send_byte2ascii(uint8_t tx_byte);

// execute received command
uint8_t exec_cmd(uint8_t *cmd_buf);

// convert hex ascii to low nibble byte
uint8_t hex2halfbyte(char val);

// convert low nibble byte val to hex ascii
char halfbyte2hex(uint8_t val);

// send one byte as 2 ASCII chars
void byte2ascii(uint8_t tx_byte);

int main(void) {

	RCC->APB2ENR =
			  RCC_APB2ENR_IOPAEN
			| RCC_APB2ENR_IOPBEN
			| RCC_APB2ENR_IOPCEN
			| RCC_APB2ENR_IOPDEN
			| RCC_APB2ENR_AFIOEN

			;

	MAKE_REMAP(AFIO_MAPR_SWJ_CFG_JTAGDISABLE);

	// Configure PC13 as Push Pull output at max 10MHz
	GPIO_INIT_PIN(LED_PORT, LED_PIN, GPIO_MODE_OUTPUT10_PUSH_PULL);
	LED_OFF;

	SysTick_Config(F_SYS_CLK / 1000);

	uart1_init(115200);
	xdev_out(uart1_putc);
	xdev_in(uart1_getc);

	uint8_t can_status = CAN_Init();
	CAN_Filter_init(0, 0, 0, 0, 0);

	while (1) {

		uint8_t c;
		if (uart1_fifo_cnt_rx()) {
			c = uart1_getc();

			if (c == CMD_END_CR) {	// check for end of command
				// execute command and return status to terminal
				xputc(exec_cmd(cmd_buf));

				// flush command buffer
				for (cmd_buf_ind = 0; cmd_buf_ind < CMD_BUFFER_LENGTH;
						cmd_buf_ind++) {
					cmd_buf[cmd_buf_ind] = 0;
				}

				cmd_buf_ind = 0;	// point to start of command
			} else if (c != 0) {	// store new char in buffer

				if (cmd_buf_ind < CMD_BUFFER_LENGTH) {
					cmd_buf[cmd_buf_ind] = c;	// store char
					cmd_buf_ind++;
				}
			}
		}


		// parsing incoming CAN messages

		CAN_RX_SAVE_REG rx_save_reg;

		if (GetRxCanMsg(&rx_save_reg) == CAN_OK) {
			CAN_MSG CAN_rx_msg;

			CAN_rx_msg.format = (rx_save_reg.RIR & CAN_RI0R_IDE) == CAN_RI0R_IDE;
			CAN_rx_msg.rtr = (rx_save_reg.RIR & CAN_RI0R_RTR) == CAN_RI0R_RTR;
			CAN_rx_msg.dlc = rx_save_reg.RDTR & CAN_RDT1R_DLC;

			// check frame format
			if (!CAN_rx_msg.format) { // Standart Frame

				CAN_rx_msg.id = (rx_save_reg.RIR & CAN_RI0R_STID) >> 21;

				if (!CAN_rx_msg.rtr) {
					xputc(SEND_11BIT_ID);
				} else {
					xputc(SEND_R11BIT_ID);
				}

				// send ID bytes
				xputc(halfbyte2hex(CAN_rx_msg.id >> 8));
				send_byte2ascii((uint8_t)CAN_rx_msg.id & 0xFF);
			} else { // Extented Frame

				CAN_rx_msg.id = rx_save_reg.RIR >> 3;

				if (!CAN_rx_msg.rtr) {
					xputc(SEND_29BIT_ID);
				} else {
					xputc(SEND_R29BIT_ID);
				}
				// send ID bytes
				send_byte2ascii((uint8_t) (CAN_rx_msg.id >> 24) & 0xFF);
				send_byte2ascii((uint8_t) (CAN_rx_msg.id >> 16) & 0xFF);
				send_byte2ascii((uint8_t) (CAN_rx_msg.id >> 8) & 0xFF);
				send_byte2ascii((uint8_t) CAN_rx_msg.id & 0xFF);
			}

			if (!CAN_rx_msg.rtr) {
				CAN_rx_msg.data[0] = rx_save_reg.RDLR;
				CAN_rx_msg.data[1] = rx_save_reg.RDLR >> 8;
				CAN_rx_msg.data[2] = rx_save_reg.RDLR >> 16;
				CAN_rx_msg.data[3] = rx_save_reg.RDLR >> 24;

				CAN_rx_msg.data[4] = rx_save_reg.RDHR;
				CAN_rx_msg.data[5] = rx_save_reg.RDHR >> 8;
				CAN_rx_msg.data[6] = rx_save_reg.RDHR >> 16;
				CAN_rx_msg.data[7] = rx_save_reg.RDHR >> 24;
			}

            // send data length code
            xputc(CAN_rx_msg.dlc + '0');
            if (!CAN_rx_msg.rtr) {	// send data only if no remote frame request
                // send data bytes
                for (uint8_t i = 0; i < CAN_rx_msg.dlc; i++) {
                	send_byte2ascii(CAN_rx_msg.data[i]);
                }
            }
            // send time stamp if required
            if (can_timestamp_enable != 0) {
                byte2ascii((uint8_t) (timestamp >> 8));
                byte2ascii((uint8_t) timestamp);
            }
            // send end tag
            xputc(CMD_END_CR);

			LED_TOGGLE;
		}

	}
}

void SysTick_Handler(void) {
	timestamp++;
	if (timestamp > 59999) {
		timestamp = 0;
	}
}

// execute received command
uint8_t exec_cmd(uint8_t *cmd_buf) {

	CAN_MSG CAN_tx_msg;

	uint8_t cmd_len = strlen((char *) cmd_buf);	// get command length

	uint8_t *cmd_buf_pntr = cmd_buf; // point to start of received string
	cmd_buf_pntr++; // skip command identifier

	// check if all chars are valid hex chars
	while (*cmd_buf_pntr) {
		if (!isxdigit(*cmd_buf_pntr)) {
			return ERROR_BEL;
		}
		cmd_buf_pntr++;
	}
	cmd_buf_pntr = cmd_buf;	// reset pointer


	switch (*cmd_buf_pntr++) {
		// get serial number
	case GET_SERIAL:
		xputc(GET_SERIAL);
		xputs(SERIAL);
		return CMD_END_CR;

		// get hard- and software version
	case GET_VERSION:
		xputc(GET_VERSION);
		send_byte2ascii(HW_VER);
		send_byte2ascii(SW_VER);
		return CMD_END_CR;

		// get only software version
	case GET_SW_VERSION:
		xputc(GET_SW_VERSION);
		send_byte2ascii(SW_VER_MAJOR);
		send_byte2ascii(SW_VER_MINOR);
		return CMD_END_CR;

		// toggle time stamp option
	case TIME_STAMP:
		can_timestamp_enable = !can_timestamp_enable;
		DINT;
		timestamp = 0;
		EINT;
		return CMD_END_CR;

		// read status flag
	case READ_STATUS:
		return ERROR_BEL;

		// set AMR
	case SET_AMR:
		// set ACR
	case SET_ACR:
		return ERROR_BEL;

		// set bitrate via BTR
	case SET_BTR:
		CAN_Config(CAN_BUS_BR500K); // set as default
		return ERROR_BEL;

		// set fix bitrate
	case SET_BITRATE:
		if (CAN_Config(*cmd_buf_pntr - '0') != CAN_OK) {
			return ERROR_BEL;
		} else {
			return CMD_END_CR;
		}

		// open CAN channel
	case OPEN_CAN_CHAN:
		if (CAN_GoNormalMode() != CAN_OK) {
			return ERROR_BEL;
		} else {
			return CMD_END_CR;
		}

		// close CAN channel
	case CLOSE_CAN_CHAN:
		if (CAN_GoInitMode() != CAN_OK) {
			return ERROR_BEL;
		} else {
			return CMD_END_CR;
		}

		// send 11bit ID message
	case SEND_R11BIT_ID:

		// check valid cmd length (only 5 bytes for RTR)
		if (cmd_len != 5)
			return ERROR_BEL;

		CAN_tx_msg.rtr = 1;	// remote transmission request

		// store std. frame format
		CAN_tx_msg.format = 0;
		// store ID
		CAN_tx_msg.id = hex2halfbyte(*cmd_buf_pntr++);
		CAN_tx_msg.id <<= 4;
		CAN_tx_msg.id += hex2halfbyte(*cmd_buf_pntr++);
		CAN_tx_msg.id <<= 4;
		CAN_tx_msg.id += hex2halfbyte(*cmd_buf_pntr++);
		// store data length
		CAN_tx_msg.dlc = hex2halfbyte(*cmd_buf_pntr++);

		// if transmit buffer was empty send message
		if (CAN_tx(&CAN_tx_msg) != CAN_OK) {
			return ERROR_BEL;
		} else {
			return CMD_END_CR;
		}

	case SEND_11BIT_ID:
		if ((cmd_len < 5) || (cmd_len > 21))
			return ERROR_BEL;	// check valid cmd length

		CAN_tx_msg.rtr = 0;	// no remote transmission request

		// store std. frame format
		CAN_tx_msg.format = 0;
		// store ID
		CAN_tx_msg.id = hex2halfbyte(*cmd_buf_pntr++);
		CAN_tx_msg.id <<= 4;
		CAN_tx_msg.id += hex2halfbyte(*cmd_buf_pntr++);
		CAN_tx_msg.id <<= 4;
		CAN_tx_msg.id += hex2halfbyte(*cmd_buf_pntr++);
		// store data length
		CAN_tx_msg.dlc = hex2halfbyte(*cmd_buf_pntr++);
		// check number of data bytes supplied against data lenght byte
		if (CAN_tx_msg.dlc != ((cmd_len - 5) / 2))
			return ERROR_BEL;

		// check for valid length
		if (CAN_tx_msg.dlc > 8)
			return ERROR_BEL;
		else {		// store data
			// cmd_len is no longer needed, so we can use it as counter here
			for (cmd_len = 0; cmd_len < CAN_tx_msg.dlc; cmd_len++) {
				CAN_tx_msg.data[cmd_len] = hex2halfbyte(*cmd_buf_pntr++);
				CAN_tx_msg.data[cmd_len] <<= 4;
				CAN_tx_msg.data[cmd_len] += hex2halfbyte(*cmd_buf_pntr++);
			}
		}

		// if transmit buffer was empty send message
		if (CAN_tx(&CAN_tx_msg) != CAN_OK) {
			return ERROR_BEL;
		} else {
			return CMD_END_CR;
		}

		// send 29bit ID message
	case SEND_R29BIT_ID:

		if (cmd_len != 10)
			return ERROR_BEL;	// check valid cmd length

		CAN_tx_msg.rtr = 1;	// remote transmission request

		// store ext. frame format
		CAN_tx_msg.format = 1;
		// store ID
		CAN_tx_msg.id = hex2halfbyte(*cmd_buf_pntr++);
		CAN_tx_msg.id <<= 4;
		CAN_tx_msg.id += hex2halfbyte(*cmd_buf_pntr++);
		CAN_tx_msg.id <<= 4;
		CAN_tx_msg.id += hex2halfbyte(*cmd_buf_pntr++);
		CAN_tx_msg.id <<= 4;
		CAN_tx_msg.id += hex2halfbyte(*cmd_buf_pntr++);
		CAN_tx_msg.id <<= 4;
		CAN_tx_msg.id += hex2halfbyte(*cmd_buf_pntr++);
		CAN_tx_msg.id <<= 4;
		CAN_tx_msg.id += hex2halfbyte(*cmd_buf_pntr++);
		CAN_tx_msg.id <<= 4;
		CAN_tx_msg.id += hex2halfbyte(*cmd_buf_pntr++);
		CAN_tx_msg.id <<= 4;
		CAN_tx_msg.id += hex2halfbyte(*cmd_buf_pntr++);
		// store data length
		CAN_tx_msg.dlc = hex2halfbyte(*cmd_buf_pntr++);

		// if transmit buffer was empty send message
		if (CAN_tx(&CAN_tx_msg) != CAN_OK) {
			return ERROR_BEL;
		} else {
			return CMD_END_CR;
		}

	case SEND_29BIT_ID:

		if ((cmd_len < 10) || (cmd_len > 26))
			return ERROR_BEL;	// check valid cmd length

		CAN_tx_msg.rtr = 0;	// no remote transmission request

		// store ext. frame format
		CAN_tx_msg.format = 1;
		// store ID
		CAN_tx_msg.id = hex2halfbyte(*cmd_buf_pntr++);
		CAN_tx_msg.id <<= 4;
		CAN_tx_msg.id += hex2halfbyte(*cmd_buf_pntr++);
		CAN_tx_msg.id <<= 4;
		CAN_tx_msg.id += hex2halfbyte(*cmd_buf_pntr++);
		CAN_tx_msg.id <<= 4;
		CAN_tx_msg.id += hex2halfbyte(*cmd_buf_pntr++);
		CAN_tx_msg.id <<= 4;
		CAN_tx_msg.id += hex2halfbyte(*cmd_buf_pntr++);
		CAN_tx_msg.id <<= 4;
		CAN_tx_msg.id += hex2halfbyte(*cmd_buf_pntr++);
		CAN_tx_msg.id <<= 4;
		CAN_tx_msg.id += hex2halfbyte(*cmd_buf_pntr++);
		CAN_tx_msg.id <<= 4;
		CAN_tx_msg.id += hex2halfbyte(*cmd_buf_pntr++);
		// store data length
		CAN_tx_msg.dlc = hex2halfbyte(*cmd_buf_pntr++);
		// check number of data bytes supplied against data lenght byte
		if (CAN_tx_msg.dlc != ((cmd_len - 10) / 2))
			return ERROR_BEL;

		// check for valid length
		if (CAN_tx_msg.dlc > 8)
			return ERROR_BEL;
		else {		// store data
			// cmd_len is no longer needed, so we can use it as counter here
			for (cmd_len = 0; cmd_len < CAN_tx_msg.dlc; cmd_len++) {
				CAN_tx_msg.data[cmd_len] = hex2halfbyte(*cmd_buf_pntr++);
				CAN_tx_msg.data[cmd_len] <<= 4;
				CAN_tx_msg.data[cmd_len] += hex2halfbyte(*cmd_buf_pntr++);
			}
		}
		// if transmit buffer was empty send message
		if (CAN_tx(&CAN_tx_msg) != CAN_OK) {
			return ERROR_BEL;
		} else {
			return CMD_END_CR;
		}

		// read Error Capture Register
		// read Arbitration Lost Register
	case READ_ECR:
	case READ_ALCR:
		return ERROR_BEL;

		// read SJA1000 register
	case READ_REG_SJ:
		return ERROR_BEL;

		// write SJA1000 register
	case WRITE_REG_SJ:
		return ERROR_BEL;

	case LISTEN_ONLY:
		return ERROR_BEL;

		// end with error on unknown commands
	default:
		return ERROR_BEL;
	}
}

// convert hex ascii to low nibble byte
uint8_t hex2halfbyte(char val) {
	if (val > 0x60)
		val -= 0x27;		// convert chars a-f
	else if (val > 0x40)
		val -= 0x07;		// convert chars A-F

	val -= 0x30;			// convert chars 0-9

	return val & 0x0F;
}

// one byte as 2 ASCII chars
void send_byte2ascii(uint8_t tx_byte) {
	// send high nibble
	xputc(halfbyte2hex(tx_byte >> 4));
	// send low nibble
	xputc(halfbyte2hex(tx_byte));
}

// convert low nibble byte val to hex ascii
char halfbyte2hex(uint8_t val) {
	val &= 0x0F;
	if (val < 10) {
		return val + '0';
	} else {
		return val - 10 + 'A';
	}
}

// send one byte as 2 ASCII chars
void byte2ascii(uint8_t tx_byte) {
	// send high nibble
	xputc(
			((tx_byte >> 4) < 10) ?
					((tx_byte >> 4) & 0x0f) + 48 :
					((tx_byte >> 4) & 0x0f) + 55);
	// send low nibble
	xputc(
			((tx_byte & 0x0f) < 10) ?
					(tx_byte & 0x0f) + 48 : (tx_byte & 0x0f) + 55);
}

