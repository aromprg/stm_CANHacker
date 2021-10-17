#include "project_def.h"

#include "gpio_init.h"
#include "can_bus.h"
#include "xprintf.h"

#define CAN_REGS	CAN1

#define SET_CAN_BIT_TIME(BRP, TS1, TS2, SJW)	( ((uint32_t)(BRP - 1) << 0U) \
						| ((uint32_t)(TS1 - 1) << 16U) \
						| ((uint32_t)(TS2 - 1) << 20U) \
						| ((uint32_t)(SJW - 1) << 24U) )

//******************************************************************************
// CAN BaudRate calculation (for CAN_CLK = 36 MHz)
// http://www.bittiming.can-wiki.info
// BAUD_RATE = CAN_CLK / (BRP + 1) * (1 + TS1 + 1 + TS2 + 1)
//******************************************************************************
#define CAN_BIT_TIME_1M		SET_CAN_BIT_TIME(2, 15, 2, 1)
#define CAN_BIT_TIME_800K	SET_CAN_BIT_TIME(3, 12, 2, 1)
#define CAN_BIT_TIME_500K	SET_CAN_BIT_TIME(4, 15, 2, 1)
#define CAN_BIT_TIME_250K	SET_CAN_BIT_TIME(9, 13, 2, 1)
#define CAN_BIT_TIME_125K	SET_CAN_BIT_TIME(18, 13, 2, 1)
#define CAN_BIT_TIME_100K	SET_CAN_BIT_TIME(20, 15, 2, 1)
#define CAN_BIT_TIME_50K	SET_CAN_BIT_TIME(45, 13, 2, 1)
#define CAN_BIT_TIME_20K	SET_CAN_BIT_TIME(100, 15, 2, 1)
#define CAN_BIT_TIME_10K	SET_CAN_BIT_TIME(225, 13, 2, 1)

const uint32_t CAN_BTR_SET[] = {
		CAN_BIT_TIME_10K,
		CAN_BIT_TIME_20K,
		CAN_BIT_TIME_50K,
		CAN_BIT_TIME_100K,
		CAN_BIT_TIME_125K,
		CAN_BIT_TIME_250K,
		CAN_BIT_TIME_500K,
		CAN_BIT_TIME_800K,
		CAN_BIT_TIME_1M
};

// CAN RX/TX Ring Buffer

#define TX_CAN_RING_BUFF_SIZE	8
static CAN_MSG tx_can_buf[TX_CAN_RING_BUFF_SIZE];
static volatile uint8_t tx_can_buf_rp = 0;
static uint8_t tx_can_buf_wp = 0;
static volatile uint8_t tx_can_buf_cnt = 0;

#define RX_CAN_RING_BUFF_SIZE	64
static CAN_RX_SAVE_REG rx_can_buf[RX_CAN_RING_BUFF_SIZE];
static volatile uint8_t rx_can_buf_rp = 0;
static uint8_t rx_can_buf_wp = 0;
static volatile uint8_t rx_can_buf_cnt = 0;

#ifdef DEBUG
volatile uint16_t CanRxIsrCnt = 0;
volatile uint16_t CanTxIsrCnt = 0;
volatile uint16_t FailCanRxCnt = 0;
#endif

uint8_t CAN_Init(void) {
	GPIO_INIT_PIN(GPIOA, 11, GPIO_MODE_INPUT_PULL_UP);
	GPIO_INIT_PIN(GPIOA, 12, GPIO_MODE_OUTPUT50_ALT_PUSH_PULL);

	MAKE_REMAP(AFIO_MAPR_CAN_REMAP_REMAP1); // CANRX mapped to PA11, CANTX mapped to PA12

	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;  	// CAN1 clock

	RCC->APB1RSTR |= RCC_APB1RSTR_CAN1RST;	// Reset module
	RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;

	// Exit from sleep mode
	CAN_REGS->MCR &= (~(uint32_t)CAN_MCR_SLEEP);

	if (CAN_GoInitMode() != CAN_OK) {
		return CAN_FAILED;
	}

	return CAN_OK;
}

uint8_t CAN_Config(CAN_BAUD Baud) {

	if (Baud >= CAN_BUS_BR_LAST_REG) {
		return CAN_FAILED;
	}

	CAN_REGS->MCR &= ~CAN_MCR_TTCM;		// Time triggered communication mode OFF
	//CAN_REGS->MCR &= ~CAN_MCR_ABOM;	// Automatic bus-off management OFF
	CAN_REGS->MCR |= CAN_MCR_ABOM;		// Automatic bus-off management ON
	CAN_REGS->MCR &= ~CAN_MCR_AWUM;		// Automatic wakeup mode OFF
	CAN_REGS->MCR &= ~CAN_MCR_NART;		// Automatic retransmission ON
	CAN_REGS->MCR &= ~CAN_MCR_RFLM;		// Receive FIFO not locked on overrun
	CAN_REGS->MCR |= CAN_MCR_TXFP;		// Transmit FIFO priority chronologically

	// baudrate
	CAN_REGS->BTR = CAN_BTR_SET[Baud];

	return CAN_OK;
}

uint8_t CAN_GoInitMode(void) {

	CAN_REGS->MCR |= CAN_MCR_INRQ; // go to init mode

	// wait
	uint32_t tmo = INAK_TIMEOUT;
	while (tmo && ((CAN_REGS->MSR & CAN_MSR_INAK) != CAN_MSR_INAK)) {
		tmo--;
	}

	// check status
	if ((CAN_REGS->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) {
		return CAN_FAILED;
	}

	DINT;
	NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);	// disable can rx FIFO_0 IRQ
	NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);	// disable can tx IRQ
	EINT;

	FlushCanTxBuffer();
	FlushCanRxBuffer();

	return CAN_OK;
}

uint8_t CAN_GoNormalMode(void) {

	CAN_REGS->MCR &= ~CAN_MCR_INRQ; // go to normal mode

	// wait
	uint32_t tmo = INAK_TIMEOUT;
	while (tmo && ((CAN_REGS->MSR & CAN_MSR_INAK) == CAN_MSR_INAK)) {
		tmo--;
	}

	// check status
	if ((CAN_REGS->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) {
		return CAN_FAILED;
	}

	CAN_REGS->RF0R |= CAN_RF0R_RFOM0; 	// release fifo

	DINT;
	NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);	// NVIC CAN rx FIFO_0 IRQ
	NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);	// NVIC CAN tx IRQ
	CAN_REGS->IER |= CAN_IER_FMPIE0;	// enable CAN rx fifo pending isr
	EINT;

	return CAN_OK;
}

uint8_t CAN_tx(const CAN_MSG *msg) {
	return PutTxCanMsg(msg);
}

void CAN_Filter_init(uint8_t filter_number, uint16_t FilterIdHigh, uint16_t FilterIdLow, uint16_t FilterMaskIdHigh, uint16_t FilterMaskIdLow) {
#if !defined (STM32F10X_CL)
	// filter number 0..13
	if (filter_number > 13) {
		filter_number = 13;
	}
#else
	// filter number 0..27
	if (filter_number > 27) {
		filter_number = 27;
	}
#endif

	uint32_t filter_number_bit_pos = ((uint32_t)1) << filter_number;

	// Initialisation mode for the filter
	CAN_REGS->FMR |= CAN_FMR_FINIT;

	// Filter Deactivation
	CAN_REGS->FA1R &= ~filter_number_bit_pos;

	// 32-bit scale for the filter
	CAN_REGS->FS1R |= filter_number_bit_pos;
	// 32-bit identifier or First 32-bit identifier
	CAN_REGS->sFilterRegister[filter_number].FR1 =
			((0x0000FFFF & (uint32_t) FilterIdHigh) << 16) |
			(0x0000FFFF	& (uint32_t) FilterIdLow);

	// 32-bit mask or Second 32-bit identifier
	CAN_REGS->sFilterRegister[filter_number].FR2 =
			((0x0000FFFF & (uint32_t) FilterMaskIdHigh) << 16)|
			(0x0000FFFF	& (uint32_t) FilterMaskIdLow);

	// Id/Mask mode for the filter
	CAN_REGS->FM1R &= ~(uint32_t) filter_number_bit_pos;

	// FIFO 0 assignation for the filter
	// bit=0 - FIFO_0; bit=1 - FIFO_1
	CAN_REGS->FFA1R &= ~(uint32_t) filter_number_bit_pos;

	// Filter activation
	CAN_REGS->FA1R |= filter_number_bit_pos;

	// Leave the initialisation mode for the filter
	CAN_REGS->FMR &= ~CAN_FMR_FINIT;
}

//---------------  TX RING BUFFER API  ---------------//
void FlushCanTxBuffer(void) {
	DINT;
	tx_can_buf_cnt = 0;
	tx_can_buf_rp = 0;
	tx_can_buf_wp = 0;
	EINT;
}

uint8_t PutTxCanMsg(const CAN_MSG *msg) {

	// transmit ongoing
	if (CAN_REGS->IER & CAN_IER_TMEIE) {
		if (tx_can_buf_cnt < TX_CAN_RING_BUFF_SIZE) {

			tx_can_buf[tx_can_buf_wp] = *msg;

			DINT;
			tx_can_buf_cnt++;
			EINT;

			CAN_REGS->IER |= CAN_IER_TMEIE;	// enable interrupt if now tx ended

#if IS_POWER_OF_2(TX_CAN_RING_BUFF_SIZE)
			tx_can_buf_wp = (tx_can_buf_wp + 1) & (TX_CAN_RING_BUFF_SIZE - 1);
#else
			tx_can_buf_wp++;
			if (tx_can_buf_wp == TX_CAN_RING_BUFF_SIZE) {
				tx_can_buf_wp = 0;
			}
#endif
			return CAN_OK;
		}
	} else {
		if (msg->format) {
			CAN_REGS->sTxMailBox[0].TIR = (msg->id << 3) | CAN_TI0R_IDE;
		} else {
			CAN_REGS->sTxMailBox[0].TIR = (msg->id << 21);
		}

		// DLC
		uint8_t len = msg->dlc;
		if (len > 8) {
			len = 8;
		}
		CAN_REGS->sTxMailBox[0].TDTR = len;

		if (msg->rtr) {
			CAN_REGS->sTxMailBox[0].TIR |= CAN_TI0R_RTR;
		} else {
			// message
			CAN_REGS->sTxMailBox[0].TDLR = 0;
			CAN_REGS->sTxMailBox[0].TDHR = 0;
			for (uint8_t i = 0; i < len; i++) {

				if (i < 4) {
					CAN_REGS->sTxMailBox[0].TDLR |=
							(uint32_t)msg->data[i] << (i * 8);
				} else {
					CAN_REGS->sTxMailBox[0].TDHR |=
							(uint32_t)msg->data[i] << ((i - 4) * 8);
				}
			}
		}

		CAN_REGS->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ; // send

		CAN_REGS->IER |= CAN_IER_TMEIE; // enable interrupt

		return CAN_OK;
	}

	return CAN_FAILED;
}

uint8_t GetTxCanMsgCnt(void) {
	return tx_can_buf_cnt;
}

//---------------  RX RING BUFFER API  ---------------//
void FlushCanRxBuffer(void) {
	DINT;
	rx_can_buf_cnt = 0;
	rx_can_buf_rp = 0;
	rx_can_buf_wp = 0;
	EINT;
}

uint8_t GetRxCanMsg(CAN_RX_SAVE_REG *msg) {
	if (rx_can_buf_cnt) {
		memcpy(msg, &rx_can_buf[rx_can_buf_rp], sizeof(CAN_RX_SAVE_REG));
		DINT;
		rx_can_buf_cnt--;
		EINT;
#if IS_POWER_OF_2(RX_CAN_RING_BUFF_SIZE)
		rx_can_buf_rp = (rx_can_buf_rp + 1) & (RX_CAN_RING_BUFF_SIZE - 1);
#else
		rx_can_buf_rp++;
		if (rx_can_buf_rp == RX_CAN_RING_BUFF_SIZE) {
			rx_can_buf_rp = 0;
		}
#endif
		return CAN_OK;
	} else {
		memset(msg, 0, sizeof(CAN_RX_SAVE_REG));
		return CAN_FAILED;
	}
}

uint8_t GetRxCanMsgCnt(void) {
	return rx_can_buf_cnt;
}

void USB_HP_CAN1_TX_IRQHandler(void) {

	// transmit mailbox empty
	if (CAN_REGS->TSR & CAN_TSR_RQCP0) {
		if (tx_can_buf_cnt) {

			if (tx_can_buf[tx_can_buf_rp].format) {
				CAN_REGS->sTxMailBox[0].TIR = (tx_can_buf[tx_can_buf_rp].id << 3) | CAN_TI0R_IDE;
			} else {
				CAN_REGS->sTxMailBox[0].TIR = (tx_can_buf[tx_can_buf_rp].id << 21);
			}

			uint8_t len = tx_can_buf[tx_can_buf_rp].dlc;
			if (len > 8) {
				len = 8;
			}
			CAN_REGS->sTxMailBox[0].TDTR = len;

			if (tx_can_buf[tx_can_buf_rp].rtr) {
				CAN_REGS->sTxMailBox[0].TIR |= CAN_TI0R_RTR;
			} else {
				// message
				CAN_REGS->sTxMailBox[0].TDLR = 0;
				CAN_REGS->sTxMailBox[0].TDHR = 0;
				for (uint8_t i = 0; i < len; i++) {

					if (i < 4) {
						CAN_REGS->sTxMailBox[0].TDLR |=
								tx_can_buf[tx_can_buf_rp].data[i] << (i * 8);
					} else {
						CAN_REGS->sTxMailBox[0].TDHR |=
								tx_can_buf[tx_can_buf_rp].data[i] << ((i - 4) * 8);
					}
				}
			}

			// send
			CAN_REGS->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;

#if IS_POWER_OF_2(TX_CAN_RING_BUFF_SIZE)
			tx_can_buf_rp = (tx_can_buf_rp + 1) & (TX_CAN_RING_BUFF_SIZE - 1);
#else
			tx_can_buf_rp++;
			if (tx_can_buf_rp == TX_CAN_RING_BUFF_SIZE) {
				tx_can_buf_rp = 0;
			}
#endif

			tx_can_buf_cnt--;
		} else {
			// disable int
			CAN_REGS->IER &= ~CAN_IER_TMEIE;
		}
	}

#ifdef DEBUG
	CanTxIsrCnt++;
#endif
}

void USB_LP_CAN1_RX0_IRQHandler(void) {
	if (CAN_REGS->RF0R & CAN_RF0R_FMP0) {
		if (rx_can_buf_cnt < RX_CAN_RING_BUFF_SIZE) {
			rx_can_buf[rx_can_buf_wp].RIR = CAN_REGS->sFIFOMailBox[0].RIR;
			rx_can_buf[rx_can_buf_wp].RDTR = CAN_REGS->sFIFOMailBox[0].RDTR;
			rx_can_buf[rx_can_buf_wp].RDLR = CAN_REGS->sFIFOMailBox[0].RDLR;
			rx_can_buf[rx_can_buf_wp].RDHR = CAN_REGS->sFIFOMailBox[0].RDHR;

			CAN_REGS->RF0R |= CAN_RF0R_RFOM0;	// free fifo after read

			rx_can_buf_cnt++;

#if IS_POWER_OF_2(RX_CAN_RING_BUFF_SIZE)
			rx_can_buf_wp = (rx_can_buf_wp + 1) & (RX_CAN_RING_BUFF_SIZE - 1);
#else
			rx_can_buf_wp++;
			if (rx_can_buf_wp == RX_CAN_RING_BUFF_SIZE) {
				rx_can_buf_wp = 0;
			}
#endif
		} else {
			CAN_REGS->RF0R |= CAN_RF0R_RFOM0; // drop message
#ifdef DEBUG
			FailCanRxCnt++;
#endif
		}
	}

#ifdef DEBUG
	CanRxIsrCnt++;
#endif
}

