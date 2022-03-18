/*------------------------------------------------------------------------/
 /  STM32F100 USART control module
 /-------------------------------------------------------------------------/
 /
 /  Copyright (C) 2013, ChaN, all right reserved.
 /
 / * This software is a free software and there is NO WARRANTY.
 / * No restriction on use. You can use, modify and redistribute it for
 /   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
 / * Redistributions of source code must retain the above copyright notice.
 /
 /-------------------------------------------------------------------------*/
#include "project_def.h"

#include "stm32f10x.h"
#include "uart_32f1.h"
#include "gpio_init.h"

#define F_PCLK1	36000000	/* APB1 clock for USART2/3  */
#define F_PCLK2	72000000	/* APB2 clock for USART1 */

#if USE_UART1

static volatile struct {
	uint16_t tri, twi, tct;
	uint16_t rri, rwi, rct;
	uint8_t tbuf[UART1_TXB];
	uint8_t rbuf[UART1_RXB];
} Fifo1;

void USART1_IRQHandler(void) {
	uint32_t sr = USART1->SR; /* Interrupt flags */
	uint8_t d;
	int i;

	if (sr & USART_SR_RXNE) { /* RXNE is set: Rx ready */
		d = USART1->DR; /* Get received byte */
		i = Fifo1.rct;
		if (i < UART1_RXB) { /* Store it into the rx fifo if not full */
			Fifo1.rct = ++i;
			i = Fifo1.rwi;
			Fifo1.rbuf[i] = d;
			Fifo1.rwi = ++i % UART1_RXB;
		}
	}

	if (sr & USART_SR_TXE) { /* TXE is set: Tx ready */
		i = Fifo1.tct;
		if (i--) { /* There is any data in the tx fifo */
			Fifo1.tct = (uint16_t) i;
			i = Fifo1.tri;
			USART1->DR = Fifo1.tbuf[i];
			Fifo1.tri = ++i % UART1_TXB;
		} else { /* No data in the tx fifo */
			USART1->CR1 &= ~USART_CR1_TXEIE; /* Clear TXEIE - Disable TXE irq */
		}
	}
}

/* Check number of data in UART Rx FIFO */
uint16_t uart1_fifo_cnt_rx(void) {
	return Fifo1.rct;
}

/* Check number of data in UART Tx FIFO */
uint16_t uart1_fifo_cnt_tx(void) {
	return Fifo1.tct;
}

/* Get a received character */
uint8_t uart1_getc(void) {
	uint8_t d;
	int i;

	/* Wait while rx fifo is empty */
	while (!Fifo1.rct)
		;

	i = Fifo1.rri; /* Get a byte from rx fifo */
	d = Fifo1.rbuf[i];
	Fifo1.rri = ++i % UART1_RXB;
	__disable_irq();
	Fifo1.rct--;
	__enable_irq();

	return d;
}

/* Put a character to transmit */
void uart1_putc(uint8_t d) {
	int i;

	/* Wait for tx fifo is not full */
	while (Fifo1.tct >= UART1_TXB)
		;

	i = Fifo1.twi; /* Put a byte into Tx fifo */
	Fifo1.tbuf[i] = d;
	Fifo1.twi = ++i % UART1_TXB;
	__disable_irq();
	Fifo1.tct++;
	USART1->CR1 |= USART_CR1_TXEIE; /* Set TXEIE - Enable TXE irq */
	__enable_irq();
}

/* Initialize USART */
void uart1_init(uint32_t bps) {
	NVIC_DisableIRQ(USART1_IRQn); /* Disable USART1 interrupts */

	/* Attach USART1 module to I/O pads */
#if USE_UART1 == 1
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // PORTA clock
	GPIO_INIT_PIN(GPIOA, 9, GPIO_MODE_OUTPUT10_ALT_PUSH_PULL);
	GPIO_INIT_PIN(GPIOA, 10, GPIO_MODE_INPUT_PULL_UP);
#else
	MAKE_REMAP(AFIO_MAPR_USART1_REMAP); // Remap
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; // PORTB clock
	GPIO_INIT_PIN(GPIOB, 6, GPIO_MODE_OUTPUT10_ALT_PUSH_PULL);
	GPIO_INIT_PIN(GPIOB, 7, GPIO_MODE_INPUT_PULL_UP);
#endif
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; /* Enable USART module */

	RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;	/* Reset USART module */
	RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;

	USART1->BRR = F_PCLK2 / bps; /* Set bit rate */

	/* Enable USART in N81, Enable RXNE irq */
	USART1->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_UE;

	/* Clear Tx/Rx fifo */
	Fifo1.tri = 0;
	Fifo1.twi = 0;
	Fifo1.tct = 0;
	Fifo1.rri = 0;
	Fifo1.rwi = 0;
	Fifo1.rct = 0;

	NVIC_EnableIRQ(USART1_IRQn); /* Enable USART1 interrupts */
}

#endif	/* USE_UART1 */

#if USE_UART2

static volatile struct {
	uint16_t tri, twi, tct;
	uint16_t rri, rwi, rct;
	uint8_t tbuf[UART2_TXB];
	uint8_t rbuf[UART2_RXB];
}Fifo2;

void USART2_IRQHandler (void)
{
	uint32_t sr = USART2->SR;
	uint8_t d;
	int i;

	if (sr & USART_SR_RXNE) {
		d = USART2->DR;
		i = Fifo2.rct;
		if (i < UART2_RXB) {
			Fifo2.rct = ++i;
			i = Fifo2.rwi;
			Fifo2.rbuf[i] = d;
			Fifo2.rwi = ++i % UART2_RXB;
		}
	}
	if (sr & USART_SR_TXE) {
		i = Fifo2.tct;
		if (i--) {
			Fifo2.tct = (uint16_t)i;
			i = Fifo2.tri;
			USART2->DR = Fifo2.tbuf[i];
			Fifo2.tri = ++i % UART2_TXB;
		} else {
			USART2->CR1 &= ~USART_CR1_TXEIE;
		}
	}
}

/* Check number of data in UART Rx FIFO */
uint16_t uart2_fifo_cnt_rx(void) {
	return Fifo2.rct;
}

/* Check number of data in UART Tx FIFO */
uint16_t uart2_fifo_cnt_tx(void) {
	return Fifo2.tct;
}

uint8_t uart2_getc (void)
{
	uint8_t d;
	int i;

	/* Wait while Rx fifo is empty */
	while (!Fifo2.rct);

	i = Fifo2.rri; /* Get a byte from Rx fifo */
	d = Fifo2.rbuf[i];
	Fifo2.rri = ++i % UART2_RXB;
	__disable_irq();
	Fifo2.rct--;
	__enable_irq();

	return d;
}

void uart2_putc (uint8_t d)
{
	int i;

	/* Wait for Tx fifo ready */
	while (Fifo2.tct >= UART2_TXB);

	i = Fifo2.twi; /* Put a byte into Tx fifo */
	Fifo2.tbuf[i] = d;
	Fifo2.twi = ++i % UART2_TXB;
	__disable_irq();
	Fifo2.tct++;
	USART2->CR1 |= USART_CR1_TXEIE;
	__enable_irq();
}

void uart2_init (uint32_t bps)
{
	NVIC_DisableIRQ(USART2_IRQn);

	/* Attach UART module to I/O pads */
#if USE_UART2 == 1
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // PORTA clock
	GPIO_INIT_PIN(GPIOA, 2, GPIO_MODE_OUTPUT10_ALT_PUSH_PULL);
	GPIO_INIT_PIN(GPIOA, 3, GPIO_MODE_INPUT_PULL_UP);
#else
	MAKE_REMAP(AFIO_MAPR_USART2_REMAP); // Remap
	RCC->APB2ENR |= RCC_APB2ENR_IOPDEN; // PORTD clock
	GPIO_INIT_PIN(GPIOD, 5, GPIO_MODE_OUTPUT10_ALT_PUSH_PULL);
	GPIO_INIT_PIN(GPIOD, 6, GPIO_MODE_INPUT_PULL_UP);

#endif
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; /* Enable USART module */

	RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;	/* Reset USART module */
	RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;

	USART2->BRR = F_PCLK1 / bps;

	/* Enable USART in N81, Enable RXNE irq */
	USART2->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_UE;

	/* Clear Tx/Rx fifo */
	Fifo2.tri = 0; Fifo2.twi = 0; Fifo2.tct = 0;
	Fifo2.rri = 0; Fifo2.rwi = 0; Fifo2.rct = 0;

	NVIC_EnableIRQ(USART2_IRQn);
}

#endif	/* USE_UART2 */

#if USE_UART3

static volatile struct {
	uint16_t tri, twi, tct;
	uint16_t rri, rwi, rct;
	uint8_t tbuf[UART3_TXB];
	uint8_t rbuf[UART3_RXB];
}Fifo3;

void USART3_IRQHandler (void)
{
	uint32_t sr = USART3->SR;
	uint8_t d;
	int i;

	if (sr & USART_SR_RXNE) {
		d = USART3->DR;
		i = Fifo3.rct;
		if (i < UART3_RXB) {
			Fifo3.rct = ++i;
			i = Fifo3.rwi;
			Fifo3.rbuf[i] = d;
			Fifo3.rwi = ++i % UART3_RXB;
		}
	}
	if (sr & USART_SR_TXE) {
		i = Fifo3.tct;
		if (i--) {
			Fifo3.tct = (uint16_t)i;
			i = Fifo3.tri;
			USART3->DR = Fifo3.tbuf[i];
			Fifo3.tri = ++i % UART3_TXB;
		} else {
			USART3->CR1 &= ~USART_CR1_TXEIE;
		}
	}
}

/* Check number of data in UART Rx FIFO */
uint16_t uart3_fifo_cnt_rx(void) {
	return Fifo3.rct;
}

/* Check number of data in UART Tx FIFO */
uint16_t uart3_fifo_cnt_tx(void) {
	return Fifo3.tct;
}

uint8_t uart3_getc (void)
{
	uint8_t d;
	int i;

	/* Wait while Rx fifo is empty */
	while (!Fifo3.rct);

	i = Fifo3.rri; /* Get a byte from Rx fifo */
	d = Fifo3.rbuf[i];
	Fifo3.rri = ++i % UART3_RXB;
	__disable_irq();
	Fifo3.rct--;
	__enable_irq();

	return d;
}

void uart3_putc (uint8_t d)
{
	int i;

	/* Wait for Tx firo ready */
	while (Fifo3.tct >= UART3_TXB);

	i = Fifo3.twi; /* Put a byte into Tx byffer */
	Fifo3.tbuf[i] = d;
	Fifo3.twi = ++i % UART3_TXB;
	__disable_irq();
	Fifo3.tct++;
	USART3->CR1 |= USART_CR1_TXEIE;
	__enable_irq();
}

void uart3_init (uint32_t bps)
{
	NVIC_DisableIRQ(USART3_IRQn);

	/* Attach UART module to I/O pads */
#if USE_UART3 == 1
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;  // PORTB clock
	GPIO_INIT_PIN(GPIOB, 10, GPIO_MODE_OUTPUT10_ALT_PUSH_PULL);
	GPIO_INIT_PIN(GPIOB, 11, GPIO_MODE_INPUT_PULL_UP);
#else
	MAKE_REMAP(AFIO_MAPR_USART3_REMAP); // Remap
	RCC->APB2ENR |= RCC_APB2ENR_IOPDEN; // PORTD clock
	GPIO_INIT_PIN(GPIOD, 8, GPIO_MODE_OUTPUT10_ALT_PUSH_PULL);
	GPIO_INIT_PIN(GPIOD, 9, GPIO_MODE_INPUT_PULL_UP);
#endif
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; /* Enable USART module */

	RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST;	/* Reset USART module */
	RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;

	USART3->BRR = F_PCLK1 / bps;

	/* Enable USART in N81, Enable RXNE irq */
	USART3->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_UE;

	/* Clear Tx/Rx fifo */
	Fifo3.tri = 0; Fifo3.twi = 0; Fifo3.tct = 0;
	Fifo3.rri = 0; Fifo3.rwi = 0; Fifo3.rct = 0;

	NVIC_EnableIRQ(USART3_IRQn);
}

#endif	/* USE_UART3 */

