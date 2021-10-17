#ifndef _UART_DEFS
#define _UART_DEFS


#define USE_UART1	2		/* 0:Disable, 1:Enable, 2:Enable w/remap */
#define UART1_RXB	64		/* Size of Rx buffer */
#define UART1_TXB	256		/* Size of Tx buffer */

#define USE_UART2	0		/* 0:Disable, 1:Enable, 2:Enable w/remap */
#define UART2_RXB	128		/* Size of Rx buffer */
#define UART2_TXB	128		/* Size of Tx buffer */

#define USE_UART3	0		/* 0:Disable, 1:Enable, 2:Enable w/remap */
#define UART3_RXB	128		/* Size of Rx buffer */
#define UART3_TXB	128		/* Size of Tx buffer */



#if USE_UART1
void uart1_init (uint32_t bps); /* Initialize USART */
uint16_t uart1_fifo_cnt_rx(void); /* Check number of data in UART Rx FIFO */
uint16_t uart1_fifo_cnt_tx(void); /* Check number of data in UART Tx FIFO */
uint8_t uart1_getc (void); /* Get a received character */
void uart1_putc (uint8_t d); /* Put a character to transmit */
#endif
#if USE_UART2
void uart2_init (uint32_t bps);
uint16_t uart2_fifo_cnt_rx(void);
uint16_t uart2_fifo_cnt_tx(void);
void uart2_putc (uint8_t);
uint8_t uart2_getc (void);
#endif
#if USE_UART3
void uart3_init (uint32_t bps);
uint16_t uart3_fifo_cnt_rx(void);
uint16_t uart3_fifo_cnt_tx(void);
void uart3_putc (uint8_t);
uint8_t uart3_getc (void);
#endif

#endif
