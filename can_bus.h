#ifndef CAN_BUS_H_
#define CAN_BUS_H_

typedef enum {
	CAN_OK,
	CAN_FAILED
} CAN_ERROR_CODE;

// baudrate settings
typedef enum {
	CAN_BUS_BR10K,
	CAN_BUS_BR20K,
	CAN_BUS_BR50K,
	CAN_BUS_BR100K,
	CAN_BUS_BR125K,
	CAN_BUS_BR250K,
	CAN_BUS_BR500K,
	CAN_BUS_BR800K,
	CAN_BUS_BR1M,

	CAN_BUS_BR_LAST_REG
} CAN_BAUD;

// Timeout for INAK bit
#define INAK_TIMEOUT ((uint32_t)0x0000FFFF)

// CAN message
typedef struct {
	uint8_t format;		// Frame: 0 - Standard ID; !=0 - Extended ID
	uint32_t id;		// Frame ID
	uint8_t rtr;		// 0 - Data; !=0 - RTR Frame
	uint8_t dlc;		// Data Length
	uint8_t data[8];	// Data Bytes
} CAN_MSG;

// save CAN data&spec register on received message
typedef struct {
	uint32_t RIR;
	uint32_t RDTR;
	uint32_t RDLR;
	uint32_t RDHR;
} CAN_RX_SAVE_REG;

uint8_t CAN_Init(void);
uint8_t CAN_Config(CAN_BAUD Baud);
uint8_t CAN_GoInitMode(void);
uint8_t CAN_GoNormalMode(void);

uint8_t CAN_tx(const CAN_MSG *msg);

void CAN_Filter_init(uint8_t filter_number, uint16_t FilterIdHigh, uint16_t FilterIdLow, uint16_t FilterMaskIdHigh, uint16_t FilterMaskIdLow);


//---------------  TX RING BUFFER API  ---------------//
void FlushCanTxBuffer(void);
uint8_t PutTxCanMsg(const CAN_MSG *msg);
uint8_t GetTxCanMsgCnt(void);

//---------------  RX RING BUFFER API  ---------------//
void FlushCanRxBuffer(void);
uint8_t GetRxCanMsg(CAN_RX_SAVE_REG *msg);
uint8_t GetRxCanMsgCnt(void);

#endif /* CAN_BUS_H_ */
