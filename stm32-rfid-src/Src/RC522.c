#include "RC522.h"
#include "stdbool.h"
#include "stm32u5xx.h"
/*
 * STM32 ->RFID
 * SPI  -> SPI
 * PA8  ->RST
 * PB0  ->CS
 * */

SPI_HandleTypeDef hspi1;


/**
 * @brief Writes to the CS pin of the RFID; Basic SPI functionality - GPIOB
 * @param state The state to set the CS pin
 *  */
void spi_cs_rfid_write(bool state)
{
	if(state)
	  {
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	  }
	  else
	  {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	  }
}


/**
 * @brief Reads an 8-bit register from the RC522
 * @param reg The register to read from
 * @return The value read from the register
 */
uint8_t rc522_regRead8(uint8_t reg)
{

  uint8_t dataRd=0;

  spi_cs_rfid_write(0);
  reg = ((reg << 1) & 0x7E) | 0x80;
  HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, &dataRd, 1, HAL_MAX_DELAY);
  spi_cs_rfid_write(1);

  return dataRd;
}


/**
 * @brief Writes an 8-bit value to a register in the RC522
 * @param reg The register to write to
 * @param data8 The data to write
 */
void rc522_regWrite8(uint8_t reg, uint8_t data8)
{
  spi_cs_rfid_write(0);

  uint8_t txData[2] = {0x7E&(reg << 1), data8};
  HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
  spi_cs_rfid_write(1);
}

/**
 * @brief Sets specific bits in a register
 * @param reg The register to modify
 * @param mask The mask of bits to set
 */
void rc522_setBit(uint8_t reg, uint8_t mask)
{
  rc522_regWrite8(reg, rc522_regRead8(reg)|mask);
}

/**
 * @brief Clears specific bits in a register
 * @param reg The register to modify
 * @param mask The mask of bits to clear
 */
void rc522_clearBit(uint8_t reg, uint8_t mask)
{
  rc522_regWrite8(reg, rc522_regRead8(reg)&(~mask));
}

/**
 * @brief Resets the RC522
 */
void rc522_reset(void)
{
  rc522_regWrite8(0x01, 0x0F);
}

/**
 * @brief Turns on the antenna of the RC522
 */
void rc522_antennaON(void)
{
  uint8_t temp;

  temp = rc522_regRead8(MFRC522_REG_TX_CONTROL);
  if (!(temp & 0x03)) { //Check if antenna is not already on, otherwise turn on
    rc522_setBit(MFRC522_REG_TX_CONTROL, 0x03);
  }
}

/**
 * @brief Checks if a card is present and returns its ID
 * @param id The ID of the card to be returned
 * @return True if a card is detected, False otherwise
 */
bool rc522_checkCard(uint8_t *id)
{
  bool status=false;

    status = rc522_request(PICC_REQIDL, id); //Find cards, return card type
    if (status == true) { //Detected a card
      status = rc522_antiColl(id); //Anti-collision, return card serial number 4 bytes
    }
    rc522_halt();//Command card into hibernation

    return status;
}


/**
 * @brief Sends a request to the RC522 and waits for a response
 * @param reqMode The request mode
 * @param tagType The tag type to be returned
 * @return True if successful, False otherwise
 */
bool rc522_request(uint8_t reqMode, uint8_t *tagType)
{
  bool status=false;
  uint16_t backBits;

  rc522_regWrite8(MFRC522_REG_BIT_FRAMING, 0x07);
  tagType[0] = reqMode;
  status = rc522_toCard(PCD_TRANSCEIVE, tagType, 1, tagType, &backBits);
  if ((status != true) || (backBits != 0x10)) {
    status = false;
  }
  return status;
}

/**
 * @brief Communicates with the RC522 card
 * @param command The command to send
 * @param sendData The data to send
 * @param sendLen Length of the data to send
 * @param backData The data received from the card
 * @param backLen Length of the data received
 * @return True if successful, False otherwise
 */
bool rc522_toCard(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen)
{
  bool status = false;
  uint8_t irqEn = 0x00;
  uint8_t waitIRq = 0x00;
  uint8_t lastBits;
  uint8_t n;
  uint16_t i;

  switch (command) {
    case PCD_AUTHENT: {
      irqEn = 0x12;
      waitIRq = 0x10;
      break;
    }
    case PCD_TRANSCEIVE: {
      irqEn = 0x77;
      waitIRq = 0x30;
      break;
    }
    default:
      break;
  }

  rc522_regWrite8(MFRC522_REG_COMM_IE_N, irqEn | 0x80);
  rc522_clearBit(MFRC522_REG_COMM_IRQ, 0x80);
  rc522_setBit(MFRC522_REG_FIFO_LEVEL, 0x80);

  rc522_regWrite8(MFRC522_REG_COMMAND, PCD_IDLE);

  //Writing data to the FIFO
  for (i = 0; i < sendLen; i++) {
    rc522_regWrite8(MFRC522_REG_FIFO_DATA, sendData[i]);
  }

  //Execute the command
  rc522_regWrite8(MFRC522_REG_COMMAND, command);
  if (command == PCD_TRANSCEIVE) {
    rc522_setBit(MFRC522_REG_BIT_FRAMING, 0x80);   //StartSend=1,transmission of data starts
  }

  //Waiting to receive data to complete
  i = 100;  //i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms???
  do {
    //CommIrqReg[7..0]
    //Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
    n = rc522_regRead8(MFRC522_REG_COMM_IRQ);
    i--;
  } while ((i!=0) && !(n&0x01) && !(n&waitIRq));

  rc522_clearBit(MFRC522_REG_BIT_FRAMING, 0x80);     //StartSend=0

  if (i != 0)  {
    if (!(rc522_regRead8(MFRC522_REG_ERROR) & 0x1B)) {
      status = true;
      if (n & irqEn & 0x01) {
        status = false;
      }

      if (command == PCD_TRANSCEIVE) {
        n = rc522_regRead8(MFRC522_REG_FIFO_LEVEL);
        uint8_t l = n;
        lastBits = rc522_regRead8(MFRC522_REG_CONTROL) & 0x07;
        if (lastBits) {
          *backLen = (n - 1) * 8 + lastBits;
        } else {
          *backLen = n * 8;
        }

        if (n == 0) {
          n = 1;
        }
        if (n > MFRC522_MAX_LEN) {
          n = MFRC522_MAX_LEN;
        }

        //Reading the received data in FIFO
        for (i = 0; i < n; i++) {
          uint8_t d = rc522_regRead8(MFRC522_REG_FIFO_DATA);
          if (l == 4)
            printf("%02x ", d);
          backData[i] = d;
        }
        if (l==4)
          printf("\r\n");
        return status;
      }
    } else {
      printf("error\r\n");
      status = false;
    }
  }

  return status;
}

/**
 * @brief Anti-collision detection to get the serial number of the card
 * @param serNum The serial number to be returned
 * @return True if successful, False otherwise
 */
bool rc522_antiColl(uint8_t* serNum)
{
  bool status;
  uint8_t i;
  uint8_t serNumCheck = 0;
  uint16_t unLen;
  rc522_regWrite8(MFRC522_REG_BIT_FRAMING, 0x00);    //TxLastBists = BitFramingReg[2..0]

  serNum[0] = PICC_ANTICOLL;
  serNum[1] = 0x20;
  status = rc522_toCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

  if (status == true) {
    //Check card serial number
    for (i = 0; i < 4; i++) {
      serNumCheck ^= serNum[i];
    }
    if (serNumCheck != serNum[i]) {
      status = false;
    }
  }
  return status;
}

void rc522_halt(void)
{
  uint16_t unLen;
  uint8_t buff[4];

  buff[0] = PICC_HALT;
  buff[1] = 0;
  rc522_calculateCRC(buff, 2, &buff[2]);

  rc522_toCard(PCD_TRANSCEIVE, buff, 4, buff, &unLen);
}

/**
 * @brief Calculates the CRC for the given data
 * @param pIndata The input data
 * @param len The length of the input data
 * @param pOutData The calculated CRC
 */
void rc522_calculateCRC(uint8_t*  pIndata, uint8_t len, uint8_t* pOutData)
{
  uint8_t i, n;

  rc522_clearBit(MFRC522_REG_DIV_IRQ, 0x04);     //CRCIrq = 0
  rc522_setBit(MFRC522_REG_FIFO_LEVEL, 0x80);      //Clear the FIFO pointer

  //Writing data to the FIFO
  for (i = 0; i < len; i++) {
    rc522_regWrite8(MFRC522_REG_FIFO_DATA, *(pIndata+i));
  }
  rc522_regWrite8(MFRC522_REG_COMMAND, PCD_CALCCRC);

  //Wait CRC calculation is complete
  i = 0xFF;
  do {
    n = rc522_regRead8(MFRC522_REG_DIV_IRQ);
    i--;
  } while ((i!=0) && !(n&0x04));      //CRCIrq = 1

  //Read CRC calculation result
  pOutData[0] = rc522_regRead8(MFRC522_REG_CRC_RESULT_L);
  pOutData[1] = rc522_regRead8(MFRC522_REG_CRC_RESULT_M);
}

/**
 * @brief Compares two card IDs
 * @param idCurrent The current ID
 * @param idReference The reference ID
 * @return True if IDs match, False otherwise
 */
bool rc522_compareIds(uint8_t *idCurrent, uint8_t *idReference)
{
  uint8_t i;
  for(i=0; i<4;i++)
  {
    if(idCurrent[i] != idReference[i])
    {
      return false;
    }
  }
  return true;
}


/**
 * @brief Initializes the RC522 module
 */
void rc522_init(void)
{
  SPI_Init();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  for(volatile int i=0;i<100000;i++); //Loop as delay since freeRTOS is being used.
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  for(volatile int i=0;i<100000;i++);
  rc522_reset();

  rc522_regWrite8(MFRC522_REG_T_MODE, 0x80);
  rc522_regWrite8(MFRC522_REG_T_PRESCALER, 0xA9);
  rc522_regWrite8(MFRC522_REG_T_RELOAD_L, 0xE8);
  rc522_regWrite8(MFRC522_REG_T_RELOAD_H, 0x03);


  rc522_regWrite8(MFRC522_REG_TX_AUTO, 0x40);
  rc522_regWrite8(MFRC522_REG_MODE, 0x3D);

  rc522_antennaON();   //Open the antenna
}

