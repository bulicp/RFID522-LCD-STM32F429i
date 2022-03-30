/*
 * MFRC522.c
 *
 *  Created on: Jan 7, 2022
 *      Author: patriciobulic
 */

/* Includes ------------------------------------------------------------------*/
//#include "main.h"
#include "MFRC522.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern UID uid;								// Used by PICC_ReadCardSerial().
/* Private function prototypes -----------------------------------------------*/
uint8_t MIFARE_TwoStepHelper(uint8_t command, uint8_t blockAddr, long data);
/* Private functions ---------------------------------------------------------*/




/* Public functions ----------------------------------------------------------*/

void PCD_WriteRegister(	uint8_t reg,		///< The register to write to. One of the PCD_Register enums.
						uint8_t value		///< The value to write.
						) {
		MFRC522_CS_LOW;				// Select slave
		SPI_TransmitByte(MFRC522_SPI, (reg & 0x7E));	// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
		SPI_TransmitByte(MFRC522_SPI, value);
		MFRC522_CS_HIGH;			// Release slave again
} // End PCD_WriteRegister()

/**
 * Writes a number of uint8_ts to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_WriteRegisterMulti( uint8_t reg,		///< The register to write to. One of the PCD_Register enums.
							 uint8_t count,		///< The number of uint8_ts to write to the register
							 uint8_t *values	///< The values to write. uint8_t array.
							) {
	MFRC522_CS_LOW;		// Select slave
	SPI_TransmitByte(MFRC522_SPI, (reg & 0x7E));	// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	for (uint8_t index = 0; index < count; index++) {
		SPI_TransmitByte(MFRC522_SPI, values[index]);
	}
	MFRC522_CS_HIGH;	// Release slave again
} // End PCD_WriteRegister()

/**
 * Reads a uint8_t from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
uint8_t PCD_ReadRegister( uint8_t reg	///< The register to read from. One of the PCD_Register enums.
						 ) {
	uint8_t value;
	MFRC522_CS_LOW;			// Select slave
	SPI_TransmitByte(MFRC522_SPI, (0x80 | (reg & 0x7E)));	// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	value = SPI_TransmitByte(MFRC522_SPI, 0);				// Read the value back. Send 0 to stop reading.
	MFRC522_CS_HIGH;		// Release slave again
	return value;
} // End PCD_ReadRegister()





/**
 * Reads a number of uint8_ts from the specified register in the MFRC522 chip.
 * The interface is described in the MFRC522 datasheet section 8.1.2.
 * Patricio Bulic, UL FRI, 17.1.2022
 */
void PCD_ReadRegisterMulti(	uint8_t reg,		///< The register to read from. One of the PCD_Register enums.
								uint8_t count,		///< The number of uint8_ts to read
								uint8_t *rcvData,	///< uint8_t array to store the values in.
								uint8_t rxAlign	    ///< Only bit positions rxAlign..7 in values[0] are updated.
								) {
	uint8_t dummy;		// this is actualy never used - only for the sake of code readability
	uint8_t readdata;
	if (count == 0) {
		return;
	}
	//Form the register address:
	uint8_t address = 0x80 | (reg & 0x7E);		// MSB == 1 is for reading. LSB is not used in address.
												// MFRC522 Datasheet section 8.1.2.3.
	uint8_t index = 0;							// Index in values array.
	MFRC522_CS_LOW;		// Select slave
	count--;								// One read is performed outside of the loop - this is dummy read
	dummy = SPI_TransmitByte(MFRC522_SPI, address);		// The first read is dummy - just tell MFRC522 which address we want to read

	while (index < count) {
		// Read value and tell that we want to read the same address again:
		readdata = SPI_TransmitByte(MFRC522_SPI, address);
		// If this is the first byte in the bit oriented frame, check where is the first received bit:
		if (index == 0 && rxAlign) { // Only update bit positions rxAlign..7 in rcvData[0]
			// Create bit mask for bit positions rxAlign..7
			uint8_t mask = 0;
			for (uint8_t i = rxAlign; i <= 7; i++) {
				mask |= (1 << i);
			}
			// Apply mask to both current value of rcvDatav[0] and new data and add the new data in rcvData.
			// clear RxAlign...7 bits in rcvData[0] and add new bits only at positions RxAlign...7:
			rcvData[0] = (rcvData[0] & ~mask) | (readdata & mask);
		}
		else { // Normal case
			rcvData[index] = readdata;	// Read value and tell that we want to read the same address again.
		}
		index++;
	}
	rcvData[index] = SPI_TransmitByte(MFRC522_SPI, 0);			// Read the final uint8_t. Send 0 to stop reading.
	MFRC522_CS_HIGH;			// Release slave again
} // End PCD_ReadRegister()





/**
 * Sets the bits given in mask in register reg.
 */
void PCD_SetRegisterBitMask( uint8_t reg,	///< The register to update. One of the PCD_Register enums.
							 uint8_t mask	///< The bits to set.
							) {
	uint8_t tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp | mask);			// set bit mask
} // End PCD_SetRegisterBitMask()

/**
 * Clears the bits given in mask from register reg.
 */
void PCD_ClearRegisterBitMask(	uint8_t reg,	///< The register to update. One of the PCD_Register enums.
								uint8_t mask	///< The bits to clear.
								) {
	uint8_t tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp & (~mask));		// clear bit mask
} // End PCD_ClearRegisterBitMask()


/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode_t PCD_CalculateCRC(
							uint8_t *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
							uint8_t length,		///< In: The number of uint8_ts to transfer.
							uint8_t *result		///< Out: Pointer to result buffer. Result is written to result[0..1], low uint8_t first.
					 	 ) {
	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
	PCD_WriteRegister(DivIrqReg, 0x04);					// Clear the CRCIRq interrupt request bit
	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);		// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegisterMulti(FIFODataReg, length, data);		// Write data to the FIFO
	PCD_WriteRegister(CommandReg, PCD_CalcCRC);		// Start the calculation

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73µs.
	int32_t i = 5000;
	uint8_t n;
	while (1) {
		n = PCD_ReadRegister(DivIrqReg);	// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq   reserved CRCIRq reserved reserved
		if (n & 0x04) {						// CRCIRq bit set - calculation done
			break;
		}
		if (--i == 0) {						// The emergency break. We will eventually terminate on this one after 89ms. Communication with the MFRC522 might be down.
			return STATUS_TIMEOUT;
		}
	}
	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop calculating CRC for new content in the FIFO.

	// Transfer the result from the registers to the result buffer
	result[0] = PCD_ReadRegister(CRCResultRegL);
	result[1] = PCD_ReadRegister(CRCResultRegH);
	return STATUS_OK;
} // End PCD_CalculateCRC()


/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Initializes the MFRC522 chip.
 */
void PCD_Init() {
	InitSPI(MFRC522_SPI);
	// Perform a soft reset
	PCD_Reset();

	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
    PCD_WriteRegister(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds

    PCD_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25µs.

    PCD_WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.

    PCD_WriteRegister(TReloadRegL, 0xE8);

	PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)

	PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)

} // End PCD_Init()



/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void PCD_Reset() {
	PCD_WriteRegister(CommandReg, PCD_SoftReset);	// Issue the SoftReset command.
	//uint8_t tmp = PCD_ReadRegister(CommandReg);
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg)
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74µs. Let us be generous: 50ms.
	HAL_Delay(50);
	//tmp = PCD_ReadRegister(CommandReg);
	// Wait for the PowerDown bit in CommandReg to be cleared
	while (PCD_ReadRegister(CommandReg) & (1<<4)) {
		// PCD still restarting - unlikely after waiting 50ms, but better safe than sorry.
	}
	//tmp = PCD_ReadRegister(CommandReg);
} // End PCD_Reset()



/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins disabled.
 */
void PCD_AntennaOn() {
	uint8_t value = PCD_ReadRegister(TxControlReg);
	if ((value & 0x03) != 0x03) {
		PCD_WriteRegister(TxControlReg, value | 0x03);
		uint8_t tmp = PCD_ReadRegister(TxControlReg);
	}
} // End PCD_AntennaOn()





/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode_t PCD_TransceiveData(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
							uint8_t sendLen,		///< Number of uint8_ts to transfer to the FIFO.
							uint8_t *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
							uint8_t *backLen,		///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
							uint8_t *validBits,		///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits. Default NULL.
							uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
							bool checkCRC			///< In: True => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
							) {
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()

/**
 * Transfers data to the MFRC522 FIFO, executes a commend, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode_t PCD_CommunicateWithPICC(uint8_t command,		///< The command to execute. One of the PCD_Command enums.
								uint8_t waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
								uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO.
								uint8_t sendLen,		///< Number of uint8_ts to transfer to the FIFO.
								uint8_t *backData,		///< NULL or pointer to buffer if data should be read back after executing the command.
								uint8_t *backLen,		///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
								uint8_t *validBits,		///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits.
								uint8_t rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
								bool checkCRC			///< In: True => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
								) {
	uint8_t n, _validBits;
	unsigned int i;

	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming	= (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
	PCD_WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	PCD_SetRegisterBitMask(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegisterMulti(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	PCD_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
	PCD_WriteRegister(CommandReg, command);			// Execute the command
	if (command == PCD_Transceive) 	{
		PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}

	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86µs.
	i = 10000;
	while (1) {
		n = PCD_ReadRegister(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq   HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			break;
		}
		if (n & 0x01) {	// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
		if (--i == 0) {						// The emergency break. If all other condions fail we will eventually terminate on this one after 35.7ms. Communication with the MFRC522 might be down.
			return STATUS_TIMEOUT;
		}
	}

	// Stop now if any errors except collisions were detected.
	uint8_t errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl   CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}

	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		n = PCD_ReadRegister(FIFOLevelReg);						// Number of uint8_ts in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;												// Number of uint8_ts returned
		PCD_ReadRegisterMulti(FIFODataReg, n, backData, rxAlign);		// Get received data from FIFO
		_validBits = PCD_ReadRegister(ControlReg) & 0x07;	// RxLastBits[2:0] indicates the number of valid bits in the last received uint8_t. If this value is 000b, the whole uint8_t is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}

	// Tell about collisions
	if (errorRegValue & 0x08) { // CollErr
		return STATUS_COLLISION;
	}

	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last uint8_t must be received.
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		uint8_t controlBuffer[2];
		n = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (n != STATUS_OK) {
			return n;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}

	return STATUS_OK;
} // End PCD_CommunicateWithPICC()

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode_t PICC_RequestA(uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
							uint8_t *bufferSize	///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
							) {
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode_t PICC_WakeupA(	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
						uint8_t *bufferSize	///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
					) {
	return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End PICC_WakeupA()


/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode_t PICC_REQA_or_WUPA(	uint8_t command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
							uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
							uint8_t *bufferSize	///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
						 ) {
	uint8_t validBits;
	StatusCode_t status;

	if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 uint8_ts long.
		return STATUS_NO_ROOM;
	}
	PCD_ClearRegisterBitMask(CollReg, 0x80);			// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;										// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) uint8_t. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits, 0, false);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // End PICC_REQA_or_WUPA()



/**
  * P.B., 2022
  *
  * Anticollision loop
  *
  */

StatusCode_t PICC_AnticollisionLoop(uint8_t CollisionLevel,
									UID* puid,
									uint8_t* anticollision_frame,
									uint8_t acframe_size,
									uint8_t* SAK_frame){

	//uint8_t anticollision_frame[9];					// The SELECT/ANTICOLLISION commands uses a 7 uint8_t anticolision frame + 2 uint8_for CRC_A
	//uint8_t SAK_frame[3];
	uint8_t nvb;
	//uint8_t uid_startindex;
	uint8_t collisionPos;
	//uint8_t totalNewValidBits;

	uint8_t bytecountUID;
	uint8_t bytecountNVB;
	uint8_t bitcountNVB;
	uint8_t Part1LengthInFIFO;
	uint8_t currentLevelKnownBits = 0;
	uint8_t txLastBits;

	uint8_t* pResponseBuffer;
	uint8_t responseBufferLength;
	uint8_t rxAlign;

	uint8_t result;

	StatusCode_t status;

	switch (CollisionLevel) {
		case PICC_CL1:
			anticollision_frame[0] = PICC_CMD_SEL_CL1;
			//uid_startindex = 0;
			break;
		case PICC_CL2:
			anticollision_frame[0] = PICC_CMD_SEL_CL2;
			//uid_startindex = 3;
			break;
		case PICC_CL3:
			anticollision_frame[0] = PICC_CMD_SEL_CL3;
			//uid_startindex = 6;
			break;
		default:
			return STATUS_INTERNAL_ERROR; // should not happen!!!
			break;
	}

	// Each level starts with no known bits of UID:
	nvb = 0x20; // the PCD will transmit no part of UID -- force PICC to respond with its complete UID
	// Set bit adjustments: For AC frames rxAlign = txLastBits (Figures 5 and 6 in ISO/IEC FCD 14443-3)
	txLastBits = 0;		// for nvb=0x20, all bits of the last byte should be transmitted
	currentLevelKnownBits = 0;

	// prepare frames and buffers
	bytecountUID	= currentLevelKnownBits / 8;	// Number of whole uint8_ts in the UID part.
	bytecountNVB	= 2 + bytecountUID;				// Number of whole uint8_ts: SEL + NVB + UIDs
	bitcountNVB		= currentLevelKnownBits % 8;
	anticollision_frame[1]	= (bytecountNVB << 4) + bitcountNVB	;	// NVB - Number of Valid Bits

	Part1LengthInFIFO	= bytecountNVB + (bitcountNVB ? 1 : 0); // length of Part1 in full bytes in FIFO
	// Having a seperate variable is overkill. But it makes the code easier to read:
	txLastBits			= bitcountNVB; // Transmit only txLastBits from FIFO

	// Store response in the unused part of anticolision frame buffer
	pResponseBuffer	= &anticollision_frame[bytecountNVB]; // pointer to the first byte to receive response from FIFO
	responseBufferLength = acframe_size - bytecountNVB;  // length of the response buffer
	// Having a seperate variable is overkill. But it makes the code easier to read:
	rxAlign = txLastBits;	// the first bit to be received is stored in FIFO at bit position txLastBits+1


	while(1) {
		// Prepare MFRC522 for transcieve:
		PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
		PCD_ClearRegisterBitMask(CollReg, 0x80);  // ValuesAfterColl=1 => Bits received after collision will be cleared.
		// Transmit Anticollision command in anticollision frame
		status = PCD_TransceiveData(anticollision_frame,
									Part1LengthInFIFO,
									pResponseBuffer,
									&responseBufferLength,
									&txLastBits,
									rxAlign,
									false);

		// Detect collision:
		if (status == STATUS_COLLISION) { // More than one PICC in the field => collision.
			result = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
			// First check if we have the valid position of collision:
			if (result & 0x20) { // CollPosNotValid
				return STATUS_INV_COLL_POS; // Without a valid collision position we cannot continue
			}
			// find out the collision position:
			collisionPos = result & 0x1F; // Values 0-31, 0 means bit 32.
			if (collisionPos == 0) {
				collisionPos = 32;
			}
			if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen
				return STATUS_INTERNAL_ERROR;
			}

			currentLevelKnownBits = currentLevelKnownBits + collisionPos;


			// prepare frames and buffers for the new reception:
			bytecountUID	= currentLevelKnownBits / 8;	// Number of whole uint8_ts in the UID part.
			bytecountNVB	= 2 + bytecountUID;				// Number of whole uint8_ts: SEL + NVB + UIDs
			bitcountNVB		= currentLevelKnownBits % 8;
			anticollision_frame[1]	= (bytecountNVB << 4) + bitcountNVB	;	// NVB - Number of Valid Bits

			// I decided to choose the PICC with the bit set
			//    - its position is bitcountNVB within the last byte in the anticollision frame:
			anticollision_frame[bytecountNVB] |= (1 << bitcountNVB);

			Part1LengthInFIFO	= bytecountNVB + (bitcountNVB ? 1 : 0); // length of Part1 in full bytes in FIFO
			// Having a seperate variable is overkill. But it makes the code easier to read:
			txLastBits			= bitcountNVB; // Transmit only txLastBits from FIFO

			// Store response in the unused part of anticolision frame buffer
			pResponseBuffer	= &anticollision_frame[bytecountNVB]; // pointer to the first byte to receive response from FIFO
			responseBufferLength = acframe_size - bytecountNVB;  // length of the response buffer
			// Having a seperate variable is overkill. But it makes the code easier to read:
			rxAlign = txLastBits;	// the first bit to be received is stored in FIFO at bit position txLastBits+1
		}
		else if (status != STATUS_OK) {  // in case of error return the status
			return status;
		}
		else { // STATUS_OK - no more colision detected
			// We now have all 32 bits of the UID in this Cascade Level
			currentLevelKnownBits = 32;
			break;
			// Issue the SELECT command outside the loop
		}
	} // while (collision)


	// There is no more collisions and we have to send the SELECT command:
	anticollision_frame[1] = 0x70; // NVB - Number of Valid Bits: Seven whole uint8_ts
	// Calculate BCC - Block Check Character
	anticollision_frame[6] = anticollision_frame[2] ^ anticollision_frame[3] ^ anticollision_frame[4] ^ anticollision_frame[5];
	// Calculate CRC_A
	status = PCD_CalculateCRC(anticollision_frame, 7, &anticollision_frame[7]);
	if (status != STATUS_OK) {
		return status;
	}
	txLastBits				= 0; // 0 => All 8 bits of the last byte will be transmitted.
	Part1LengthInFIFO		= 9; // the length of the SELECT frame is 9: SEL-NVB + 4 UIDs + BCC + 2 CRC
	responseBufferLength	= 3; // SAK response is 3-byte long

	rxAlign = txLastBits; // the first bit to be received is stored in FIFO at bit position txLastBits (0 in this case)
	PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(anticollision_frame,
								Part1LengthInFIFO,
								SAK_frame,
								&responseBufferLength,
								&txLastBits,
								rxAlign,
								false);

	if (SAK_frame[0] & 0x04) { // Cascade bit set: UID not complete
		return STATUS_UID_NOTCOMPLETE;
	}
	else if ((SAK_frame[0] & 0x20) && !(SAK_frame[0] & 0x04)) { // UID complete, PICC compliant with ISO/IEC 14443-4
		return STATUS_UID_COMPLETE;
	}
	else if (!(SAK_frame[0] & 0x20) && !(SAK_frame[0] & 0x04)) { // UID complete, PICC non-compliant with ISO/IEC 14443-4
			return STATUS_UID_COMPLETE_NC;
		}
	else {
		return STATUS_ERROR;
	}


	return STATUS_ERROR; // should not get here!
}




/**
 * P.B. 2022
 *
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 *
 * A PICC UID consists of 4, 7 or 10 uint8_ts.
 * Only 4 uint8_ts can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID uint8_ts		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode_t PICC_Select(	UID *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
						uint8_t validBits	///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
						) {

		// Description of anticollision_frame_CLx structure:
		// 		uint8_t 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
		// 		uint8_t 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete uint8_ts, Low nibble: Extra bits.
		// 		uint8_t 2: UID-data or CT		See explanation below. CT means Cascade Tag.
		// 		uint8_t 3: UID-data
		// 		uint8_t 4: UID-data
		// 		uint8_t 5: UID-data
		// 		uint8_t 6: BCC					Block Check Character - XOR of uint8_ts 2-5
		//		uint8_t 7: CRC_A
		//		uint8_t 8: CRC_A
		// The BCC and CRC_A is only transmitted if we know all the UID bits of the current Cascade Level.
		//
		// Description of uint8_ts 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
		//		UID size	Cascade level	uint8_t2	uint8_t3	uint8_t4	uint8_t5
		//		========	=============	=====	=====	=====	=====
		//		 4 uint8_ts		1			uid0	uid1	uid2	uid3
		//		 7 uint8_ts		1			CT		uid0	uid1	uid2
		//						2			uid3	uid4	uid5	uid6
		//		10 uint8_ts		1			CT		uid0	uid1	uid2
		//						2			CT		uid3	uid4	uid5
		//						3			uid6	uid7	uid8	uid9

	uint8_t anticollision_frame_CL1[9];
	uint8_t anticollision_frame_CL2[9];
	uint8_t anticollision_frame_CL3[9];
	uint8_t SAK_frame[3];

	uint8_t acframe_size = sizeof(anticollision_frame_CL1) / sizeof(anticollision_frame_CL1[0]);

	StatusCode_t status = PICC_AnticollisionLoop(PICC_CL1, uid, anticollision_frame_CL1, acframe_size, SAK_frame);
	if (status == STATUS_UID_NOTCOMPLETE) {
		status = PICC_AnticollisionLoop(PICC_CL2, uid, anticollision_frame_CL2, acframe_size, SAK_frame);
		if (status == STATUS_UID_NOTCOMPLETE) {
			status = PICC_AnticollisionLoop(PICC_CL3, uid, anticollision_frame_CL3, acframe_size, SAK_frame);
			uid->size = 10;
		}
		else{
			uid->size = 7;
		}
	}
	else{
		uid->size = 4;
	}

	for (int i = 0; i<10; i++) uid->uids[i] = 0;

	// Copy UIDs form anticollision frames into uid buffer:
	if (uid->size == 4){
		for (int i = 0; i < 4; i++){
			uid->uids[i] = anticollision_frame_CL1[i+2];
		}
	}
	if (uid->size == 7){
		for (int i = 0; i<3; i++){
			uid->uids[i] = anticollision_frame_CL1[i+3];
		}
		for (int i = 0; i<4; i++){
			uid->uids[i+3] = anticollision_frame_CL2[i+2];
		}
	}
	// Copy final SAK frame:
	for (int i = 0; i<3; i++){
		uid->sak[i] = SAK_frame[i];
	}
	return status;
}






/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode_t PICC_HaltA() {
	StatusCode_t status;
	uint8_t buffer[4];

	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	status = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (status != STATUS_OK) {
		return status;
	}

	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	status = PCD_TransceiveData(buffer, sizeof(buffer), NULL, 0, NULL, 0, false);
	if (status == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (status == STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return status;
} // End PICC_HaltA()




/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with MIFARE PICCs
/////////////////////////////////////////////////////////////////////////////////////
/**
 * Executes the MFRC522 MFAuthent command.
 * This command manages MIFARE authentication to enable a secure communication to any MIFARE Mini, MIFARE 1K and MIFARE 4K card.
 * The authentication is described in the MFRC522 datasheet section 10.3.1.9 and http://www.nxp.com/documents/data_sheet/MF1S503x.pdf section 10.1.
 * For use with MIFARE Classic PICCs.
 * The PICC must be selected - ie in state ACTIVE(*) - before calling this function.
 * Remember to call PCD_StopCrypto1() after communicating with the authenticated PICC - otherwise no new communications can start.
 *
 * All keys are set to FFFFFFFFFFFFh at chip delivery. A key consists of 6 uint8_ts.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise. Probably STATUS_TIMEOUT if you supply the wrong key.
 */
StatusCode_t PCD_Authenticate(	uint8_t command,		///< PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
							uint8_t blockAddr, 	///< The block number. See numbering in the comments in the .h file.
							MIFARE_Key *key,	///< Pointer to the Crypto key to use (6 uint8_ts)
							UID *uid			///< Pointer to Uid struct. The first 4 uint8_ts of the UID is used.
							) {
	uint8_t waitIRq = 0x10;		// IdleIRq

	// Build command buffer
	uint8_t sendData[12];
	sendData[0] = command;
	sendData[1] = blockAddr;
	// copy authentication key into sendData[]:
	if (command == PICC_CMD_MF_AUTH_KEY_A){
		for (uint8_t i = 0; i < MF_KEY_SIZE; i++) {	// 6 key uint8_ts
			sendData[2+i] = key->keyA[i];
		}
	}
	else if (command == PICC_CMD_MF_AUTH_KEY_B){
		for (uint8_t i = 0; i < MF_KEY_SIZE; i++) {	// 6 key uint8_ts
			sendData[2+i] = key->keyB[i];
		}
	}
	else return STATUS_ERROR;

	// copy UID into authentication data
	for (uint8_t i = 0; i < 4; i++) {				// The first 4 uint8_ts of the UID
		sendData[8+i] = uid->uids[i];
	}

	// Start the authentication.
	return PCD_CommunicateWithPICC(PCD_MFAuthent, waitIRq, &sendData[0], sizeof(sendData), NULL, 0, 0, 0, false);
} // End PCD_Authenticate()

/**
 * Used to exit the PCD from its authenticated state.
 * Remember to call this function after communicating with an authenticated PICC - otherwise no new communications can start.
 */
void PCD_StopCrypto1() {
	// Clear MFCrypto1On bit
	PCD_ClearRegisterBitMask(Status2Reg, 0x08); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved   MFCrypto1On ModemState[2:0]
} // End PCD_StopCrypto1()

/**
 * Reads 16 uint8_ts (+ 2 uint8_ts CRC_A) from the active PICC.
 *
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 *
 * For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
 * The MF0ICU1 returns a NAK for higher addresses.
 * The MF0ICU1 responds to the READ command by sending 16 uint8_ts starting from the page address defined by the command argument.
 * For example; if blockAddr is 03h then pages 03h, 04h, 05h, 06h are returned.
 * A roll-back is implemented: If blockAddr is 0Eh, then the contents of pages 0Eh, 0Fh, 00h and 01h are returned.
 *
 * The buffer must be at least 18 uint8_ts because a CRC_A is also returned.
 * Checks the CRC_A before returning STATUS_OK.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode_t MIFARE_Read(uint8_t blockAddr, 	///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
					uint8_t *buffer,		///< The buffer to store the data in
					uint8_t *bufferSize	///< Buffer size, at least 18 uint8_ts. Also number of uint8_ts returned if STATUS_OK.
					) {
	StatusCode_t status;

	// Sanity check
	if (buffer == NULL || *bufferSize < 18) {
		return STATUS_NO_ROOM;
	}

	// Build command buffer
	buffer[0] = PICC_CMD_MF_READ;
	buffer[1] = blockAddr;
	// Calculate CRC_A
	status = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (status != STATUS_OK) {
		return status;
	}

	// Transmit the buffer and receive the response, validate CRC_A.
	return PCD_TransceiveData(buffer, 4, buffer, bufferSize, NULL, 0, true);
} // End MIFARE_Read()

/**
 * Writes 16 uint8_ts to the active PICC.
 *
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 *
 * For MIFARE Ultralight the opretaion is called "COMPATIBILITY WRITE".
 * Even though 16 uint8_ts are transferred to the Ultralight PICC, only the least significant 4 uint8_ts (uint8_ts 0 to 3)
 * are written to the specified address. It is recommended to set the remaining uint8_ts 04h to 0Fh to all logic 0.
 * *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode_t MIFARE_Write(	 uint8_t blockAddr, ///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The page (2-15) to write to.
					 	 uint8_t *buffer,	///< The 16 uint8_ts to write to the PICC
					 	 uint8_t bufferSize	///< Buffer size, must be at least 16 uint8_ts. Exactly 16 uint8_ts are written.
						) {
	StatusCode_t status;

	// Sanity check
	if (buffer == NULL || bufferSize < 16) {
		return STATUS_INVALID;
	}

	// Mifare Classic protocol requires two communications to perform a write.
	// Step 1: Tell the PICC we want to write to block blockAddr.
	uint8_t cmdBuffer[2];
	cmdBuffer[0] = PICC_CMD_MF_WRITE;
	cmdBuffer[1] = blockAddr;
	status = PCD_MIFARE_Transceive(cmdBuffer, 2, true); // Adds CRC_A and checks that the response is MF_ACK.
	if (status != STATUS_OK) {
		return status;
	}

	// Step 2: Transfer the data
	status = PCD_MIFARE_Transceive(	buffer, bufferSize, true); // Adds CRC_A and checks that the response is MF_ACK.
	if (status != STATUS_OK) {
		return status;
	}

	return STATUS_OK;
} // End MIFARE_Write()

/**
 * Writes a 4 uint8_t page to the active MIFARE Ultralight PICC.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode_t MIFARE_Ultralight_Write(uint8_t page, 		///< The page (2-15) to write to.
								uint8_t *buffer,	///< The 4 uint8_ts to write to the PICC
								uint8_t bufferSize	///< Buffer size, must be at least 4 uint8_ts. Exactly 4 uint8_ts are written.
								) {
	StatusCode_t status;

	// Sanity check
	if (buffer == NULL || bufferSize < 4) {
		return STATUS_INVALID;
	}

	// Build commmand buffer
	uint8_t cmdBuffer[6];
	cmdBuffer[0] = PICC_CMD_UL_WRITE;
	cmdBuffer[1] = page;
	memcpy(&cmdBuffer[2], buffer, 4);

	// Perform the write
	status = PCD_MIFARE_Transceive(cmdBuffer, 6, true); // Adds CRC_A and checks that the response is MF_ACK.
	if (status != STATUS_OK) {
		return status;
	}
	return STATUS_OK;
} // End MIFARE_Ultralight_Write()

/**
 * MIFARE Decrement subtracts the delta from the value of the addressed block, and stores the result in a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode_t MIFARE_Decrement(	uint8_t blockAddr, ///< The block (0-0xff) number.
								long delta		///< This number is subtracted from the value of block blockAddr.
							) {
	return MIFARE_TwoStepHelper(PICC_CMD_MF_DECREMENT, blockAddr, delta);
} // End MIFARE_Decrement()

/**
 * MIFARE Increment adds the delta to the value of the addressed block, and stores the result in a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode_t MIFARE_Increment(	uint8_t blockAddr, ///< The block (0-0xff) number.
								long delta		///< This number is added to the value of block blockAddr.
							) {
	return MIFARE_TwoStepHelper(PICC_CMD_MF_INCREMENT, blockAddr, delta);
} // End MIFARE_Increment()

/**
 * MIFARE Restore copies the value of the addressed block into a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode_t MIFARE_Restore(	uint8_t blockAddr ///< The block (0-0xff) number.
							) {
	// The datasheet describes Restore as a two step operation, but does not explain what data to transfer in step 2.
	// Doing only a single step does not work, so I chose to transfer 0L in step two.
	return MIFARE_TwoStepHelper(PICC_CMD_MF_RESTORE, blockAddr, 0L);
} // End MIFARE_Restore()

/**
 * MIFARE Transfer writes the value stored in the volatile memory into one MIFARE Classic block.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode_t MIFARE_Transfer(	uint8_t blockAddr ///< The block (0-0xff) number.
							) {
	StatusCode_t status;
	uint8_t cmdBuffer[2]; // We only need room for 2 uint8_ts.

	// Tell the PICC we want to transfer the result into block blockAddr.
	cmdBuffer[0] = PICC_CMD_MF_TRANSFER;
	cmdBuffer[1] = blockAddr;
	status = PCD_MIFARE_Transceive(	cmdBuffer, 2, true); // Adds CRC_A and checks that the response is MF_ACK.
	if (status != STATUS_OK) {
		return status;
	}
	return STATUS_OK;
} // End MIFARE_Transfer()


/////////////////////////////////////////////////////////////////////////////////////
// Support functions
/////////////////////////////////////////////////////////////////////////////////////


/**
 * Helper function for the two-step MIFARE Classic protocol operations Decrement, Increment and Restore.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode_t MIFARE_TwoStepHelper(	uint8_t command,	///< The command to use
								uint8_t blockAddr,	///< The block (0-0xff) number.
								long data		///< The data to transfer in step 2
							) {
	StatusCode_t status;
	uint8_t cmdBuffer[2]; // We only need room for 2 uint8_ts.

	// Step 1: Tell the PICC the command and block address
	cmdBuffer[0] = command;
	cmdBuffer[1] = blockAddr;
	status = PCD_MIFARE_Transceive(	cmdBuffer, 2, false); // Adds CRC_A and checks that the response is MF_ACK.
	if (status != STATUS_OK) {
		return status;
	}

	// Step 2: Transfer the data
	status = PCD_MIFARE_Transceive(	(uint8_t *)&data, 4, true); // Adds CRC_A and accept timeout as success.
	if (status != STATUS_OK) {
		return status;
	}

	return STATUS_OK;
} // End MIFARE_TwoStepHelper()


/**
 * Wrapper for MIFARE protocol communication.
 * Adds CRC_A, executes the Transceive command and checks that the response is MF_ACK or a timeout.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode_t PCD_MIFARE_Transceive(	uint8_t *sendData,		///< Pointer to the data to transfer to the FIFO. Do NOT include the CRC_A.
								uint8_t sendLen,		///< Number of uint8_ts in sendData.
								bool acceptTimeout		///< True => A timeout is also success
								) {
	StatusCode_t status;
	uint8_t cmdBuffer[18]; // We need room for 16 uint8_ts data and 2 uint8_ts CRC_A.

	// Sanity check
	if (sendData == NULL || sendLen > 16) {
		return STATUS_INVALID;
	}

	// Copy sendData[] to cmdBuffer[] and add CRC_A
	memcpy(cmdBuffer, sendData, sendLen);
	status = PCD_CalculateCRC(cmdBuffer, sendLen, &cmdBuffer[sendLen]);
	if (status != STATUS_OK) {
		return status;
	}
	sendLen += 2;

	// Transceive the data, store the reply in cmdBuffer[]
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	uint8_t cmdBufferSize = sizeof(cmdBuffer);
	uint8_t validBits = 0;
	status = PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, cmdBuffer, sendLen, cmdBuffer, &cmdBufferSize, &validBits, 0, false);
	if (acceptTimeout && status == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (status != STATUS_OK) {
		return status;
	}
	// The PICC must reply with a 4 bit ACK
	if (cmdBufferSize != 1 || validBits != 4) {
		return STATUS_ERROR;
	}
	if (cmdBuffer[0] != MF_ACK) {
		return STATUS_MIFARE_NACK;
	}
	return STATUS_OK;
} // End PCD_MIFARE_Transceive()

/**
 * Returns a string pointer to a status code name.
 *
 */
const char *GetStatusCodeName(uint8_t code	///< One of the StatusCode enums.
										) {
	switch (code) {
		case STATUS_OK:				return "Success."; break;
		case STATUS_ERROR:			return "Error in communication."; break;
		case STATUS_COLLISION:		return "Collission detected."; break;
		case STATUS_TIMEOUT:		return "Timeout in communication."; break;
		case STATUS_NO_ROOM:		return "A buffer is not big enough."; break;
		case STATUS_INTERNAL_ERROR:	return "Internal error in the code. Should not happen."; break;
		case STATUS_INVALID:		return "Invalid argument."; break;
		case STATUS_CRC_WRONG:		return "The CRC_A does not match."; break;
		case STATUS_MIFARE_NACK:	return "A MIFARE PICC responded with NAK."; break;
		default:
			return "Unknown error";
			break;
	}
} // End GetStatusCodeName()

/**
 * Translates the SAK (Select Acknowledge) to a PICC type.
 *
 * @return PICC_Type
 */
uint8_t PICC_GetType(uint8_t sak		///< The SAK uint8_t returned from PICC_Select().
					) {
	if (sak & 0x04) { // UID not complete
		return PICC_TYPE_NOT_COMPLETE;
	}

	switch (sak) {
		case 0x09:	return PICC_TYPE_MIFARE_MINI;	break;
		case 0x08:	return PICC_TYPE_MIFARE_1K;		break;
		case 0x18:	return PICC_TYPE_MIFARE_4K;		break;
		case 0x00:	return PICC_TYPE_MIFARE_UL;		break;
		case 0x10:
		case 0x11:	return PICC_TYPE_MIFARE_PLUS;	break;
		case 0x01:	return PICC_TYPE_TNP3XXX;		break;
		default:	break;
	}

	if (sak & 0x20) {
		return PICC_TYPE_ISO_14443_4;
	}

	if (sak & 0x40) {
		return PICC_TYPE_ISO_18092;
	}

	return PICC_TYPE_UNKNOWN;
} // End PICC_GetType()

/**
 * Returns a string pointer to the PICC type name.
 *
 */
const char *PICC_GetTypeName(uint8_t piccType	///< One of the PICC_Type enums.
										) {
	switch (piccType) {
		case PICC_TYPE_ISO_14443_4:		return "PICC compliant with ISO/IEC 14443-4";		break;
		case PICC_TYPE_ISO_18092:		return "PICC compliant with ISO/IEC 18092 (NFC)";	break;
		case PICC_TYPE_MIFARE_MINI:		return "MIFARE Mini, 320 uint8_ts";					break;
		case PICC_TYPE_MIFARE_1K:		return "MIFARE 1KB";								break;
		case PICC_TYPE_MIFARE_4K:		return "MIFARE 4KB";								break;
		case PICC_TYPE_MIFARE_UL:		return "MIFARE Ultralight or Ultralight C";			break;
		case PICC_TYPE_MIFARE_PLUS:		return "MIFARE Plus";								break;
		case PICC_TYPE_TNP3XXX:			return "MIFARE TNP3XXX";							break;
		case PICC_TYPE_NOT_COMPLETE:	return "SAK indicates UID is not complete.";		break;
		case PICC_TYPE_UNKNOWN:
		default:						return "Unknown type";								break;
	}
} // End PICC_GetTypeName()


/**
 * P.B., 2022
 * Calculates the bit pattern needed for the specified access bits.
 */
void MIFARE_SetAccessBits(
		uint8_t* accessBitBuffer,	///< Pointer to uint8_t 6, 7 and 8 in the sector trailer. Bytes [0..2] will be set.
		struct_block_access_bits* B0,	///< Access bits [C1 C2 C3] for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
		struct_block_access_bits* B1,	///< Access bits C1 C2 C3] for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
		struct_block_access_bits* B2,	///< Access bits C1 C2 C3] for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
		struct_block_access_bits* TB		///< Access bits C1 C2 C3] for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
		) {


	uint8_t C1 = (TB->c1bit << 3) | (B2->c1bit << 2) | (B1->c1bit << 1) | (B0->c1bit << 3);
	uint8_t C2 = (TB->c2bit << 3) | (B2->c2bit << 2) | (B1->c2bit << 1) | (B0->c2bit << 3);
	uint8_t C3 = (TB->c3bit << 3) | (B2->c3bit << 2) | (B1->c3bit << 1) | (B0->c3bit << 3);

	// Byte 6 in the trailer block:
	accessBitBuffer[0] = 	((~C2 << 4) & 0xF0) | (~C1 & 0xF);
	// Byte 7 in the trailer block:
	accessBitBuffer[1] =  	(( C1 << 4) & 0xF0) | (~C3 & 0xF);
	// Byte 8 in the trailer block:
	accessBitBuffer[2] =  	(( C3 << 4) & 0xF0) | ( C2 & 0xF);

} // End MIFARE_SetAccessBits()




/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 *
 * @return bool
 */
bool PICC_IsNewCardPresent() {
	uint8_t bufferATQA[2];
	uint8_t bufferSize = sizeof(bufferATQA);
	StatusCode_t status = PICC_RequestA(bufferATQA, &bufferSize);
	return (status == STATUS_OK || status == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

/**
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 *
 * @return bool
 */
bool PICC_ReadCardSerial(UID* uid) {
	//uint8_t result = PICC_Select(uid, 0);
	StatusCode_t status = PICC_Select(uid, 0);

	return ((status == STATUS_UID_COMPLETE) || (status == STATUS_UID_COMPLETE_NC)); //return a '1' if PICC_Select returns a valid UID, else a '0'
} // End PICC_ReadCardSerial()




/**
  * P.B., 2022
  *
  *
  */
StatusCode_t PICC_WriteBlock(int blockNumber, uint8_t arrayAddress[], MIFARE_Key *key, auth_key_t authkey)
{
  //this makes sure that we only write into data blocks. Every 4th block is a trailer block for the access/security info.
  //int largestModulo4Number=blockNumber/4*4;
  int largestModulo4Number=blockNumber/4*4;
  int trailerBlock=largestModulo4Number+3;//determine trailer block for the sector

  StatusCode_t status;

  // DO NOT ALLOW writes in the trailer block:
  if (blockNumber > 2 && (blockNumber+1)%4 == 0){
	  //printf("%d is a trailer block:", blockNumber);
	  return STATUS_ERROR;
  }//block number is a trailer block (modulo 4); quit and send error code 2
  //printf("%d is a data block:", blockNumber);

  /*****************************************authentication of the desired block for access***********************************************************/
  //uint8_t PCD_Authenticate(uint8_t command, uint8_t blockAddr, MIFARE_Key *key, Uid *uid);
  //this method is used to authenticate a certain block for writing or reading
  //command: See enumerations above -> PICC_CMD_MF_AUTH_KEY_A	= 0x60 (=1100000),		// this command performs authentication with Key A
  //blockAddr is the number of the block from 0 to 15.
  //MIFARE_Key *key is a pointer to the MIFARE_Key struct defined above, this struct needs to be defined for each block. New cards have all A/B= FF FF FF FF FF FF
  //Uid *uid is a pointer to the UID struct that contains the user ID of the card.
  if (authkey == AUTH_KEY_A) {
	  status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, trailerBlock, key, &(uid));
  }
  else if (authkey == AUTH_KEY_B) {
	  status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_B, trailerBlock, key, &(uid));
  }
  else return STATUS_ERROR;
  if (status != STATUS_OK) {
	  return STATUS_AUTH_FAILED;
  }
  //it appears the authentication needs to be made before every block read/write within a specific sector.
  //If a different sector is being authenticated access to the previous one is lost.


  /*****************************************writing the block***********************************************************/

  status = MIFARE_Write(blockNumber, arrayAddress, 16);//valueBlockA is the block number, MIFARE_Write(block number (0-15), uint8_t array containing 16 values, number of uint8_ts in block (=16))
  //status = mfrc522.MIFARE_Write(9, value1Block, 16);
  if (status != STATUS_OK) {
	  //printf("MIFARE_Write() failed: %s\r\n", GetStatusCodeName(status));
	  return STATUS_MIFARE_W_FAIL;
  }
  //printf("block was written\r\n");
  return status;
}


/**
  * P.B., 2022
  *
  *
  */
StatusCode_t PICC_WriteTrailerBlock(
		int blockNumber,
		struct_block_access_bits* B0,
		struct_block_access_bits* B1,
		struct_block_access_bits* B2,
		struct_block_access_bits* TB,
		MIFARE_Key *current_key,
		MIFARE_Key *new_key
		)
{
  uint8_t trailer[16];

  // Allow writes to trailer blocks only:
  if ( !((blockNumber+1)%4 == 0) ){
	  // not a trailer block
	  return STATUS_ERROR;
  }//block number is not a trailer block (modulo 4); quit and send error code

  MIFARE_SetAccessBits(&trailer[6], B0, B1, B2, TB);

  for (int i = 0; i<6; i++){
	  trailer[i] 		= new_key->keyA[i];
	  trailer[i+10] 	= new_key->keyB[i];
  }


  /*****************************************authentication of the desired block for access***********************************************************/
  uint8_t status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, blockNumber, current_key, &(uid));
  //uint8_t PCD_Authenticate(uint8_t command, uint8_t blockAddr, MIFARE_Key *key, Uid *uid);
  //this method is used to authenticate a certain block for writing or reading
  //command: See enumerations above -> PICC_CMD_MF_AUTH_KEY_A	= 0x60 (=1100000),		// this command performs authentication with Key A
  //blockAddr is the number of the block from 0 to 15.
  //MIFARE_Key *key is a pointer to the MIFARE_Key struct defined above, this struct needs to be defined for each block. New cards have all A/B= FF FF FF FF FF FF
  //Uid *uid is a pointer to the UID struct that contains the user ID of the card.
  if (status != STATUS_OK) {
	  //printf("PCD_Authenticate() failed: %s\r\n", GetStatusCodeName(status));
	  return STATUS_ERROR;
  }
  //it appears the authentication needs to be made before every block read/write within a specific sector.
  //If a different sector is being authenticated access to the previous one is lost.

  /*****************************************writing the block***********************************************************/

  status = MIFARE_Write(blockNumber, trailer, 16); // number of uint8_ts in block (=16))
  if (status != STATUS_OK) {
	  return STATUS_MIFARE_W_FAIL;
  }
  return status;
}



/**
  * P.B., 2022
  *
  *
  */
StatusCode_t PICC_ReadBlock(int blockNumber, uint8_t arrayAddress[], MIFARE_Key *key, auth_key_t authkey)
{
  int largestModulo4Number=blockNumber/4*4;
  int trailerBlock=largestModulo4Number+3;//determine trailer block for the sector
  StatusCode_t status;

  /*****************************************authentication of the desired block for access***********************************************************/
  //uint8_t PCD_Authenticate(uint8_t command, uint8_t blockAddr, MIFARE_Key *key, Uid *uid);
  //this method is used to authenticate a certain block for writing or reading
  //command: See enumerations above -> PICC_CMD_MF_AUTH_KEY_A	= 0x60 (=1100000),		// this command performs authentication with Key A
  //blockAddr is the number of the block from 0 to 15.
  //MIFARE_Key *key is a pointer to the MIFARE_Key struct defined above, this struct needs to be defined for each block. New cards have all A/B= FF FF FF FF FF FF
  //Uid *uid is a pointer to the UID struct that contains the user ID of the card.
  if (authkey == AUTH_KEY_A) {
	  status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, trailerBlock, key, &(uid));
  }
  else if (authkey == AUTH_KEY_B) {
	  status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_B, trailerBlock, key, &(uid));
  }
  else return STATUS_ERROR;
  if (status != STATUS_OK) {
	  //printf("PCD_Authenticate() failed (read): %s\r\n", GetStatusCodeName(status));
      return STATUS_AUTH_FAILED;
  }
  //it appears the authentication needs to be made before every block read/write within a specific sector.
  //If a different sector is being authenticated access to the previous one is lost.


  /*****************************************reading a block***********************************************************/

  uint8_t buffersize = 18;//we need to define a variable with the read buffer size, since the MIFARE_Read method below needs a pointer to the variable that contains the size...
  status = MIFARE_Read(blockNumber, arrayAddress, &buffersize);//&buffersize is a pointer to the buffersize variable; MIFARE_Read requires a pointer instead of just a number
  if (status != STATUS_OK) {
	  //printf("MIFARE_read() failed: %s\r\n", GetStatusCodeName(status));
	  return STATUS_MIFARE_R_FAIL;
  }
  //printf("block was read\r\n");
  return status;
}







