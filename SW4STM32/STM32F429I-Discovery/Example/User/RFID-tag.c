/*
 * RFID-tag.c
 *
 *  Created on: Jan 9, 2022
 *      Author: patriciobulic
 */


/* Includes ------------------------------------------------------------------*/
#include "RFID-tag.h"
#include "MFRC522.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const uint8_t RFIDcodesSize4[NUM_CODES_SIZE4][SIZE4] = {
	{0xe6, 0xd4, 0x5e, 0xaf} ,
	{0xbb, 0x49, 0x65, 0x0a} ,
	{0x99, 0x0b, 0xfe, 0x4b} ,
	{0xb9, 0x74, 0x07, 0x7f} ,
	{0x57, 0xd5, 0x8e, 0x38}  // Blaz Fit-FIt
};


const uint8_t RFIDcodesSize7[NUM_CODES_SIZE7][SIZE7] = {
	{0x04, 0x3e, 0x0b, 0xea, 0x3b, 0x38, 0x80}
};




/* Private function prototypes -----------------------------------------------*/
/* Public functions ----------------------------------------------------------*/


StatusCode_t TAG_WriteName(char *name, MIFARE_Key *key, auth_key_t authkey)
{
	uint8_t len = strlen(name);
	if (len > 16)
		len = 16;
	//uint8_t out[PICC_BLOCK_SIZE];
	char out[PICC_BLOCK_SIZE];
	strcpy(out, name);
	for (int i = strlen(name); i < PICC_BLOCK_SIZE; i++)
		out[i] = 0x00;
	return PICC_WriteBlock(PICC_BLOCK_NAME, (uint8_t* )out, key, authkey);
}

StatusCode_t TAG_WriteSurname(char *name, MIFARE_Key *key, auth_key_t authkey)
{
	uint8_t len = strlen(name);
	if (len > 16)
		len = 16;
	char out[PICC_BLOCK_SIZE];
	strcpy(out, name);
	for (int i = strlen(name); i < PICC_BLOCK_SIZE; i++)
		out[i] = 0x00;
	return PICC_WriteBlock(PICC_BLOCK_SURNAME, (uint8_t* )out, key, authkey);
}


StatusCode_t TAG_WritePin(uint8_t *pin, MIFARE_Key *key, auth_key_t authkey)
{
	uint8_t config[PICC_BLOCK_SIZE+2];
	TAG_ReadConfig(config, key, authkey);
	for (uint8_t i = 0; i < PIN_LENGTH; i++)
		config[(PICC_BLOCK_SIZE-PIN_LENGTH)+i] = pin[i];
	return PICC_WriteBlock(PICC_BLOCK_CONFIG, config, key, authkey);
}


StatusCode_t TAG_ReadPin(uint8_t *pin, MIFARE_Key *key, auth_key_t authkey)
{
	char in[PICC_BLOCK_SIZE+2];
	StatusCode_t status = PICC_ReadBlock(PICC_BLOCK_CONFIG, (uint8_t*)in, key, authkey);
	for (int i = (PICC_BLOCK_SIZE-PIN_LENGTH); i < PICC_BLOCK_SIZE; i++) {
		pin[i-12] = in[i];
	}
	return status;
}


StatusCode_t TAG_ReadConfig(uint8_t *config, MIFARE_Key *key, auth_key_t authkey)
{
	char in[PICC_BLOCK_SIZE+2];
	StatusCode_t status = PICC_ReadBlock(PICC_BLOCK_CONFIG, (uint8_t* )in, key, authkey);
	strcpy((char* )config, in);
	return status;
}

StatusCode_t TAG_ReadName(char *name, MIFARE_Key *key, auth_key_t authkey)
{
	char in[PICC_BLOCK_SIZE+2];
	StatusCode_t status = PICC_ReadBlock(PICC_BLOCK_NAME, (uint8_t*)in, key, authkey);
	strcpy(name, in);
	return status;
}

StatusCode_t TAG_ReadSurname(char *name, MIFARE_Key *key, auth_key_t authkey)
{
	char in[PICC_BLOCK_SIZE+2];
	StatusCode_t status = PICC_ReadBlock(PICC_BLOCK_SURNAME, (uint8_t *)in, key, authkey);
	strcpy(name, in);
	return status;
}

StatusCode_t TAG_ReadTrailer(uint8_t block, char *trail, MIFARE_Key *key, auth_key_t authkey)
{
	char in[PICC_BLOCK_SIZE+2];
	//StatusCode_t status = PICC_ReadBlock(block, (uint8_t*)in, key);
	StatusCode_t status = PICC_ReadBlock(block, (uint8_t*)trail, key, authkey);
	strcpy(trail, in);
	return status;
}



StatusCode_t TAG_CheckID(uint8_t *id, uint8_t* codeTable, uint8_t numCodes, uint8_t codeSize, int8_t* indexUID)
{
	uint8_t id_c = 0;
	for (uint8_t i = 0; i < numCodes; i++) {
		for (uint8_t j = 0; j < codeSize; j++) {
			if (id[j] == *(codeTable + i*codeSize + j)){
				id_c++;
			}
		}
		if (id_c == codeSize) {
			*indexUID = i;		// index of the matched uid in the known UIDS table
			return STATUS_OK;
		} else {
			id_c = 0;
		}
	}
	return STATUS_INVALID;
}



StatusCode_t TAG_ReadMifareClassicSector(UID* uid, MIFARE_CLASSIC_1K_Card_t* card, uint8_t sector, MIFARE_Key *key, auth_key_t authkey){
	uint8_t in[PICC_BLOCK_SIZE+2];
	StatusCode_t status;
	uint8_t block;
	uint8_t firstblock = sector*4;
	uint8_t trailer = sector*4 + 3;


	// 1. Authenticate sector:
	if (authkey == AUTH_KEY_A) {
		status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, firstblock, key, uid);
	}
	else if (authkey == AUTH_KEY_B) {
		status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_B, firstblock, key, uid);
	}
	else return STATUS_ERROR;
	//status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_B, firstblock, key, uid);
	if (status != STATUS_OK) {
		printf("Authentication FAILED in Sector %d \n", sector);
		return status;
	}

	// 2. Read all four blocks from the authenticated sector:
	for (int j = 0; j < 4; j++){
		block = 4*sector + j;
		status = PICC_ReadBlock(block, (uint8_t*)in, key, authkey);
		if (status != STATUS_OK){
			//card->sector[i].isSectorBlocked = 1;
			return status;
			//break;
		}
		// copy read data to block:
		for (int k= 0; k<16; k++){
			card->sector[sector].Block[j][k] = in[k];
		}
	}

	return STATUS_OK;
}



StatusCode_t TAG_ReadMifareClassic1K(UID* uid,
		MIFARE_CLASSIC_1K_Card_t* card,
		uint8_t first_sector, 	// first secxtor to read (0...15)
		uint8_t last_sector, 	// last szecotr to read (0..15)
		MIFARE_Key *key,
		auth_key_t authkey){

	uint8_t in[PICC_BLOCK_SIZE+2];
	StatusCode_t status;
	uint8_t block;

	card->uid = uid;

	if (first_sector > last_sector) return STATUS_ERROR;


	// 1. clear the structure which will hold the card data:
	for (int i = 0; i < 16; i++){
		for (int j = 0; j < 4; j++){
			for (int k= 0; k<16; k++){
				card->sector[i].Block[j][k] = 0;
			}
		}
		card->sector[i].isSectorBlocked = 0;
	}

	// 2. read the Mifare card into the structure:
	for (int sector = first_sector; sector < last_sector+1; sector++){
		status = TAG_ReadMifareClassicSector(uid, card, sector, key, authkey);
	}
	PICC_HaltA(); // Halt the PICC before stopping the encrypted session.
	PCD_StopCrypto1();


	return STATUS_OK;
}


/*
 * Patricio Bulic, 17.1. - This is just a wrapper function
 */

StatusCode_t SetNewTrailerForSector(
		uint8_t sector,
		struct_block_access_bits* B0,
		struct_block_access_bits* B1,
		struct_block_access_bits* B2,
		struct_block_access_bits* TB,
		MIFARE_Key *current_key,
		MIFARE_Key *new_key
		)
{
	StatusCode_t status = PICC_WriteTrailerBlock((sector*4 + 3),B0,B1,B2,TB,current_key,new_key);
	return status;
}










/* Private functions ---------------------------------------------------------*/
