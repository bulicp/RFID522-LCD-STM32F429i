/*
 * RFID-tag.h
 *
 *  Created on: Jan 9, 2022
 *      Author: patriciobulic
 */

#ifndef RFID_TAG_H_
#define RFID_TAG_H_

/* Includes ------------------------------------------------------------------*/
//#include "general.h"
#include "main.h"
#include "MFRC522.h"
/* Exported typedef -----------------------------------------------------------*/

typedef struct {
	uint8_t Block[4][16];
	bool isSectorBlocked;
} MIFARE_CLASSIC_1K_Sector_t;

typedef struct {
	UID* uid;
	MIFARE_CLASSIC_1K_Sector_t sector[16];
} MIFARE_CLASSIC_1K_Card_t;


/* Exported define ------------------------------------------------------------*/
#define PICC_BLOCK_CONFIG 			2		// 3rd block in Sector 0
#define PICC_BLOCK_CONFIG_TRAIL 	3		// trailer for block 0
#define PICC_BLOCK_NAME 			60		// 1st block in Sector 1
#define PICC_BLOCK_SURNAME 			61		// 2nd block in Sector 1
#define PICC_BLOCK_NAME_TRAIL		7		// trailer for block 1
#define PICC_BLOCK_SIZE 			16

#define PIN_LENGTH 4
/* Exported macro -------------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
StatusCode_t TAG_WriteName(char *name, MIFARE_Key *key, auth_key_t authkey);
StatusCode_t TAG_WriteSurname(char *name, MIFARE_Key *key, auth_key_t authkey);
StatusCode_t TAG_WritePin(uint8_t *pin, MIFARE_Key *key, auth_key_t authkey);
StatusCode_t TAG_ReadPin(uint8_t *pin, MIFARE_Key *key, auth_key_t authkey);
StatusCode_t TAG_ReadConfig(uint8_t *config, MIFARE_Key *key, auth_key_t authkey);
StatusCode_t TAG_ReadName(char *name, MIFARE_Key *key, auth_key_t authkey);
StatusCode_t TAG_ReadSurname(char *name, MIFARE_Key *key, auth_key_t authkey);
StatusCode_t TAG_CheckID(uint8_t *id, uint8_t* codeTable, uint8_t numCodes, uint8_t codeSize, int8_t* indexUID);
StatusCode_t TAG_CheckIDSize4(uint8_t *id);
StatusCode_t TAG_CheckIDSize7(uint8_t *id);
StatusCode_t TAG_ReadTrailer(uint8_t block, char *trial, MIFARE_Key *key, auth_key_t authkey);
StatusCode_t TAG_ReadMifareClassicSector(UID* uid, MIFARE_CLASSIC_1K_Card_t* card, uint8_t sector, MIFARE_Key *key, auth_key_t authkey);
StatusCode_t TAG_ReadMifareClassic1K(UID* uid, MIFARE_CLASSIC_1K_Card_t* card, uint8_t first_sector,uint8_t last_sector,MIFARE_Key *key, auth_key_t authkey);
StatusCode_t SetNewTrailerForSector(
						uint8_t sector,
						struct_block_access_bits* B0,
						struct_block_access_bits* B1,
						struct_block_access_bits* B2,
						struct_block_access_bits* TB,
						MIFARE_Key *current_key,
						MIFARE_Key *new_key
						);


#endif /* RFID_TAG_H_ */
