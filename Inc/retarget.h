/*
 * retarget.h
 *
 *  Created on: 16 Jan 2022
 *      Author: patriciobulic
 */

// All credit to Carmine Noviello for this code
// https://github.com/cnoviello/mastering-stm32/blob/master/nucleo-f030R8/system/include/retarget/retarget.h

#ifndef _RETARGET_H_
#define _RETARGET_H_

#include "stm32f4xx_hal.h"
//#include "stm32f4xx_nucleo.h"
#include <sys/stat.h>

void RetargetInit(UART_HandleTypeDef *huart);
int _isatty(int fd);
int _write(int fd, char* ptr, int len);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _read(int fd, char* ptr, int len);
int _fstat(int fd, struct stat* st);


#endif /* _RETARGET_H_ */
