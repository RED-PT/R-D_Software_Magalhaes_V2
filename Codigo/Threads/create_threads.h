/*
 * create_threads.h
 *
 *  Created on: Oct 6, 2025
 *      Author: texman
 */

#ifndef INC_CREATE_THREADS_H_
#define INC_CREATE_THREADS_H_

//  C libraries
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

//  freertos libraries
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "cmsis_os2.h"
#include "ff.h"

//	our libraries
#include "config.h"

//	ID threads

// create threads function
void create_threads();

#endif /* INC_CREATE_THREADS_H_ */

