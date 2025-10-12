/*
 * threads.c
 *
 *  Created on: Oct 8, 2025
 *      Author: texman
 */

// -- STANDARD libraries--
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
#include "math.h"

// -- Codigo --
#include "threads.h"
#include "create_threads.h"
#include "config.h"
#include "Flight Computer/flight_computer.h"
#include "defs.h"
#include "dma_msg.h"

// -- Modular Drivers --
#include "flags.h"
#include "malloc.h"
#include "buffer_read_write.h"
#include "conditional_variable.h"
#include "retarget.h"


