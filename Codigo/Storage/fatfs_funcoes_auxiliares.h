/*
 * fatfs_funcoes_auxiliares.h
 *
 *  Created on: 23/11/2022
 *      Author: Pipes' Laptop, Controllerstech
 */

#ifndef INC_FATFS_FUNCOES_AUXILIARES_H_
#define INC_FATFS_FUNCOES_AUXILIARES_H_

#include "fatfs_sd.h"
#include "main.h"
#include "cmsis_os2.h"
#include "ff.h"
#include "fatfs.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
#include "stdlib.h"
#include "malloc.h"
#include "stddef.h"

// flag de debug: se for true, vai fazer print dos resultados das operações
#define DEBUG_SD_CARD false
#define SD_CARD_BENCHMARK false
#define INVALID -1

#define FILENAME_MAX_LENGTH 30

typedef enum { FREE = false, TAKEN = true } allocation_state_t;

// estrutura que cria uma camada de abstração entre o FAT FS e o código a ser desenvolvido
typedef struct {
	char name[FILENAME_MAX_LENGTH];
	FIL open_file;
	uint16_t time_opened_ms;
	bool state;
} custom_open_file;
custom_open_file* get_open_file(int index);


/* mounts the sd card*/
void Mount_SD (const TCHAR* path);

/* unmounts the sd card*/
void Unmount_SD (const TCHAR* path);

/* Start node to be scanned (***also used as work area***) */
FRESULT Scan_SD (char* pat);

/* Only supports removing files from home directory. Directory remover to be added soon */
FRESULT Format_SD ();

/* checks the free space in the sd card*/
void Check_SD_Space ();

/* performs tests on the sd card*/
bool Test_SD ();

/* creates a directory
 * @ name: is the path to the directory
 */
FRESULT Create_Dir (char *name);

/* verifies if a file with the $path + '_X.txt' exists
 * if it exists, try again, but increment the number
 * if not, create a file and return the return of Create_File()
 * @ name : is the "template" name of the file*/
uint16_t File_Exists (char *name);

/* creates the file, if it does not exists
 * @ name : is the path to the file
 * @ return : index do ficheiro aberto*/
int Create_File (char *name);

/* Removes the file from the sd card
 * @ name : is the path to the file*/
FRESULT Remove_File (char *name);

//*** Functions using custom open file ***//

/* opens the file
 * @ name : is the path to the file
 * @ return : index do ficheiro aberto*/
int Open_File (char *name);

/* closes the file e remove-a da lista de ficheiros abertos
 * @ name : is the path to the file*/
void Close_File (int index);

/* write the data to the file
 * @ name : is the path to the file*/
void Write_File (int index, void* data, ssize_t offset, size_t len);

/* updates the data to the file, but opens the file periodically
 * @ name : is the path to the file
 * @ return: returns the index of the file stored in the open files array */
void Update_File (int index, void* data, size_t len);

/* NAO USAR  DIRETAMENTE - read data from the file - used by other functions
 * @ name : is the path to the file*/
void Read_File (int index, void* buffer, int offset, size_t len);

/* print data from the file
 * @ name : is the path to the file*/
//void Print_File (int index);

void Read_All_Files();

#endif /* INC_FATFS_FUNCOES_AUXILIARES_H_ */
