/*
 * fatfs_funcoes_auxiliares.c
 *
 *  Created on: 23/11/2022
 *      Author: Pipes' Laptop, Controllerstech
 */

#include "fatfs_funcoes_auxiliares.h"
#include "math.h"
//#include "test.h"

//extern UART_HandleTypeDef hua;
//#define UART &huart1

#define MAX_OPEN_FILES 15
#define TIME_THRESHOLD 1000 //ms

/* =============================>>>>>>>> NO CHANGES AFTER THIS LINE =====================================>>>>>>> */

#define BLOCK_SIZE 512

FATFS fs;  // file system
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count
bool sd_not_mounted = true;

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

/**** custom open file table *****/
custom_open_file open_file_table[MAX_OPEN_FILES];

// prototypes
void cof_table_init();
void cof_table_destroy();
int reserve_cof();
void add_to_cof(int index, char *name);
void remove_from_cof(int index);
custom_open_file* get_open_file(int index);

void return_sd_card_not_mounted() {
	if (DEBUG_SD_CARD)
		printf("SD card não está montado");
}

void Send_Uart(char *string) {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();

		return;
	}

	if (DEBUG_SD_CARD)
		printf(string);
}

void Mount_SD(const TCHAR *path) {
	fresult = f_mount(&fs, path, 1);
	if (fresult != FR_OK)
		printf("ERROR - %d!!! in mounting SD CARD...\r\n\n", fresult);
	else
		printf("SD CARD mounted successfully...\r\n");

	// initialize the open file table
	cof_table_init();

	sd_not_mounted = false;
}

void Unmount_SD(const TCHAR *path) {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();
		return;
	}

	fresult = f_mount(NULL, path, 1);
	if (fresult == FR_OK)
		Send_Uart("SD CARD UNMOUNTED successfully...\r\n\n\n");
	else
		Send_Uart("ERROR!!! in UNMOUNTING SD CARD\r\n\n\n");

	// destroy the open file table
	//cof_table_destroy();
}

/* Start node to be scanned (***also used as work area***) */
FRESULT Scan_SD(char *pat) {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();
		return INVALID;
	}

	DIR dir;
	UINT i;
	char buf[50];
	char fname[30];
	char path[20];
	sprintf(path, "%s", pat);

	fresult = f_opendir(&dir, path); /* Open the directory */
	if (fresult == FR_OK) {
		for (;;) {
			fresult = f_readdir(&dir, &fno); /* Read a directory item */
			if (fresult != FR_OK || fno.fname[0] == 0)
				break; /* Break on error or end of dir */
			if (fno.fattrib & AM_DIR) /* It is a directory */
			{
				if (!(strcmp("SYSTEM~1", fno.fname)))
					continue;
				strncpy(fname, fno.fname, 30);
				sprintf(buf, "Dir: %s\r\r\n", fname);
				Send_Uart(buf);
				i = strlen(path);
				sprintf(&path[i], "/%s", fno.fname);
				fresult = Scan_SD(path); /* Enter the directory */
				if (fresult != FR_OK)
					break;
				path[i] = 0;
			} else { /* It is a file. */
				strncpy(fname, fno.fname, 30);
				sprintf(buf, "File: %s/%s\r\n", path, fname);
				Send_Uart(buf);
			}
		}
		f_closedir(&dir);
	}
	return fresult;
}

/* Only supports removing files from home directory */
FRESULT Format_SD() {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();
		return INVALID;
	}

	DIR dir;
	char path[20];
	sprintf(path, "%s", "/");

	fresult = f_opendir(&dir, path); /* Open the directory */
	if (fresult == FR_OK) {
		for (;;) {
			fresult = f_readdir(&dir, &fno); /* Read a directory item */
			if (fresult != FR_OK || fno.fname[0] == 0)
				break; /* Break on error or end of dir */
			if (fno.fattrib & AM_DIR) /* It is a directory */
			{
				if (!(strcmp("SYSTEM~1", fno.fname)))
					continue;
				fresult = f_unlink(fno.fname);
				if (fresult == FR_DENIED)
					continue;
			} else { /* It is a file. */
				fresult = f_unlink(fno.fname);
			}
		}
		f_closedir(&dir);
	}
	return fresult;
}

void Check_SD_Space() {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();
		return;
	}

	/* Check free space */
	f_getfree("", &fre_clust, &pfs);

	total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	char buf[30];
	sprintf(buf, "SD CARD Total Size: \t%lu\r\r\n", total);
	Send_Uart(buf);
	free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);
	memset(buf, 0, 30);
	sprintf(buf, "SD CARD Free Space: \t%lu\r\r\n", free_space);
	Send_Uart(buf);
}

bool Test_SD() {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();
		return false;
	}

	//  FATFS sd_card;
	FIL open_file;
	char f_name[100];
	strcpy(f_name, "test-file");

	// verifies existance of file
//	if(f_stat(f_name, &finfo) != FR_OK)
//		Send_Uart("file not found\r\r\n");
//	else
//		Send_Uart("SD file found successfully...\r\r\n");

	// open file
	if (f_open(&open_file, f_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
		Send_Uart("Error openning file\r\r\n");
		return false;
	} else
		Send_Uart("SD File openned successfully...\r\r\n");

	/* Write text */
	f_puts("STM32 SD Card I/O Example via SPI\r\r\n", &open_file);
	f_puts("Hello world!\r\r\n", &open_file);

	/* Close file */
	if (f_close(&open_file) != FR_OK) {
		Send_Uart(" Failed to close...\r\r\n");
		return false;
	} else
		Send_Uart("SD File closed successfully...\r\r\n");

	/* Open file to read */
	if (f_open(&open_file, f_name, FA_OPEN_ALWAYS | FA_READ) != FR_OK) {
		Send_Uart("OPENNING FILE FAILED TO OPEN...\r\r\n");
		return false;
	} else
		Send_Uart("OPENNING FILE successfully...\r\r\n");

	// read file
	char buffer[100];
	while (f_gets(buffer, 100, &open_file)) {
		Send_Uart(buffer);
	}

	/* Close file */
	if (f_close(&open_file) != FR_OK) {
		Send_Uart("Close file error..\r\r\n");
		return false;
	} else
		Send_Uart("Close FILE successfully...\r\r\n");

	return true;
}

FRESULT Create_Dir(char *name) {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();
		return INVALID;
	}

	fresult = f_mkdir(name);
	if (fresult == FR_OK) {
		char buf[100];
		sprintf(buf, "*%s* has been created successfully\r\n", name);
		Send_Uart(buf);
	} else {
		char buf[100];
		sprintf(buf, "ERROR No. %d in creating directory *%s*\r\n\n", fresult,
				name);
		Send_Uart(buf);
	}
	return fresult;
}

uint16_t File_Exists(char *name) {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();
		return INVALID;
	}

	fresult = FR_OK;
	char new_name[100];
	memset(new_name, 0, 100);

	uint32_t n_possible_files; // numero de ficheiros na árvore
	uint32_t file_number; // número do ficheiro para ser criado
	for (n_possible_files = 1; fresult != FR_NO_FILE;
			n_possible_files = n_possible_files * 2) // profundidade ++
					{
		sprintf(new_name, "%s_%ld.txt\r\n", name, n_possible_files);
		Send_Uart(new_name);

		fresult = f_stat(new_name, &fno);

		if (fresult != FR_OK && fresult != FR_NO_FILE) {
			// Error_Handler();
		}
	}

	// chegamos a uma arvore com n_possible_files de ficheiros possíveis. Procura binária encontra o ficheiro
	n_possible_files /= 2;
	file_number = n_possible_files;
	while (n_possible_files != 1) {
		sprintf(new_name, "%s_%ld.txt\r\n", name, file_number);
		Send_Uart(new_name);

		fresult = f_stat(new_name, &fno);

		if (fresult != FR_OK && fresult != FR_NO_FILE) {
			// Error_Handler();
		}

		// check if file exists
		if (fresult == FR_OK)
			file_number += n_possible_files / 2; // file exists
		else
			file_number -= n_possible_files / 2; // file doesn't exist
		n_possible_files = n_possible_files / 2;
	}
	// edge case - no caso do único ficheiro possível ser exponencial de 2
	sprintf(new_name, "%s_%ld.txt\r\n", name, file_number);
	Send_Uart(new_name);

	fresult = f_stat(new_name, &fno);

	if (fresult != FR_OK && fresult != FR_NO_FILE) {
		// Error_Handler();
	}

	// check if file exists
	if (fresult == FR_OK)
		return (file_number + 1); // no caso do ficheiro existir, incrementa o numero

	return file_number;

}

int Create_File(char *name) {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();
		return INVALID;
	}

	char buf[100];
	FIL fil;

	// open file
	fresult = f_open(&fil, name, FA_CREATE_ALWAYS);

	if (fresult != FR_OK) {
		sprintf(buf, "ERROR!!! No. %d in creating file *%s*\r\n\n", fresult,
				name);
		Send_Uart(buf);
		return INVALID;
	}

	sprintf(buf,
			"*%s* created successfully\r\n Now use Write_File to write data\r\n",
			name);
	Send_Uart(buf);

	// close file
	fresult = f_close(&fil);

	if (fresult != FR_OK) {
		sprintf(buf, "ERROR No. %d in closing file *%s*\r\n\n", fresult, name);
		Send_Uart(buf);
		return INVALID;
	}

	sprintf(buf, "File *%s* CLOSED successfully\r\n", name);
	Send_Uart(buf);

	return 0;
}

FRESULT Remove_File(char *name) {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();
		return INVALID;
	}

	char buf[100];

	/**** check whether the file exists or not ****/
	fresult = f_stat(name, &fno);
	if (fresult != FR_OK)
		sprintf(buf, "ERROR!!! *%s* does not exists\r\n\n", name);
	else {
		fresult = f_unlink(name);
		if (fresult == FR_OK)
			sprintf(buf, "*%s* has been removed successfully\r\n", name);
		else
			sprintf(buf, "ERROR No. %d in removing *%s*\r\n\n", fresult, name);
	}

	Send_Uart(buf);
	return fresult;
}

//* =============================>>>>>>>> FUNCTIONS USING CUSTOM OPEN FILE =====================================>>>>>>> */

int Open_File(char *name) {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();
		return INVALID;
	}

	// reserve an open file on the cof
	int index = reserve_cof();
	if (index == INVALID)
		return INVALID;
	add_to_cof(index, name);
	custom_open_file *cof = get_open_file(index);

	/* Create a file with read write access and open it */
	fresult = f_open(&cof->open_file, cof->name,
	FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
	if (fresult != FR_OK) {
		char buf[150];
		sprintf(buf, "ERROR!!! No. %d in opening file *%s*\r\n\n", fresult,
				cof->name);
		Send_Uart(buf);
		remove_from_cof(index);
		return INVALID;
	}

	// file opened successfully
	char buf[100];
	sprintf(buf, "File opened SUCCESSFULLY!\r\n\n");
	Send_Uart(buf);

	return index;
}

void Close_File(int index) {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();
		return;
	}

	custom_open_file *cof = get_open_file(index);

	/* Create a file with read write access and open it */
	fresult = f_close(&cof->open_file);
	if (fresult != FR_OK) {
		char buf[100];
		sprintf(buf, "ERROR!!! No. %d in closing file *%s*\r\n\n", fresult,
				cof->name);
		Send_Uart(buf);
	}

	remove_from_cof(index);
}

void Write_File(int index, void *data, ssize_t offset, size_t len) {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();
		return;
	}

	custom_open_file *cof = get_open_file(index);
	size_t bytes_written = 0, total_bytes = 0;

	// move o cursor do ponteiro para
	if (offset == INVALID)
		f_lseek(&cof->open_file, f_size(&cof->open_file));
	else
		f_lseek(&cof->open_file, offset);

	while (total_bytes < len) {
		// write data
		fresult = f_write(&cof->open_file, data + total_bytes,
				len - total_bytes, &bytes_written);
		if (fresult != FR_OK) {
			fresult = f_close(&cof->open_file);
			if (fresult != FR_OK) {
				char buf[100];
				sprintf(buf,
						"ERROR!!! No. %d while writing to the FILE *%s*\r\n\n",
						fresult, cof->name);
				Send_Uart(buf);

				// Error_Handler();
			}

			fresult = f_open(&cof->open_file, cof->name,
			FA_OPEN_EXISTING | FA_WRITE);
			if (fresult != FR_OK) {
				char buf[100];
				sprintf(buf,
						"ERROR!!! No. %d while writing to the FILE *%s*\r\n\n",
						fresult, cof->name);
				Send_Uart(buf);

				// Error_Handler();
			}
		}

		// calculate bytes left
		total_bytes += bytes_written;
	}
}

uint16_t msg_counter = 0;
uint16_t executionTime;
uint32_t data_size = 0;
void Update_File(int index, void *data, size_t len) {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();
		return;
	}

	custom_open_file *cof = get_open_file(index);

	executionTime = time_in_millis - cof->time_opened_ms;
	if (executionTime > TIME_THRESHOLD)	//Entenda-se "maior que TIME_THRESHOLD ms"
	{
		// save data to sd card
		f_sync(&cof->open_file);

		// debugging
#if (DEBUG_SD_CARD)
		printf("sd card data: %ld MB, %ld KB\r\n", data_size/1000000, data_size/1000);

		printf("sd card freq: %d Hz\r\n", msg_counter);

//		printf("execution time = %d s\r\n", executionTime);

		printf("sd card datarate: %f KB/s\r\n\n", (float)msg_counter*(float)len/(float)executionTime);
		#endif

		// reset counters
		msg_counter = 0;
		cof->time_opened_ms = time_in_millis;
	}

	// write data to the file and refresh variables
	Write_File(index, data, INVALID, len);
	msg_counter++;
	data_size += len;
}

void Read_File(int index, void *buffer, int offset, size_t length) {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();
		return;
	}

	custom_open_file *cof = get_open_file(index);
	size_t n_bytes_received = 0, n_bytes_read = 0;

	// move o cursor do ponteiro para
	if (offset == INVALID)
		f_lseek(&cof->open_file, f_size(&cof->open_file));
	else
		f_lseek(&cof->open_file, offset);

	/* Read data from the file
	 * see the function details for the arguments */

	while (n_bytes_received < length) {
		fresult = f_read(&cof->open_file, buffer + n_bytes_received,
				length - n_bytes_received, &n_bytes_read);
		if (fresult != FR_OK) {
			char buf[100];
			sprintf(buf, "ERROR!!! No. %d in reading file *%s*\r\n\n", fresult,
					cof->name);
			Send_Uart(buf);
			// Error_Handler();
			return;
		}

		// calculate bytes left
		n_bytes_received += n_bytes_read;
	}
}


//* =============================>>>>>>>> FUNCTIONS CUSTOM OPEN FILE =====================================>>>>>>> */

void cof_table_init() {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();
		return;
	}

	for (int i = 0; i < MAX_OPEN_FILES; i++) {
		memset(open_file_table[i].name, 0, FILENAME_MAX_LENGTH);
		open_file_table[i].state = FREE;
		open_file_table[i].time_opened_ms = 0;
	}
}

int reserve_cof() {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();
		return INVALID;
	}

	for (int i = 0; i < MAX_OPEN_FILES; i++) {
		if (open_file_table[i].state == FREE) {
			open_file_table[i].state = TAKEN;
			return i;
		}
	}
	return INVALID;
}

void add_to_cof(int index, char *name) {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();
		return;
	}

	strncpy(open_file_table[index].name, name, strlen(name));
	open_file_table[index].time_opened_ms = time_in_millis;
}

void remove_from_cof(int index) {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();
		return;
	}

	open_file_table[index].state = FREE;
	memset(open_file_table[index].name, '\0',
			strlen(open_file_table[index].name));
}

custom_open_file* get_open_file(int index) {
	if (sd_not_mounted) {
		return_sd_card_not_mounted();
		return NULL;
	}

	return &open_file_table[index];
}

