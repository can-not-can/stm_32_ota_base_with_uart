#ifndef _BOOTLOADER_CODE_H_
#define _BOOTLOADER_CODE_H_
/* Private includes ----------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "flag_content.h"
#include "usart.h"
/* Private typedef -----------------------------------------------------------*/
typedef	void(*FUNC)(void);

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/

void Bootloader_Waiting_For_Receive_Update_Head(void);
void Bootloader_Check_Update_Is_Required(void);



#endif
