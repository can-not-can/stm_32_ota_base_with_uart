#ifndef _FLAG_CONTENT_H_
#define _FLAG_CONTENT_H_

/* Private includes ----------------------------------------------------------*/
#include <stdint.h>
#include "stm32f1xx_hal.h"
/* Private define ------------------------------------------------------------*/
#define Bootloader_Address_Start	0x08000000ul
#define Bootloader_Address_Size		0x6000ul

#define App_Address_Size		App_Address_Size_1
#define Magic_Code_Start		0xAA55ul
#define Magic_Code_Stop			0x55AAul

#define App_Address_Start_1		0x08006000ul
#define App_Address_Size_1		0x39000ul

#define App_Address_Start_2		0x0803F000ul
#define App_Address_Size_2		0x39000ul

#define Flag_Address_Start		0x08078000ul
#define Flag_Address_Size			0x1000ul

#define Flag_BootFlagTypeDef_Start	0x08079000ul
/* Private value ------------------------------------------------------------*/
typedef struct
{
    uint16_t magic_code;         // 魔数，固定0xAA55，用于判断当前标记页数据是否有效
    uint8_t  update_flag;        // 升级标记：标记哪个分区存放了待启动的新版本固件
    uint8_t  try_cnt;            // 升级尝试次数，新APP启动失败时递减；计数为0则放弃升级，回滚旧版本
    uint32_t active_app_addr;    // 当前稳定正常运行的APP分区起始地址，升级失败回退到此分区
    uint32_t new_fw_crc;         // 待升级（新版本）固件的完整CRC32校验值，Boot校验固件完整性
    uint32_t new_fw_len;         // 待升级（新版本）固件有效字节长度，用于CRC计算
    uint32_t fw_version;         // 待升级（新版本）固件版本号，可用于版本校验、禁止降级
		uint8_t head_try_cnt;
    uint8_t  reserved[11];       // 保留字节，4字节对齐，预留后续扩展字段
} BootFlagTypeDef;

extern BootFlagTypeDef* Flag_Type_Base;


/* Private function prototypes -----------------------------------------------*/


#endif
