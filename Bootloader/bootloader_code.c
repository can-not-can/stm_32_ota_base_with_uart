#include "bootloader_code.h"
/*
@
@校验函数
@
*/
static uint32_t CRC_CalcBuffer(uint8_t *buf, uint32_t len);
/*
@
@往flash更新结构体函数
@
*/
static HAL_StatusTypeDef HAL_Erase_Pages(uint32_t Address);
static HAL_StatusTypeDef Bootloader_Write_Flash(uint32_t Address,uint16_t* Data,uint32_t Len);
static HAL_StatusTypeDef Bootloader_WriteBootFlagTypeDef_Flash(BootFlagTypeDef* flag);
/**
@擦除整个APP分区（STM32F1，页大小2KB = 0x800）
@app_addr APP分区起始地址
@1成功，0失败
 */
static uint8_t Flash_Erase_AppPartition(uint32_t app_addr);
/*
@
@向上位机发送1字节握手/诊断信号
@  0x5A=请求发包头  0xA5=擦除完成请求发固件  0x55=升级成功即将跳转
@  0xE0=包头重试5次放弃 0xE1=包头接收超时 0xE2=包头CRC错 0xE3=magic错
@  0xE4=长度非法 0xE5=固件片接收超时 0xE6=写Flash失败 0xE7=固件CRC错 0xE8=擦App失败
*/
static void Boot_SendSignal(uint8_t sig);
/*
@
@Bootloader_Ota_Update更新函数，需要更新时进入该函数，对app1或app2的flash区域进行更新
@									更新结构体Flag_Type_Base数据
@返回0 则OTA升级失败	返回1则OTA升级成功
*/
static uint8_t Bootloader_Ota_Update(BootFlagTypeDef* Flag_Type_Base);
/*
@
@JumpToApplication简单的函数跳转
@									App_Address_Start_x是跳转地方的首地址
*/
static void	JumpToApplication(uint32_t App_Address_Start_x);
/*
@
@Bootloader_Check_Update_Is_Required在bootloader阶段检查是否需要更新
@									
*/
void Bootloader_Check_Update_Is_Required(void);

/*
@
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@函数具体实现如下
@
*/

/*
@
@CRC校验函数
@buf首地址，len长度
*/
static uint32_t CRC_CalcBuffer(uint8_t *buf, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    uint32_t i,j;
    for(i=0;i<len;i++)
    {
        crc ^= buf[i];
        for(j=0;j<8;j++)
        {
            if(crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>=1;
        }
    }
    return ~crc;
}

/*
@
@发送1字节握手/诊断信号给上位机
@
*/
static void Boot_SendSignal(uint8_t sig)
{
    uint8_t frame[3] = {sig, '\r', '\n'}; //信号字节后加\r\n换行，便于上位机按行接收/串口助手观察
    HAL_UART_Transmit(&huart1, frame, 3, 100);
}

/*
@
@往flash更新结构体函数
@
*/
static HAL_StatusTypeDef HAL_Erase_Pages(uint32_t Address){
	HAL_StatusTypeDef status = HAL_OK;
	HAL_FLASH_Unlock();
	
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError = 0;
	
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = Address;
	EraseInitStruct.NbPages = 1;
	
	if(HAL_FLASHEx_Erase(&EraseInitStruct,&PageError) != HAL_OK){
		//@@@擦除错误操作
		HAL_FLASH_Lock();
		for(;;);
	}
	
	HAL_FLASH_Lock();
	return status;
}

static HAL_StatusTypeDef Bootloader_Write_Flash(uint32_t Address,uint16_t* Data,uint32_t Len){
	HAL_StatusTypeDef status = HAL_OK;
	
	HAL_FLASH_Unlock();
	for(uint32_t i=0;i<Len;i++){
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,Address+i*2,Data[i]);
		if(status != HAL_OK){
			goto END_LOCK;
		}
	}
END_LOCK:
	HAL_FLASH_Lock();
	return status;
}

static HAL_StatusTypeDef Bootloader_WriteBootFlagTypeDef_Flash(BootFlagTypeDef* flag){
	uint16_t write_data[16] = {0};
	
	memcpy(write_data, flag, sizeof(BootFlagTypeDef));
	HAL_Erase_Pages(Flag_BootFlagTypeDef_Start);
	
	return Bootloader_Write_Flash(Flag_BootFlagTypeDef_Start,write_data,16);
}
/*
@
@上位机发magic_code_start(2)+crc(4)+len(4)+version(4)+该调数据的校验位(4)+magic_code_stop(2)
@
*/
void Bootloader_Waiting_For_Receive_Update_Head(void)
{
	BootFlagTypeDef flagRam = *Flag_Type_Base;
	uint8_t Receive_Content[24] = {0};
	uint8_t head_try_cnt = 0;  //临时计数，放RAM，不上Flash！本次上电有效
	
	while(1) //持续请求包头，直到上位机响应并解析成功（不再限制次数，避免错过上位机启动时机）
	{
		if(head_try_cnt >= 10)break;
		
		memset(Receive_Content,0,sizeof(Receive_Content));
		
		//主动请求上位机发送包头；上位机收到0x5A后立即回发20字节包头
		Boot_SendSignal(0x5A);
		
		HAL_StatusTypeDef uart_ret;
		uart_ret = HAL_UART_Receive(&huart1,Receive_Content,20,1000);
		if(uart_ret != HAL_OK)
		{
			Boot_SendSignal(0xE1); //0xE1:包头接收超时/不完整
			head_try_cnt++;
			continue;
		}

		uint16_t rec_magic_start = ((uint16_t)Receive_Content[0] | ((uint16_t)Receive_Content[1] << 8));
		uint32_t rec_crc = ((uint32_t)Receive_Content[2] | ((uint32_t)Receive_Content[3]<<8)|((uint32_t)Receive_Content[4]<<16)|((uint32_t)Receive_Content[5]<<24));
		uint32_t rec_len = ((uint32_t)Receive_Content[6]|((uint32_t)Receive_Content[7]<<8)|((uint32_t)Receive_Content[8]<<16)|((uint32_t)Receive_Content[9]<<24));
		uint32_t rec_version = ((uint32_t)Receive_Content[10]|((uint32_t)Receive_Content[11]<<8)|((uint32_t)Receive_Content[12]<<16)|((uint32_t)Receive_Content[13]<<24));
		uint32_t rec_crc_Head = ((uint32_t)Receive_Content[14]|((uint32_t)Receive_Content[15]<<8)|((uint32_t)Receive_Content[16]<<16)|((uint32_t)Receive_Content[17]<<24));
		uint16_t rec_magic_stop = ((uint16_t)Receive_Content[18]|((uint16_t)Receive_Content[19]<<8));

		uint32_t Bootloader_Calc_Crc = CRC_CalcBuffer(&Receive_Content[2], 12);
		if(Bootloader_Calc_Crc != rec_crc_Head)
		{
			Boot_SendSignal(0xE2); //0xE2:包头CRC错
			head_try_cnt++;
			continue;
		}

		if(rec_magic_start != Magic_Code_Start || rec_magic_stop != Magic_Code_Stop)
		{
			Boot_SendSignal(0xE3); //0xE3:magic不匹配
			head_try_cnt++;
			continue;
		}
		
		if(rec_len == 0 || rec_len > (uint32_t)App_Address_Size)
		{
			Boot_SendSignal(0xE4); //0xE4:固件长度非法
			head_try_cnt++;
			continue;
		}

		//包头合法，写入升级标记到Flash
		flagRam.magic_code = Magic_Code_Start; //首次上电Flash为0xFFFF，必须写入有效魔数，否则后续不会进入升级
		flagRam.update_flag = 1;
		flagRam.new_fw_crc = rec_crc;
		flagRam.new_fw_len = rec_len;
		flagRam.fw_version = rec_version;
		Bootloader_WriteBootFlagTypeDef_Flash(&flagRam);
		return;
	}
}




static uint8_t Bootloader_Ota_Update(BootFlagTypeDef* Flag_Type_Base){
	//@具体是实现，OTA升级操作，并校验正确性操作无误后，擦除另一个app区域的数据留着给下次更新操作
	uint32_t Download_Address = 0;
	if(Flag_Type_Base->active_app_addr == (uint32_t)App_Address_Start_1){
		Download_Address = App_Address_Start_2;
	}else{
		Download_Address = App_Address_Start_1;
	}
	
	if(Flag_Type_Base->new_fw_len == 0 || Flag_Type_Base->new_fw_len >(uint32_t)App_Address_Size){
		Boot_SendSignal(0xE4); //0xE4:固件长度非法
		return 0;
	}
	
	if(!Flash_Erase_AppPartition(Download_Address)){
		Boot_SendSignal(0xE8); //0xE8:擦除App分区失败
		return 0;
	}
	
	Boot_SendSignal(0xA5); //擦除完成，请求上位机发送固件
	/*排空握手期间在途字节：清溢出标志后逐字节丢弃，直到连续50ms总线静默，
	  保证固件接收从第1个真实字节开始。与上位机收到0xA5后延时100ms再发相配合*/
	__HAL_UART_CLEAR_OREFLAG(&huart1);
	uint8_t drop_byte;
	while(HAL_UART_Receive(&huart1, &drop_byte, 1, 50) == HAL_OK){}

	uint8_t fwBuf[128];
	uint16_t pData[64];
	uint32_t write_address = Download_Address;
	uint32_t remainBytes = Flag_Type_Base->new_fw_len;
	
	while(remainBytes > 0){
		uint16_t readLen = (remainBytes > sizeof(fwBuf)) ? sizeof(fwBuf) : (uint16_t)remainBytes;
		/* 阻塞接收一整片：收满readLen字节才返回HAL_OK；超时/出错直接判升级失败
		   115200下128字节约11ms，1000ms超时既覆盖正常片间间隔，也能识别掉线 */
		if(HAL_UART_Receive(&huart1, fwBuf, readLen, 1000) != HAL_OK){
			Boot_SendSignal(0xE5); //0xE5:固件分片接收超时
			return 0;
		}
		uint32_t rec_Len = readLen; //上位机发送长度必为偶数(奇数已补0xFF)
		for(uint32_t i=0;i<rec_Len/2;i++){
			pData[i] = (uint16_t)fwBuf[2*i+1]<<8|(uint16_t)fwBuf[2*i];
		}
		
		if(Bootloader_Write_Flash(write_address,pData,rec_Len/2)==HAL_OK)	write_address+=rec_Len;
		else{
			Boot_SendSignal(0xE6); //0xE6:写Flash失败
			return 0;
		}
		
		remainBytes -= rec_Len;
	}
	
	uint32_t calcFwCrc = CRC_CalcBuffer((uint8_t*)Download_Address, Flag_Type_Base->new_fw_len);
	if(calcFwCrc != Flag_Type_Base->new_fw_crc)
	{
		Boot_SendSignal(0xE7); //0xE7:固件整体CRC校验失败
		return 0;
	}else{
		BootFlagTypeDef flagRam = *Flag_Type_Base;
		
		flagRam.update_flag = 0;
		flagRam.try_cnt = 0;
		flagRam.active_app_addr = ((Flag_Type_Base->active_app_addr== App_Address_Start_1)? App_Address_Start_2: App_Address_Start_1);
		
		//把RAM里的结构体写入Flash永久保存
		Bootloader_WriteBootFlagTypeDef_Flash(&flagRam);
	}
	
	return 1;
}


static uint8_t Flash_Erase_AppPartition(uint32_t app_addr)
{
    uint32_t page_num;
    uint32_t erase_status;
    FLASH_EraseInitTypeDef EraseInitStruct;
    const uint16_t *p_page;
    uint8_t page_need_erase;

    HAL_FLASH_Unlock();
   
    page_num = App_Address_Size / 0x800ul;
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.NbPages     = 1;

    for(uint32_t i = 0; i < page_num; i++)
    {
        uint32_t page_addr = app_addr + i * 0x800;
        p_page = (const uint16_t *)page_addr;
        page_need_erase = 0;

        // 遍历本页，检查是否存在非0xFFFF数据（F1 Flash半字是0xFFFF代表空）
        // 每次检查半字，一页0x800 = 1024个半字
        for(uint32_t j = 0; j < 0x800 / 2; j++)
        {
            if(p_page[j] != 0xFFFF)
            {
                page_need_erase = 1;
                break;
            }
        }

        if(page_need_erase == 0)
        {
            continue;
        }

        EraseInitStruct.PageAddress = page_addr;
        if(HAL_FLASHEx_Erase(&EraseInitStruct, &erase_status) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return 0;
        }
        if(erase_status != 0xFFFFFFFF)
        {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
    }
    HAL_FLASH_Lock();
    return 1;
}



static void	JumpToApplication(uint32_t App_Address_Start_x){
	if(((*(volatile uint32_t*)App_Address_Start_x) & 0x2FFE0000) == 0x20000000){
		SCB->VTOR	= App_Address_Start_x;
		
		FUNC	App_Reset_Handler = (FUNC)(*(volatile uint32_t*)(App_Address_Start_x + 4));
		__set_MSP(*(volatile uint32_t*)App_Address_Start_x);
		
		App_Reset_Handler();
	}
}

void Bootloader_Check_Update_Is_Required(void){
	Bootloader_Waiting_For_Receive_Update_Head();
	
	BootFlagTypeDef Flag = *Flag_Type_Base;
	if(Flag.magic_code == 0xAA55){
		if(Flag.update_flag == 1){
			if(Bootloader_Ota_Update(&Flag)){
				Boot_SendSignal(0x55); //升级成功，通知上位机后跳转新App
				HAL_Delay(50);         //等0x55发完再跳转，避免App立刻改串口配置
				JumpToApplication((uint32_t)(Flag_Type_Base->active_app_addr));
				//@@@@@@@@@@@需要额外操作
			}else{
				//OTA升级失败(CRC错/接收超时)：软复位重新进入Boot，可再次等待上位机升级
				//后续配合try_cnt递减，计数耗尽后清update_flag回滚旧App
				NVIC_SystemReset();
			}
		}else{
			JumpToApplication((uint32_t)(Flag_Type_Base->active_app_addr));
		}
	}

}

