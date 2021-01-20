/***************************************************************************** 
  Copyright (c) 2020-2020, XinShiYun Technology Corporation
  All rights reserved.
 
  File Name     :     mcu_update.c
  Programmer    :     Fu Guangbin
  Date          :     2021.1
  The Project   :     XTS100
  Description   :	  used for mcu update
******************************************************************************/

#include "mcu_update.h"


unsigned char bin_file_buf[MAX_BIN_LEN] = {0};


static int Update_Ser_Open(void)
{
	int ret;
	struct termios options;

	mcu_update_serial_handle = open(MCU_UPDATE_UART, O_RDWR);
	if (mcu_update_serial_handle == INVALID_HANDLE_VALUE) {
		printf("Open file err:%s errno:%d\n", MCU_UPDATE_UART, errno);
		return -1;
	}

	tcgetattr(mcu_update_serial_handle, &options);

	memset(&options,0,sizeof(struct termios));
	options.c_lflag = 0;
	options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
	options.c_iflag &= ~(IXON | IXOFF | IXANY);
	options.c_oflag &= ~(ONLCR | OCRNL);
	options.c_cc[VTIME] = 0;
	options.c_cc[VMIN] = 0;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //rockchip add
	
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_cflag |= (CLOCAL | CREAD); //rockchip modify

	options.c_cflag |= PARENB;
	options.c_cflag &= ~PARODD;//even
	options.c_iflag &= ~INPCK;
	options.c_cflag &= ~CSTOPB;

	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);
	
	ret = tcsetattr(mcu_update_serial_handle, TCSANOW, &options);
	if (ret == -1) {
		printf("SetArrt tcsetattr err: %d\n", errno);
		return -1;
	}

	return 0;
}


static void Update_Ser_Close(void)
{
	if (mcu_update_serial_handle != INVALID_HANDLE_VALUE) {
		close(mcu_update_serial_handle);
	}
	mcu_update_serial_handle = INVALID_HANDLE_VALUE;
}


static void Update_Ser_Clear(void)
{
	printf("Update_Ser_Clear: clear the data of uart ...\n");
	tcflush(mcu_update_serial_handle, TCIOFLUSH);
	printf("Update_Ser_Clear: clear end ...\n");
}


static int Update_Ser_Write(unsigned char *pszBuf, int SendCnt, unsigned long TimeOutMS)
{
	int ret = 0;
	struct timeval CurTime;
	fd_set wwrite;

	FD_ZERO(&wwrite);
	FD_SET(mcu_update_serial_handle, &wwrite);

	CurTime.tv_sec = (TimeOutMS / 1000);
	CurTime.tv_usec = (TimeOutMS % 1000) * 1000;

	ret = select(mcu_update_serial_handle + 1, NULL, &wwrite, NULL, &CurTime);
	if (ret == 0) {
		printf("Update_Ser_Write select timeout\n");
		return -1;
	} else if (ret < 0) {
		printf("Update_Ser_Write select error: %d\n", errno);
		return -1;
	}

#ifdef DEBUG_PRINT
	for(int i =0;i<SendCnt;i++)
	{
		printf("Write_Buf[%d]:%2x\n",i,pszBuf[i]);
	}
#endif

	ret = write(mcu_update_serial_handle, pszBuf, SendCnt);
	if (ret <= 0) {
		Update_Ser_Clear();
		printf("Update_Ser_Write write buffer error:%d\n", errno);
		return -1;
	}
	return 0;
}


static int Update_Ser_Read(unsigned char *pszBuf, int ReadCnt,unsigned long TimeOutMS)
{
	int ret = 0;
	int index = 0;
	struct timeval CurTime;
	fd_set wread;

	FD_ZERO(&wread);
	FD_SET(mcu_update_serial_handle, &wread);

	CurTime.tv_sec = (TimeOutMS / 1000);
	CurTime.tv_usec = (TimeOutMS % 1000) * 1000;

	ret = select(mcu_update_serial_handle + 1, &wread, NULL, NULL, &CurTime);
	if (ret == 0) {
		printf("Update_Ser_Read select timeout\n");
		return -1;
	} else if (ret < 0) {
		printf("Update_Ser_Read select error: %d\n", errno);
		return -1;
	}

	ret = read(mcu_update_serial_handle, pszBuf, ReadCnt);
	if (ret <= 0) {
		printf("Update_Ser_Read read buffer error:%d\n",errno);
		return -1;
	}

#ifdef DEBUG_PRINT
	for (int i=0;i<ret;i++){
		printf("Read_Buf[%d]:%2x\n",i,pszBuf[i]);
	}
#endif

	return ret;
}


static int Send_Command(int step)
{
	int ret = -1;
	int length = 0;
	unsigned char buffer[MAX_BUF_LEN] = {0};

	switch(step)
	{
		case STM32_BURN_INIT:
			buffer[0] = 0x7F;
			length = 1;
			break;

		case CHECK_ID_VERSION:
			buffer[0] = 0x02;
			buffer[1] = 0xFD;
			length = 2;
			break;

		case START_ERASE:
			buffer[0] = 0x44;
			buffer[1] = 0xBB;
			length = 2;
			break;

		case ERASE_ALL_CHIP:
			buffer[0] = 0xFF;
			buffer[1] = 0xFF;
			buffer[2] = 0x00;
			length = 3;
			break;

		case START_WRITE:
			buffer[0] = 0x31;
			buffer[1] = 0xCE;
			length = 2;
			break;

		case SET_WRITE_BYTE_NUM:
			buffer[0] = 0xFF;
			length = 1;
			break;

		case SET_READ_PROTECT:
			buffer[0] = 0x82;
			buffer[1] = 0x7D;
			length = 2;
			break;
		
		case REMOVE_READ_PROTECT:
			buffer[0] = 0x92;
			buffer[1] = 0x6D;
			length = 2;
			break;

		case RUN_GO_MCU:
			buffer[0] = 0x21;
			buffer[1] = 0xDE;
			length = 2;
			break;

		default:
			printf("wrong step num!\n");
	}

	ret = Update_Ser_Write(buffer,length,W_TIMEOUTMS*length);
	if(ret < 0){
		printf("Send_Command: Update_Ser_Write error!\n");
	}
	return ret;
}


static int Check_Respond(int num)
{
	int r_bytes = -1;
	int i = 0;
	int r_num = 0;
	unsigned char pReponse[MAX_BUF_LEN] = {0};
	//读返回值
	r_bytes = Update_Ser_Read(pReponse,MAX_BUF_LEN,R_TIMEOUTMS);

	if(r_bytes <= 0){
		printf("Check_Respond: Update_Ser_Read error!\n");
		return -1;
	}
	else{
		for(i=0;i<r_bytes;i++)
		{
			if(RIGHT_WORD == pReponse[i])
			{
				r_num ++;
			}
			if(r_num == num)
			{
				break;
			}
		}
		if(r_num < num)
		{
			printf("Check_Respond check error!\n");
			return -1;
		}
	}
	return 0;
}



static int Get_bin_size()
{
	int size = -1;

	if(0 == access(MCU_BIN_PATH,F_OK)){
		printf("Mcu bin file exist\n");
	}else{
		printf("[Get_bin_size] error! Mcu bin file not exist!\n");
		return -1;
	}

	FILE *fp = fopen(MCU_BIN_PATH,"r");
	if(NULL == fp)
	{
		printf("Write_bin: open bin file error!\n");
		return -1;
	}
	fseek(fp,0L,SEEK_END);
	size = ftell(fp);
	fclose(fp);
	printf("Bin file size = %d\n",size);

	return size;
}

static int Get_crc(unsigned char* des_buf,int len)
{
	int i = 0;
	int crc = 0;
	for(i=0;i<len;i++)
	{
		crc = crc ^ des_buf[i];
	}
#ifdef DEBUG_PRINT
	printf("Get_crc: crc:0x%2x\n",crc);
#endif
	return crc;
}


static int Write_bin()
{
	int bin_file = -1;
	int size = -1;
	int i = 0;
	int j = 0;
	int num = 0;
	int ret = -1;
	int addr = 0;
	int length = 0;
	unsigned char addr_buffer[MAX_BUF_LEN] = {0};
	unsigned char bin_buffer[WRITE_BIN_BUF_LEN+2] = {0};

	//获取bin文件大小
	size = Get_bin_size();
	if(size<=0){
		printf("Write_bin:Get_bin_size error!\n");
		return -1;
	}

	//读取bin文件
	bin_file = open(MCU_BIN_PATH,O_RDONLY);
	if(-1 == bin_file){
		printf("open bin file error! errno:%d\n",errno);
		return -1;
	}else{
		printf("open bin file success\n");
	}
	read(bin_file,bin_file_buf,size);

	//依次发送烧录指令+地址+数据+校验
	num = (size/256)+1;

	for(i=0;i<num;i++)
	{
		//清空buffer
		memset(addr_buffer,0,MAX_BUF_LEN);
		memset(bin_buffer,0,WRITE_BIN_BUF_LEN+2);

		//发送烧录指令
		ret = Send_Command(START_WRITE);
		if(ret < 0)
		{
			printf("Send_Command: Start write error!\n");
			return -1;
		}
		//等待回复0x79
		ret = Check_Respond(CHECK_ONCE);
		if(ret < 0){
			printf("Check_Respond: Start write error!\n");
			return -1;
		}
		usleep(50000);//50ms

		//发送烧录地址+校验位
		addr = BASE_WRITE_ADDR + 256*i;
		addr_buffer[0] = (addr >> 24) & 0xff;
		addr_buffer[1] = (addr >> 16) & 0xff;
		addr_buffer[2] = (addr >> 8) & 0xff;
		addr_buffer[3] = addr & 0xff;
		addr_buffer[4] = Get_crc(addr_buffer,4);
		length = 5;

		ret = Update_Ser_Write(addr_buffer,length,W_TIMEOUTMS*length);
		if(ret < 0){
			printf("Write_bin: Write addr error!,num=%d\n",num);
		}

		//等待回复0x79
		ret = Check_Respond(CHECK_ONCE);
		if(ret < 0){
			printf("Check_Respond: Write addr error!\n");
			return -1;
		}
		usleep(50000);//50ms

		//发送字节数N + (N+1)数据字节 + 校验位，N=255
		bin_buffer[0] = 0xFF;
		for(j=0;j<WRITE_BIN_BUF_LEN;j++)
		{
			bin_buffer[j+1] = bin_file_buf[(i*WRITE_BIN_BUF_LEN)+j];
		}
		bin_buffer[WRITE_BIN_BUF_LEN+1] = Get_crc(bin_buffer,WRITE_BIN_BUF_LEN+1);
		length = WRITE_BIN_BUF_LEN + 2;

		ret = Update_Ser_Write(bin_buffer,length,W_TIMEOUTMS*length);
		if(ret < 0){
			printf("Write_bin: Write bin_buffer error!,num=%d\n",num);
			return -1;
		}

		if(0 == (i+1)%20){
			printf("Total write %d KB...\n",(i+1)*256/1024);
		}

		usleep(50000);//50ms
	}
	return 0;
}


static int Run_mcu()
{
	int ret = -1;
	int length = 0;
	unsigned char buffer[MAX_BUF_LEN] = {0};
	ret = Send_Command(RUN_GO_MCU);
	if(ret < 0){
		printf("Send_Command: Run go mcu error!\n");
		return -1;
	}

	ret = Check_Respond(CHECK_TWICE);
	if(ret < 0){
		printf("Check_Respond: Run go mcu cmd error!\n");
		return -1;
	}

	usleep(100000);//100ms

	buffer[0] = (BASE_WRITE_ADDR >> 24) & 0xff;
	buffer[1] = (BASE_WRITE_ADDR >> 16) & 0xff;
	buffer[2] = (BASE_WRITE_ADDR >> 8) & 0xff;
	buffer[3] = BASE_WRITE_ADDR & 0xff;
	buffer[4] = Get_crc(buffer,4);
	length = 5;

	ret = Update_Ser_Write(buffer,length,W_TIMEOUTMS*length);
	if(ret < 0){
		printf("Run_mcu: Write go addr error!\n");
		return -1;
	}

	ret = Check_Respond(CHECK_ONCE);
	if(ret < 0){
		printf("Check_Respond: Run go mcu addr error!\n");
		return -1;
	}

	return 0;
}


static int Update_mcu(void)
{
	int ret = -1;
	int retry = 0;

	for(retry=0;retry<RETRY_TIMES;retry++)
	{
		//Step2:打开串口
		printf("Step 2: Open Update Serial\n");
		ret = Update_Ser_Open();
		if(ret < 0){
			printf("Serial Open error!\n");
			goto err_handle;
		}
		printf("------------------------ pass\n\n");


		//Step3:控制gpio使mcu进入烧录模式
		printf("Step 3: Enter burn mode\n");
		Burn_mode_enter();
		printf("------------------------ pass\n\n");


		//Step4:初始化，测试波特率
		printf("Step 4: Stm32 burn init\n");
		ret = Send_Command(STM32_BURN_INIT);
		if(ret < 0){
			printf("Send_Command: Stm32 burn init error!\n");
			goto err_handle;
		}

		ret = Check_Respond(CHECK_ONCE);
		if(ret < 0){
			printf("Check_Respond: Stm32 burn init error!\n");
			goto err_handle;
		}
		printf("------------------------ pass\n\n");
		usleep(100000);//100ms


		//Step5:测试获取id版本号
		printf("Step 5: Check id version\n");
		ret = Send_Command(CHECK_ID_VERSION);
		if(ret < 0){
			printf("Send_Command: Check id version error!\n");
			goto err_handle;
		}

		ret = Check_Respond(CHECK_TWICE);
		if(ret < 0){
			printf("Check_Respond: Check id version error!\n");
			goto err_handle;
		}
		printf("------------------------ pass\n\n");
		usleep(100000);//100ms


		//Step6:开始擦除
		printf("Step 6: Start erase\n");
		ret = Send_Command(START_ERASE);
		if(ret < 0){
			printf("Send_Command: Start erase error!\n");
			goto err_handle;
		}

		ret = Check_Respond(CHECK_ONCE);
		if(ret < 0){
			printf("Check_Respond: Start erase error!\n");
			goto err_handle;
		}
		printf("------------------------ pass\n\n");
		usleep(100000);//100ms


		//Step7:全部擦除
		printf("Step 7: Erase all chip\n");
		ret = Send_Command(ERASE_ALL_CHIP);
		if(ret < 0){
			printf("Send_Command: Erase all chip error!\n");
			goto err_handle;
		}

		ret = Check_Respond(CHECK_ONCE);
		if(ret < 0){
			printf("Check_Respond: Erase all chip error!\n");
			goto err_handle;
		}
		printf("------------------------ pass\n\n");
		usleep(100000);//100ms

	
		//Step8:进行烧录
		printf("Step 8: Write bin to stm32\n");
		ret = Write_bin();
		if(ret < 0){
			printf("Write_bin error!\n");
			goto err_handle;
		}
		printf("------------------------ pass\n\n");
		usleep(100000);//100ms


		//Step9:控制gpio使mcu进入运行模式
		printf("Step 9: Enter run mode\n");
		Run_mode_enter();
		printf("------------------------ pass\n\n");


		//Step10:关闭串口
		printf("Step 10: Close Update Serial\n");
		Update_Ser_Close();
		printf("------------------------ pass\n\n");
		printf("-----------------------------\n");
		printf("Stm32 update success!!!\n");
		printf("-----------------------------\n\n");
		break;


	err_handle:
		//关闭串口
		printf("Close Update Serial\n");
		Update_Ser_Close();
		printf("Stm32 update failed! retry!\n\n");
	}
	return 0;
}


int main(void)
{
	int ret = -1;
	printf("#############################\n");
	printf("Step 1: Compare mcu version\n");
	ret = Compare_ver();
	if(NO_NEED_UPDATE == ret)
	{
		printf("No update required!\n");
		printf("------------------------ pass\n\n");
		printf("#############################\n");
		return 0;
	}
	else
	{
		printf("Update mcu next!\n");
		printf("------------------------ pass\n\n");
		Update_mcu();
		printf("#############################\n");
	}
	return 0;
}

