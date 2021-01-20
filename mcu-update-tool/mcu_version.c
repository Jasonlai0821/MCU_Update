/***************************************************************************** 
  Copyright (c) 2020-2020, XinShiYun Technology Corporation
  All rights reserved.
 
  File Name     :     mcu_version.c
  Programmer    :     Fu Guangbin
  Date          :     2021.1
  The Project   :     XTS100
  Description   :	  used for mcu update
******************************************************************************/

#include "mcu_update.h"

static int Service_Ser_Open(void)
{
	int ret;
	struct termios options;

	mcu_service_serial_handle = open(MCU_SERVICE_UART, O_RDWR);
	if (mcu_service_serial_handle == INVALID_HANDLE_VALUE) {
		printf("Open file err:%s errno:%d\n", MCU_SERVICE_UART, errno);
		return -1;
	}

	tcgetattr(mcu_service_serial_handle, &options);

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

	options.c_cflag &= ~PARENB;//no odd/even
	options.c_iflag &= ~INPCK;
	options.c_cflag &= ~CSTOPB;

	cfsetispeed(&options, B460800);
	cfsetospeed(&options, B460800);

	ret = tcsetattr(mcu_service_serial_handle, TCSANOW, &options);
	if (ret == -1) {
		printf("SetArrt tcsetattr err: %d\n", errno);
		return -1;
	}

	return 0;
}


static void Service_Ser_Close(void)
{
	if (mcu_service_serial_handle != INVALID_HANDLE_VALUE) {
		close(mcu_service_serial_handle);
	}
	mcu_service_serial_handle = INVALID_HANDLE_VALUE;
}


static void Service_Ser_Clear(void)
{
	printf("Service_Ser_Clear: clear the data of uart ...\n");
	tcflush(mcu_service_serial_handle, TCIOFLUSH);
	printf("Service_Ser_Clear: clear end ...\n");
}


static int Service_Ser_Write(unsigned char *pszBuf, int SendCnt, unsigned long TimeOutMS)
{
	int ret = 0;
	struct timeval CurTime;
	fd_set wwrite;

	FD_ZERO(&wwrite);
	FD_SET(mcu_service_serial_handle, &wwrite);

	CurTime.tv_sec = (TimeOutMS / 1000);
	CurTime.tv_usec = (TimeOutMS % 1000) * 1000;

	ret = select(mcu_service_serial_handle + 1, NULL, &wwrite, NULL, &CurTime);
	if (ret == 0) {
		printf("Service_Ser_Write select timeout\n");
		return -1;
	} else if (ret < 0) {
		printf("Service_Ser_Write select error: %d\n", errno);
		return -1;
	}

#ifdef DEBUG_PRINT
	for(int i =0;i<SendCnt;i++)
	{
		printf("Write_Buf[%d]:%2x\n",i,pszBuf[i]);
	}
#endif

	ret = write(mcu_service_serial_handle, pszBuf, SendCnt);
	if (ret <= 0) {
		Service_Ser_Clear();
		printf("Service_Ser_Write write buffer error:%d\n", errno);
		return -1;
	}
	return 0;
}


static int Service_Ser_Read(unsigned char *pszBuf, int ReadCnt,unsigned long TimeOutMS)
{
	int ret = 0;
	int index = 0;
	struct timeval CurTime;
	fd_set wread;

	FD_ZERO(&wread);
	FD_SET(mcu_service_serial_handle, &wread);

	CurTime.tv_sec = (TimeOutMS / 1000);
	CurTime.tv_usec = (TimeOutMS % 1000) * 1000;

	ret = select(mcu_service_serial_handle + 1, &wread, NULL, NULL, &CurTime);
	if (ret == 0) {
		printf("Service_Ser_Read select timeout\n");
		return -1;
	} else if (ret < 0) {
		printf("Service_Ser_Read select error: %d\n", errno);
		return -1;
	}

	ret = read(mcu_service_serial_handle, pszBuf, ReadCnt);
	if (ret <= 0) {
		printf("Service_Ser_Read read buffer error:%d\n",errno);
		return -1;
	}

#ifdef DEBUG_PRINT
	for (int i=0;i<ret;i++){
		printf("Read_Buf[%d]:%2x\n",i,pszBuf[i]);
	}
#endif

	return ret;
}


static int GetCrc16(unsigned char *pszBuf, int ReadCnt) {
	int i = 0;
    unsigned short wResult = 0;
    unsigned short wTableNo = 0;

    for(i = 0; i < ReadCnt; i++)
    {
        wTableNo = ((wResult & 0xff) ^ (pszBuf[i] & 0xff));
        wResult = ((wResult >> 8) & 0xff) ^ wCRC16Table[wTableNo];
    }

	return wResult;
}



static int Get_mcu_version(void)
{
    int crc_val = 0;
    int length = 0;
    int r_bytes = 0;
    int ret = -1;
    unsigned int ver_num = 0;
    int i = 0;
    unsigned char buffer[MAX_BUF_LEN] = {0};
    unsigned char pReponse[MAX_BUF_LEN] = {0};

    Service_Ser_Open();

    buffer[0] = 0x5E;
    buffer[1] = 0x05;
    buffer[2] = 0x00;

    crc_val = GetCrc16(buffer,3);

    buffer[3] = (crc_val>>8) & 0xff;
    buffer[4] = crc_val & 0xff;
    length = 5;

    ret = Service_Ser_Write(buffer,length,W_TIMEOUTMS*length);
    if(ret < 0){
		printf("Get_mcu_version: Service_Ser_Write error!\n");
        return -1;
	}

    r_bytes = Service_Ser_Read(pReponse,MAX_BUF_LEN,R_TIMEOUTMS);
    if(r_bytes <= 0){
		printf("Get_mcu_version: Service_Ser_Read error!\n");
		return -1;
	}

	if((0==pReponse[3]) && (0==pReponse[4]) && (0==pReponse[5]))
	{
		printf("Get current mcu version failed!\n");
		return -1;
	}

    crc_val = GetCrc16(pReponse,r_bytes-2);
#ifdef DEBUG_PRINT
    printf("crc_val = %4x\n",crc_val);
#endif
    if((pReponse[r_bytes-2] == ((crc_val>>8)&0xff)) && (pReponse[r_bytes-1] == (crc_val&0xff))){
        printf("Current mcu ver:%x.%x.%x\n",pReponse[3],pReponse[4],pReponse[5]);
    }else{
        printf("mcu version crc check error!\n");
        return -1;
    }

    ver_num = (pReponse[3]*100) + (pReponse[4]*10)+ pReponse[5];
#ifdef DEBUG_PRINT
    printf("Current ver_num = %d\n",ver_num);
#endif
    return ver_num;
}


static int Get_ota_mcu_version(void)
{
    int ver_file = -1;
    int i = 0;
    int size = 0;
    unsigned int ver_num = 0;
    unsigned char ver_file_buf[MAX_BUF_LEN*4] = {0};
    if(0 != access(MCU_VER_PATH,F_OK))
	{
		printf("Mcu ver file not exist! error!\n");
		return -1;
	}

    FILE *fp = fopen(MCU_VER_PATH,"r");
	if(NULL == fp)
	{
		printf("open ver file error!\n");
		return -1;
	}
	fseek(fp,0L,SEEK_END);
	size = ftell(fp);
	fclose(fp);
#ifdef DEBUG_PRINT
	printf("Ver file size = %d\n",size);
#endif
    ver_file = open(MCU_VER_PATH,O_RDONLY);
	if(-1 == ver_file){
		printf("open ver file error! errno:%d\n",errno);
		return -1;
	}
	read(ver_file,ver_file_buf,size);

#ifdef DEBUG_PRINT
    for(i=0;i<size;i++)
    {
        printf("ver_file_buf[%d]:%2x\n",i,ver_file_buf[i]);
    }
#endif
	printf("Ota mcu ver:%d.%d.%d\n",ver_file_buf[0]-0x30,ver_file_buf[2]-0x30,ver_file_buf[4]-0x30);
    ver_num = ((ver_file_buf[0]-0x30)*100) + ((ver_file_buf[2]-0x30)*10) + (ver_file_buf[4]-0x30);
#ifdef DEBUG_PRINT
	printf("OTA ver_num = %d\n",ver_num);
#endif
    return ver_num;
}


int Compare_ver(void)
{
    int current_ver = -1;
    int ota_ver = -1;

	current_ver = Get_mcu_version();
	if(current_ver <= 0)
	{
		printf("Need update!\n");
		return NEED_UPDATE;
	}

    ota_ver = Get_ota_mcu_version();
    if(ota_ver <= 0)
    {
        printf("Get ota mcu version failed!\n");
		return NO_NEED_UPDATE;
    }

    if(current_ver == ota_ver){
        printf("The same version!\n");
		return NO_NEED_UPDATE;
    }
    else
	{
        printf("New version found!\n");
		return NEED_UPDATE;
    }

    return 0;
}

