/***************************************************************************** 
  Copyright (c) 2020-2020, XinShiYun Technology Corporation
  All rights reserved.
 
  File Name     :     gpio_control.c
  Programmer    :     Fu Guangbin
  Date          :     2021.1
  The Project   :     XTS100
  Description   :	  used for mcu update
******************************************************************************/

#include "mcu_update.h"


static void gpio_export()
{
	FILE *p = NULL;

	p = fopen("/sys/class/gpio/export","w");
	fprintf(p,"%d",BOOT0_GPIO);
	fclose(p);
	usleep(1000);

	p = fopen("/sys/class/gpio/export","w");
	fprintf(p,"%d",RST_GPIO);
	fclose(p);
	usleep(1000);
}


static void gpio_unexport()
{
	FILE *p = NULL;

	p = fopen("/sys/class/gpio/unexport","w");
	fprintf(p,"%d",BOOT0_GPIO);
	fclose(p);
	usleep(1000);

	p = fopen("/sys/class/gpio/unexport","w");
	fprintf(p,"%d",RST_GPIO);
	fclose(p);
	usleep(1000);
}


void Burn_mode_enter(void)
{
	FILE *p = NULL;

	gpio_export();

	p = fopen("/sys/class/gpio/gpio66/direction","w");
	fprintf(p,"out");
	fclose(p);
	usleep(1000);

	p = fopen("/sys/class/gpio/gpio66/value","w");
	fprintf(p,"%d",1);
	fclose(p);
	usleep(1000);

	p = fopen("/sys/class/gpio/gpio44/direction","w");
	fprintf(p,"out");
	fclose(p);
	usleep(1000);

	p = fopen("/sys/class/gpio/gpio44/value","w");
	fprintf(p,"%d",0);
	fclose(p);
	usleep(1000);

	p = fopen("/sys/class/gpio/gpio44/value","w");
	fprintf(p,"%d",1);
	fclose(p);
	usleep(10000);//10ms

	gpio_unexport();
}


void Run_mode_enter(void)
{
	FILE *p = NULL;

	gpio_export();

	p = fopen("/sys/class/gpio/gpio66/direction","w");
	fprintf(p,"out");
	fclose(p);
	usleep(1000);

	p = fopen("/sys/class/gpio/gpio66/value","w");
	fprintf(p,"%d",0);
	fclose(p);
	usleep(1000);

	p = fopen("/sys/class/gpio/gpio44/direction","w");
	fprintf(p,"out");
	fclose(p);
	usleep(1000);

	p = fopen("/sys/class/gpio/gpio44/value","w");
	fprintf(p,"%d",0);
	fclose(p);
	usleep(1000);

	p = fopen("/sys/class/gpio/gpio44/value","w");
	fprintf(p,"%d",1);
	fclose(p);
	usleep(10000);//10ms

	gpio_unexport();
}

