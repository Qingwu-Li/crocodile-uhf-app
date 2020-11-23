
#include "hal_gpio_imx.h"
#include "unistd.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>


/**
* @brief   Read the value from gpio sysfs.
* @param  const char * file can be:
active_low  device     edge   subsystem  uevent
consumers   direction  power  suppliers  value

export unexport

* @retval The number of milliseconds indicates success.
*/

char * gpio_sysfs_read(int id,const char * file)
{
    char gpio_sysfs_path[256];

        snprintf(gpio_sysfs_path, sizeof(gpio_sysfs_path), "/sys/class/gpio/gpio%d/%s", id,file);
        int fd = open(gpio_sysfs_path, O_RDONLY);
        if (-1 == fd)
            perror(gpio_sysfs_path);
        return NULL;

}
/**
* @brief   Read the value from gpio sysfs.
* @param  const char * file can be:
active_low  device     edge   subsystem  uevent
consumers   direction  power  suppliers  value

export unexport

* @retval The number of milliseconds indicates success.
*/
int gpio_sysfs_write(int id,const char * file,const char * value)
{   int ret=-1;
    char gpio_sysfs_path[256];
        snprintf(gpio_sysfs_path, sizeof(gpio_sysfs_path), "/sys/class/gpio/gpio%d/%s", id,file);
        int fd = open(gpio_sysfs_path, O_WRONLY);
        if (-1 == fd)
            perror(gpio_sysfs_path);
            return -1;
        ret=write(fd, value, strlen(value));
        if (-1 ==ret)
		        perror("gpio_sysfs_write write failed");  
            return -1;
        close(fd);

        return ret;
  
}


int gpio_open_export() 
{
	int export_fd = open("/sys/class/gpio/export",O_WRONLY);
	if (-1 == export_fd)
		perror("/sys/class/gpio/export");
	return export_fd;
}

int gpio_open_unexport() 
{
	int export_fd = open("/sys/class/gpio/unexport",O_RDWR);
	if (-1 == export_fd)
		perror("/sys/class/gpio/unexport");
	return export_fd;
}

int gpio_open_value(int id) 
{
	char path[0x100];
	snprintf(path, 0x100, "/sys/class/gpio/gpio%d/value", id);
	int fd = open(path, O_RDWR);
	if (-1 == fd)
		perror(path);
	return fd;
}

int gpio_open_direction(int id) {
	char path[0x100];
	snprintf(path, 0x100, "/sys/class/gpio/gpio%d/direction", id);
	int fd = open(path, O_RDWR);
	if (-1 == fd)
		perror(path);
	return fd;
}

int gpio_open_edge(int id) {
	char path[0x100];
	snprintf(path, 0x100, "/sys/class/gpio/gpio%d/edge", id);
	int fd = open(path, O_RDWR);
	if (-1 == fd)
		perror(path);
	return fd;
}

void gpio_write(int fd, const char * value) 
{

	if (-1 == write(fd, value, strlen(value)))
		perror("gpio_write failed");
}

int gpio_read(int fd) 
{
  unsigned char value=0;
	if (-1 == read(fd, &value, 1))
		perror("gpio_read failed");
    
  return value-0x30;
}


void RadioGpioInit(RadioGpioPin xGpio, RadioGpioMode xGpioMode)
{
  int f_direction=gpio_open_direction(xGpio);
  char str_gpio_number[4]={0};
  snprintf(str_gpio_number,sizeof(str_gpio_number),"%d",xGpio);


  if(-1==f_direction)
  {
    int f_export=gpio_open_export();
    if(-1==f_export)
    {
      printf("gpio_open_export failed \n");
      return;
    }
    gpio_write(f_export,str_gpio_number);
    close(f_export);
    f_direction=gpio_open_direction(xGpio);
        
  }

  if(-1==f_direction)
  {
    hal_debug("f_direction -1\n");
    return;
  }

  if(RADIO_MODE_GPIO_OUT==xGpioMode)
  {
    hal_debug("set gpio %d to out\n",xGpio);
    gpio_write(f_direction,"out");
    close(f_direction);
  }
  else
  {
    hal_debug("set gpio %d to in\n",xGpio);
    gpio_write(f_direction,"in");
    close(f_direction);

    if(RADIO_MODE_EXTI_IN==xGpioMode)
    {
      int f_edge=gpio_open_edge(xGpio);
      if(-1==f_edge)
      {
        printf("gpio_open_edge failed \n");
        return;
      }



    }
  }
 




}


/**
* @brief  Returns the level of a specified GPIO.
* @param  xGpio Specifies the GPIO to be read.
*         This parameter can be one of following parameters:
* @retval FlagStatus Level of the GPIO. This parameter can be:
*         SET or RESET.
*/
int RadioGpioGetLevel(RadioGpioPin xGpio)
{
  int f_value=gpio_open_value(xGpio) ;
  int gpio_value=0;
  if(-1!=f_value)
  {
    gpio_value=gpio_read(f_value);
    close(f_value);
  }
  
  return gpio_value;
}


/**
* @brief  Sets the level of a specified GPIO.
* @param  xGpio Specifies the GPIO to be set.
*         This parameter can be one of following parameters:
* @param  GPIO_PinState Level of the GPIO. This parameter can be:
*         GPIO_PIN_SET or GPIO_PIN_RESET.
* @retval None.
*/
void RadioGpioSetLevel(RadioGpioPin xGpio, GPIO_PinState xState)
{
  int f_value=gpio_open_value(xGpio) ;
  char str_value[2]={0};
  snprintf(str_value,sizeof(str_value),"%d",xState);

  hal_debug("RadioGpioSetLevel %d %s\n",xGpio,str_value);
  if(-1!=f_value)
  {
    hal_debug("gpio_write %s\n",str_value);
    gpio_write(f_value,str_value);
    close(f_value);
  }

}


/**
* @brief  Puts at logic 1 the SDN pin.
* @param  None.
* @retval None.
*/
void SdkEvalEnterShutdown(void)
{
  
  RadioGpioSetLevel(HOST_GPIO_TO_RADIO_SDN, GPIO_PIN_SET);
  sleep(1);
}


/**
* @brief  Put at logic 0 the SDN pin.
* @param  None.
* @retval None.
*/
void SdkEvalExitShutdown(void)
{
  RadioGpioSetLevel(HOST_GPIO_TO_RADIO_SDN, GPIO_PIN_RESET);
  sleep(1);
}

/**
* @brief  check the logic(0 or 1) at the SDN pin.
* @param  None.
* @retval FlagStatus.
*/
int SdkEvalCheckShutdown(void)
{
  return RadioGpioGetLevel(HOST_GPIO_TO_RADIO_SDN);
}






/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
