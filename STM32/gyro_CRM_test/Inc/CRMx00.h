#ifndef __CRMx00_H
#define __CRMx00_H

#include "stm32f4xx_hal.h"


typedef struct
{
	SPI_HandleTypeDef*	spi_port;
	GPIO_TypeDef*		ss_port;
	uint16_t			ss_pin;
	GPIO_TypeDef*		reset_port;
	uint16_t			reset_pin;

	uint16_t range;
	uint8_t  command;
	uint8_t  status;

	int16_t rate  , temperature  ;
	float   rate_f, temperature_f;

}CRMx00_t;


CRMx00_t SetCRM_port
(
	SPI_HandleTypeDef* hspi, 
	GPIO_TypeDef* ss_port   , uint16_t ss_pin,
	GPIO_TypeDef* reset_port, uint16_t reset_pin
);

void SetCRM_range(CRMx00_t*, uint16_t);
void ResetCRM(CRMx00_t*);

int UpdateCRM(CRMx00_t* ,uint32_t Timeout);



#endif