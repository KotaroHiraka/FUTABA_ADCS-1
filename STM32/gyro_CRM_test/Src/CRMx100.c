#include "CRMx00.h"

CRMx00_t SetCRM_port(
	SPI_HandleTypeDef* hspi,
	GPIO_TypeDef* ss_port, uint16_t ss_pin,
	GPIO_TypeDef* reset_port, uint16_t reset_pin
	)
{
	CRMx00_t CRMx00;
	CRMx00.spi_port		= hspi;
	CRMx00.ss_port		= ss_port;
	CRMx00.ss_pin		= ss_pin;
	CRMx00.reset_port	= reset_port;
	CRMx00.reset_pin	= reset_pin;

	return CRMx00;
}

void SetCRM_range(CRMx00_t* CRMx00, uint16_t range) {
	CRMx00->range = range;

	switch (range)
	{
	case 75:
		CRMx00->command = 0b00111000;
		break;
	case 150:
		CRMx00->command = 0b00110000;
		break;
	case 300:
		CRMx00->command = 0b00101000;
		break;
	case 900:
		CRMx00->command = 0b00100000;
		break;
	default:
		CRMx00->command = 0b00000000;
		break;
	}
}

void ResetCRM(CRMx00_t* CRMx00) {
	HAL_GPIO_WritePin(CRMx00->reset_port, CRMx00->reset_pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(CRMx00->reset_port, CRMx00->reset_pin, GPIO_PIN_SET);
}


 int UpdateCRM(CRMx00_t* CRMx00, uint32_t Timeout) {
	
	uint8_t send_data[6] = { 0 };
	uint8_t receive_data[6];

	send_data[0] = CRMx00->command;
	send_data[5] = ~(send_data[0] + send_data[1] + send_data[2] + send_data[3] + send_data[4]); //CHECKSUM

	HAL_GPIO_WritePin(CRMx00->ss_port, CRMx00->ss_pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_TransmitReceive(CRMx00->spi_port, send_data, receive_data, 6, Timeout);
	HAL_Delay(1);
	HAL_GPIO_WritePin(CRMx00->ss_port, CRMx00->ss_pin, GPIO_PIN_SET);

	//checksum�m�F�͏ȗ�
	//��ŏ���

	CRMx00->status		= receive_data[0];
	CRMx00->rate		= ((receive_data[1] << 8) + receive_data[2]);
	CRMx00->temperature = ((receive_data[3] << 8) + receive_data[4] - 531);

	CRMx00->rate_f		  = (float)CRMx00->rate / (7200 / CRMx00->range);
	CRMx00->temperature_f = CRMx00->temperature / 2.75;

	return 0;
}

