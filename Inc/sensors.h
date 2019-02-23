/*
 * sensors.h
 *
 *  Created on: 11.02.2019
 *      Author: Szymon
 */

#ifndef SENSORS_H_
#define SENSORS_H_

typedef struct ds18b20_t
{
	int16_t t1;
	int16_t t2;
	int16_t t3;
	int16_t t4;
	int16_t t5;
	int16_t t6;
	int16_t t7;
} ds18b20_t;

void UART_Init();
void DS18B20ReadTemperature();

#endif /* SENSORS_H_ */
