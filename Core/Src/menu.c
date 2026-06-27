/*
 * menu.c
 *
 *  Created on: Jun 27, 2026
 *      Author: Eylül Öztek
 */

#include "menu.h"

void displayMainMenu(float setPoint, float temperature, uint8_t PWM) {

	char setPointBuffer[32] = { 0 };
	char temperatureBuffer[32] = { 0 };
	char PWMBuffer[16] = { 0 };
	ssd1306_Fill(Black);

	sprintf(setPointBuffer, "Set Point:%.2f", setPoint);
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString(setPointBuffer, Font_7x10, White);

	sprintf(temperatureBuffer, "Temp:%.2f°C", temperature);
	ssd1306_SetCursor(0, 20);
	ssd1306_WriteString(temperatureBuffer, Font_7x10, White);

	sprintf(PWMBuffer, "PWM:%d %%", PWM);
	ssd1306_SetCursor(0, 40);
	ssd1306_WriteString(PWMBuffer, Font_7x10, White);

	ssd1306_UpdateScreen();

}
