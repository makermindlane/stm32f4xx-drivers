#include "stm32f411xx_rcc_driver.h"

/*
 * AHB and APB1 prescaler values
 */
int16_t ahbPrescaler[8] = { 2, 4, 8, 16, 64, 128, 256, 512 };
uint8_t apbxPrescaler[4] = { 2, 4, 8, 16 };

/*
 * Get the peripheral clock 1 value.
 */
uint32_t rcc_getPclk1Value() {

	uint32_t sysClk, pClk1;
	uint8_t clkSrc, temp, ahbPres, apb1Pres;
	// get the clock source
	clkSrc = ((RCC->CFGR >> 2) & 0x3);

	if (clkSrc == 0) {
		// HSI is selected as system clock source
		sysClk = 16000000;
	} else if (clkSrc == 1) {
		// HSE is selected as system clock source
		sysClk = 8000000;
	} else if (clkSrc == 2) {
		// PLL is selected as system clock source
		sysClk = rcc_getPllOutputClk();
	}

	// Get AHB prescaler
	// 0---: system clock not divided (or divided by 1)
	// 1000: system clock divided by 2
	// 1001: system clock divided by 4
	// 1010: system clock divided by 8
	// 1011: system clock divided by 16
	// 1100: system clock divided by 64
	// 1101: system clock divided by 128
	// 1110: system clock divided by 256
	// 1111: system clock divided by 512
	temp = ((RCC->CFGR >> 4) & 0xF);
	if (temp < 8) {
		ahbPres = 1;
	} else {
		ahbPres = ahbPrescaler[temp - 8];
	}

	// Get APB1 prescaler
	// 0--: AHB clock not divided (or divided by 1)
	// 100: AHB clock divided by 2
	// 101: AHB clock divided by 4
	// 110: AHB clock divided by 8
	// 111: AHB clock divided by 16
	temp = ((RCC->CFGR >> 10) & 0x7);
	if (temp < 4) {
		apb1Pres = 1;
	} else {
		apb1Pres = apbxPrescaler[temp - 4];
	}

	// For peripheral hanging on APB1 bus, peripheral 1 clock speed = (system clock / AHB prescaler) / APB1 prescaler.
	pClk1 = (sysClk / ahbPres) / apb1Pres;

	return pClk1;
}

/*
 * Get the peripheral clock 2 value.
 */
uint32_t rcc_getPclk2Value() {
	uint32_t sysClk, pClk2;
	uint8_t clkSrc, temp, ahbPres, apb2Pres;
	// get the clock source
	clkSrc = ((RCC->CFGR >> 2) & 0x3);

	if (clkSrc == 0) {
		// HSI is selected as system clock source
		sysClk = 16000000;
	} else if (clkSrc == 1) {
		// HSE is selected as system clock source
		sysClk = 8000000;
	} else if (clkSrc == 2) {
		// PLL is selected as system clock source
		sysClk = rcc_getPllOutputClk();
	}

	// Get AHB prescaler
	// 0---: system clock not divided (or divided by 1)
	// 1000: system clock divided by 2
	// 1001: system clock divided by 4
	// 1010: system clock divided by 8
	// 1011: system clock divided by 16
	// 1100: system clock divided by 64
	// 1101: system clock divided by 128
	// 1110: system clock divided by 256
	// 1111: system clock divided by 512
	temp = ((RCC->CFGR >> 4) & 0xF);
	if (temp < 8) {
		ahbPres = 1;
	} else {
		ahbPres = ahbPrescaler[temp - 8];
	}

	// Get APB2 prescaler
	// 0--: AHB clock not divided (or divided by 1)
	// 100: AHB clock divided by 2
	// 101: AHB clock divided by 4
	// 110: AHB clock divided by 8
	// 111: AHB clock divided by 16
	temp = ((RCC->CFGR >> 13) & 0x7);
	if (temp < 4) {
		apb2Pres = 1;
	} else {
		apb2Pres = apbxPrescaler[temp - 4];
	}

	// For peripheral hanging on APB2 bus, peripheral 2 clock speed = (system clock / AHB prescaler) / APB2 prescaler.
	pClk2 = (sysClk / ahbPres) / apb2Pres;

	return pClk2;
}

/*
 * Get the PLL clock value (as of now it does nothing)
 */
uint32_t rcc_getPllOutputClk() {
	return 16000000;
}
