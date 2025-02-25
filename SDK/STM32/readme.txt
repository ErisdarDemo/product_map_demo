@brief 		Firmware Job Search Feedback
@details 	Discussion w/Heitor
@auth 		Justin Reina
@date 		2/24/25


@section 	My Install Location
	
	C:\Sw\ST\STM32CubeIDE_1.17.0\STM32CubeIDE
	
	(they stuff it in this loc down deep; ewwwww)

@section 	Demo Target
	
	STM32F446RE [1] (MCU from a popular Nucleo dev board [2)]

	Something with a connected peripheral
	
	NEw -> STM32 Project

@section 	Demo Procedure

	New IOC:
		GPIO (PA1:Out, PA2: In)
		FreeRTOS(V2 - (2) Tasks &  (1) Binary Semaphore)
		HAL Timebase
		Timer (Ext. Clk - ITR0)
	
	@sel stm32cube_fw_f4_v1281.zip
	
@section 	Reference
	1. https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html
	
	2. https://www.st.com/en/evaluation-tools/nucleo-f446re.html#sample-buy

