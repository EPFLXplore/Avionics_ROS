/*
 * System->cpp
 *
 *  Created on: Feb 27, 2025
 *      Author: pcsal
 */


#include "System.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "adc.h"


TestTask* System::test = nullptr;

void System::init(){
	//Allocate memory for the test thread
	test = new TestTask();

	test->start();
}
