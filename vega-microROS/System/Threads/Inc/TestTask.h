/*
 * TestTask.h
 *
 *  Created on: Jul 9, 2025
 *      Author: pedro
 */

#ifndef THREADS_INC_TESTTASK_H_
#define THREADS_INC_TESTTASK_H_

#include "Thread.h"

class TestTask : public Thread {
public:
	TestTask();

	void init();
	void loop();
	void ClearFaultsOnce();
	void LTC3889_Recover();
private:
	bool cleared = false;
};

#endif /* THREADS_INC_TESTTASK_H_ */
