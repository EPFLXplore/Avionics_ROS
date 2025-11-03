/*
 * Thread.cpp
 *
 *  Created on: 2021
 *      Author: AV Team 2021
 */

#include <stdarg.h>
#include <string.h>
#include <Thread.h>


#define DEFAULT_STACK_SIZE (2048) // Danger zone: changing the stack size might create very nasty bugs

static char buffer[128];

void task_run(void* arg) {
	Thread* thread = (Thread*) arg;

	osDelay(pdMS_TO_TICKS(thread->getTickDelay()));

	thread->init();

	while(thread->isRunning()) {
		thread->loop();
		osDelay(pdMS_TO_TICKS(thread->getTickDelay()));
	}

	delete thread;

	vTaskDelete(nullptr);

	while(true) {
		osDelay(pdMS_TO_TICKS(1000));
	}
}

Thread::Thread(const char* name) : Thread(name, (osPriority) osPriorityNormal) {
	;
}

Thread::Thread(const char* name, osPriority priority) : Thread(name, priority, DEFAULT_STACK_SIZE) {
	;
}

Thread::Thread(const char* name, uint32_t stackSize) : Thread(name, (osPriority) osPriorityNormal, stackSize) {
	;
}

Thread::Thread(const char* name, osPriority priority, uint32_t stackSize){
	this->attributes.name = (char*) name;
	this->attributes.stack_size = stackSize;
	this->attributes.priority = (osPriority_t) priority;
}

void Thread::start() {
    if (started) return;
    this->started = true;
    this->handle = osThreadNew(task_run, this, &attributes);
    configASSERT(handle);
    this->name = name;
}

osThreadId Thread::getHandle() {
	return handle;
}

const char* Thread::getName(){
	return this->name;
}

void Thread::terminate() {
	this->running = false;
}


void Thread::setTickDelay(uint32_t ms) {
	this->delay = ms;
}

uint32_t Thread::getTickDelay() {
	return this->delay;
}
