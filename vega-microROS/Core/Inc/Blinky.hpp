
// BlinkyThread.h
#pragma once
#include <Threads.hpp>
#include "main.h"     // HAL GPIO prototypes
#include "usbd_cdc_if.h"

class BlinkyThread : public Thread {
public:
    BlinkyThread() : Thread("blink", osPriorityNormal, 1024) {
        setTickDelay(500); // ms
        running = true;    // if not already defaulted in Thread.h
    }
    void init() override {
        // nothing extra
    }
    void loop() override {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4); // LD1 on PB0 (default)
        CDC_Transmit_FS((uint8_t*)"Hello from STM32H7\r\n", 21);
    }
    //bool isRunning() override { return running; }
private:
    bool running;
};


class BlinkyThread600 : public Thread {
public:
    BlinkyThread600()
    : Thread("blink600", osPriorityBelowNormal, 1024) {  // lower priority
        setTickDelay(600);                               // 600 ms
    }
    void init() override {
           // nothing extra
    }
    void loop() override {
	   HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4); // LD1 on PB0 (default)
    }
   //bool isRunning() override { return running; }
   private:
       bool running;
   };


