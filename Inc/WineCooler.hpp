#ifndef WINE_COOLER_HPP
#define WINE_COOLER_HPP

#include <string>
#include <array>

#include "Temp.hpp"
#include "Peltier.hpp"
#include "FanArray.hpp"

#include <stdint.h>

#include "wine_cooler_task.h"
#include "main.h"
#include "tim.h"

#define LOG_BUFFER_LENGTH 80

void pwm_setvalue_1(uint16_t value);
void pwm_setvalue_2(uint16_t value);

extern "C" void wine_cooler_task(void (*uart_tx)(uint8_t*, uint8_t), uint8_t (*uart_rx)(void));

class WineCooler {
public:
	WineCooler(void (*uart_tx)(uint8_t*, uint8_t), uint8_t (*uart_rx)(void));
	void RunLoop();
private:
	void Log(int level, std::string message);

	// Function pointers.
	void (*UartTx)(uint8_t* message, uint8_t length);
	uint8_t (*UartRx)();

	std::array<Temp, 3> temp_;
	std::array<Peltier, 2> peltier_;
	FanArray fan_array_internal_;
	FanArray fan_array_external_;
};

#endif /* WINE_COOLER_HPP */
