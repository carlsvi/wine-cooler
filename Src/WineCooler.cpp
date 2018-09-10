#include "../Inc/WineCooler.hpp"

void wine_cooler_task(void (*uart_tx)(uint8_t*, uint8_t), uint8_t (*uart_rx)(void)) {
	WineCooler wine_cooler(uart_tx, uart_rx);

	while (1) {
		wine_cooler.RunLoop();
	}
}

void WineCooler::Log(int level, std::string message) {
	  char print_buffer[LOG_BUFFER_LENGTH];
	  uint8_t print_length = sprintf (print_buffer, "%d %s\r\n", message);

	  UartTx(print_buffer, print_length);
}

void pwm_setvalue_1(uint16_t value) {
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void pwm_setvalue_2(uint16_t value) {
    TIM_OC_InitTypeDef sConfigOC;

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}
