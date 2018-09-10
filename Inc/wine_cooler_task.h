#ifndef WINE_COOLER_TASK_H
#define WINE_COOLER_TASK_H

extern "C" {
	void uart_tx(uint8_t* buffer, uint8_t buffer_length);
	uint8_t uart_rx(void);

	void wine_cooler_task(void (*uart_tx)(uint8_t*, uint8_t), uint8_t (*uart_rx)(void));
}

#endif /* WINE_COOLER_TASK_H */
