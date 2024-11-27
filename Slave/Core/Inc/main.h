#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

// LED pin definitions
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA

// Blue push button pin definition (PC13)
#define BUTTON_Pin GPIO_PIN_13
#define BUTTON_GPIO_Port GPIOC

// Buzzer pin definitions
#define BUZZER_Pin GPIO_PIN_0
#define BUZZER_GPIO_Port GPIOA

#define INPUT_Pin GPIO_PIN_9
#define INPUT_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
