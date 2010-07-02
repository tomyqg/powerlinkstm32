/*
 * PushButton.h
 *
 *  Created on: 21/apr/2010
 *      Author: alessio
 */

#ifndef PUSHBUTTON_H_
#define PUSHBUTTON_H_

#include "stm32f10x.h"
#define BUTTONn 3
/**
 * @brief Wakeup push-button
 */
#define WAKEUP_BUTTON_PORT          GPIOA
#define WAKEUP_BUTTON_CLK           RCC_APB2Periph_GPIOA
#define WAKEUP_BUTTON_PIN           GPIO_Pin_0
#define WAKEUP_BUTTON_EXTI_LINE     EXTI_Line0
#define WAKEUP_BUTTON_PORT_SOURCE   GPIO_PortSourceGPIOA
#define WAKEUP_BUTTON_PIN_SOURCE    GPIO_PinSource0
#define WAKEUP_BUTTON_IRQn          EXTI0_IRQn

/**
 * @brief Tamper push-button
 */
#define TAMPER_BUTTON_PORT          GPIOC
#define TAMPER_BUTTON_CLK           RCC_APB2Periph_GPIOC
#define TAMPER_BUTTON_PIN           GPIO_Pin_13
#define TAMPER_BUTTON_EXTI_LINE     EXTI_Line13
#define TAMPER_BUTTON_PORT_SOURCE   GPIO_PortSourceGPIOC
#define TAMPER_BUTTON_PIN_SOURCE    GPIO_PinSource13
#define TAMPER_BUTTON_IRQn          EXTI15_10_IRQn

/**
 * @brief Key push-button
 */
#define KEY_BUTTON_PORT             GPIOB
#define KEY_BUTTON_CLK              RCC_APB2Periph_GPIOB
#define KEY_BUTTON_PIN              GPIO_Pin_9
#define KEY_BUTTON_EXTI_LINE        EXTI_Line9
#define KEY_BUTTON_PORT_SOURCE      GPIO_PortSourceGPIOB
#define KEY_BUTTON_PIN_SOURCE       GPIO_PinSource9
#define KEY_BUTTON_IRQn             EXTI9_5_IRQn

/**
 * @brief Joystick Right push-button
 */
#define RIGHT_BUTTON_PORT           GPIOE
#define RIGHT_BUTTON_CLK            RCC_APB2Periph_GPIOE
#define RIGHT_BUTTON_PIN            GPIO_Pin_0
#define RIGHT_BUTTON_EXTI_LINE      EXTI_Line0
#define RIGHT_BUTTON_PORT_SOURCE    GPIO_PortSourceGPIOE
#define RIGHT_BUTTON_PIN_SOURCE     GPIO_PinSource0
#define RIGHT_BUTTON_IRQn           EXTI0_IRQn
/**
 * @brief Joystick Left push-button
 */
#define LEFT_BUTTON_PORT            GPIOE
#define LEFT_BUTTON_CLK             RCC_APB2Periph_GPIOE
#define LEFT_BUTTON_PIN             GPIO_Pin_1
#define LEFT_BUTTON_EXTI_LINE       EXTI_Line1
#define LEFT_BUTTON_PORT_SOURCE     GPIO_PortSourceGPIOE
#define LEFT_BUTTON_PIN_SOURCE      GPIO_PinSource1
#define LEFT_BUTTON_IRQn            EXTI1_IRQn
/**
 * @brief Joystick Up push-button
 */
#define UP_BUTTON_PORT              GPIOD
#define UP_BUTTON_CLK               RCC_APB2Periph_GPIOD
#define UP_BUTTON_PIN               GPIO_Pin_8
#define UP_BUTTON_EXTI_LINE         EXTI_Line8
#define UP_BUTTON_PORT_SOURCE       GPIO_PortSourceGPIOD
#define UP_BUTTON_PIN_SOURCE        GPIO_PinSource8
#define UP_BUTTON_IRQn              EXTI9_5_IRQn
/**
 * @brief Joystick Down push-button
 */
#define DOWN_BUTTON_PORT            GPIOD
#define DOWN_BUTTON_CLK             RCC_APB2Periph_GPIOD
#define DOWN_BUTTON_PIN             GPIO_Pin_14
#define DOWN_BUTTON_EXTI_LINE       EXTI_Line14
#define DOWN_BUTTON_PORT_SOURCE     GPIO_PortSourceGPIOD
#define DOWN_BUTTON_PIN_SOURCE      GPIO_PinSource14
#define DOWN_BUTTON_IRQn            EXTI15_10_IRQn
/**
 * @brief Joystick Sel push-button
 */
#define SEL_BUTTON_PORT             GPIOD
#define SEL_BUTTON_CLK              RCC_APB2Periph_GPIOD
#define SEL_BUTTON_PIN              GPIO_Pin_12
#define SEL_BUTTON_EXTI_LINE        EXTI_Line12
#define SEL_BUTTON_PORT_SOURCE      GPIO_PortSourceGPIOD
#define SEL_BUTTON_PIN_SOURCE       GPIO_PinSource12
#define SEL_BUTTON_IRQn             EXTI15_10_IRQn

typedef enum
{
  Button_WAKEUP = 0,
  Button_TAMPER = 1,
  Button_KEY = 2,
  Button_RIGHT = 3,
  Button_LEFT = 4,
  Button_UP = 5,
  Button_DOWN = 6,
  Button_SEL = 7
} Button_TypeDef;

void PushButtonInit(Button_TypeDef Button);
uint32_t PushButtonGetState(Button_TypeDef Button);
#endif /* PUSHBUTTON_H_ */
