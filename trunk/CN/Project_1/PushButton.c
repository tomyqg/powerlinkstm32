/*
 * PushButton.c
 *
 *  Created on: 21/apr/2010
 *      Author: alessio
 */
#include "PushButton.h"

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {WAKEUP_BUTTON_PORT, TAMPER_BUTTON_PORT,
                                        KEY_BUTTON_PORT, RIGHT_BUTTON_PORT,
                                        LEFT_BUTTON_PORT, UP_BUTTON_PORT,
                                        DOWN_BUTTON_PORT, SEL_BUTTON_PORT};

const uint16_t BUTTON_PIN[BUTTONn] = {WAKEUP_BUTTON_PIN, TAMPER_BUTTON_PIN,
                                        KEY_BUTTON_PIN, RIGHT_BUTTON_PIN,
                                        LEFT_BUTTON_PIN, UP_BUTTON_PIN,
                                        DOWN_BUTTON_PIN, SEL_BUTTON_PIN};

const uint32_t BUTTON_CLK[BUTTONn] = {WAKEUP_BUTTON_CLK, TAMPER_BUTTON_CLK,
                                        KEY_BUTTON_CLK, RIGHT_BUTTON_CLK,
                                        LEFT_BUTTON_CLK, UP_BUTTON_CLK,
                                        DOWN_BUTTON_CLK, SEL_BUTTON_CLK};

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter can be one of following parameters:
  *     @arg Button_WAKEUP: Wakeup Push Button
  *     @arg Button_TAMPER: Tamper Push Button
  *     @arg Button_KEY: Key Push Button
  *     @arg Button_RIGHT: Joystick Right Push Button
  *     @arg Button_LEFT: Joystick Left Push Button
  *     @arg Button_UP: Joystick Up Push Button
  *     @arg Button_DOWN: Joystick Down Push Button
  *     @arg Button_SEL: Joystick Sel Push Button
  * @retval None
  */
void PushButtonInit(Button_TypeDef Button)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable Button GPIO clock */
  RCC_APB2PeriphClockCmd(BUTTON_CLK[Button] | RCC_APB2Periph_AFIO, ENABLE);

  /* Configure Button pin as input floating */
  GPIO_InitStructure.GPIO_Pin = BUTTON_PIN[Button];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStructure);
}



/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter can be one of following parameters:
  *     @arg Button_WAKEUP: Wakeup Push Button
  *     @arg Button_TAMPER: Tamper Push Button
  *     @arg Button_KEY: Key Push Button
  *     @arg Button_RIGHT: Joystick Right Push Button
  *     @arg Button_LEFT: Joystick Left Push Button
  *     @arg Button_UP: Joystick Up Push Button
  *     @arg Button_DOWN: Joystick Down Push Button
  *     @arg Button_SEL: Joystick Sel Push Button
  * @retval The Button GPIO pin value.
  */
uint32_t PushButtonGetState(Button_TypeDef Button)
{
  return GPIO_ReadInputDataBit(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}
