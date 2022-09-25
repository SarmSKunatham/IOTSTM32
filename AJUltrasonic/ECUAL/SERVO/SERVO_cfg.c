/*
 * File: SERVO_cfg.c
 * Driver Name: [[ SERVO Motor ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "SERVO.h"

const SERVO_CfgType SERVO_CfgParam[SERVO_NUM] =
{
	// Servo Motor 1 Configurations
    {
	    GPIOA,
		GPIO_PIN_7,
		TIM3,
		&TIM3->CCR2,
		TIM_CHANNEL_2,
		84000000,
		1.0,
		2.0
	}
};
