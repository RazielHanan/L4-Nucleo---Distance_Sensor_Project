#include "tim_handler.h"
#include "main.h"

extern volatile uint32_t ic_val1;
extern volatile uint32_t ic_val2;
extern volatile uint8_t is_first_captured;
extern volatile uint32_t pulse_width_us;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (is_first_captured == 0)
        {
        	ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            is_first_captured = 1;
            //__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else if (is_first_captured == 1)
        {
            ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            uint32_t captured;
            if (ic_val2 > ic_val1)
                captured = ic_val2 - ic_val1;
            else
                captured = (0xFFFFFFFF - ic_val1) + ic_val2;

            pulse_width_us = (captured + 40) / 80; // round to nearest // Since 1 tick = 1 µs
            is_first_captured = 0;

            // Reset to rising edge to prepare for next pulse
            //__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}
