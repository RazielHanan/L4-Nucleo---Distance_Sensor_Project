# L4-Nucleo---Distance_Sensor_Project
UART:
uart using interrupts + rx_buffer uses dynamic memory allocation (reallocate every single byte).
using uart_handler struct.


ADC: PC0 works as ADC (ADC3)

DAC: PA4 genereates 1.5V

PA0- TIMER input capture - used to check the duration of "HIGH" state of its input (connects to echo pin the ultrasony sensor- HC-SR04)

PA9- TIMER PWM Generator - used to create 10usec pulse (for "Trig" input of ultrasony sensor - HC-SR04)

PC13 - used as input interrupt to toggle a led.
