/*
 * shell_task.h
 *
 *  Created on: Jan 30, 2021
 *      Author: jrosen
 */

#ifndef SHELL_TASK_H_
#define SHELL_TASK_H_

#define SHELL_SERIAL_UART_INSTANCE (6)
#define SHELL_SERIAL_BAUD_RATE (115200)
#define SHELL_SERIAL_CLK_FREQ (80000000U)
#define SHELL_RECEIVE_BUFFER_LEN (512U)
#define SHELL_TRANSMIT_BUFFER_LEN (512U)

void ShellTask( void *pvParameters );

void ShutdownShell();


#endif /* SHELL_TASK_H_ */
