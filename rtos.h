/*
	\file 	  rtos.h
	\brief	  Header file of the RTOS API
	\authors: César Villarreal Hernández, ie707560
	          Luís Fernando Rodríguez Gutiérrez, ie705694
	\date	  17/09/2019
*/

#ifndef SOURCE_RTOS_H_
#define SOURCE_RTOS_H_

#include "rtos_config.h"
#include "stdint.h"

/*! @brief Autostart state type */
typedef enum
{
	kAutoStart, kStartSuspended
} rtos_autostart_e;

/*! @brief Autostart state type */
typedef enum
{
	PRIORITY_0,
	PRIORITY_1,
	PRIORITY_2,
	PRIORITY_3,
	PRIORITY_4,
	PRIORITY_5,
	PRIORITY_6,
	PRIORITY_7,
	PRIORITY_8,
	PRIORITY_9
}rtos_priority_enum;

typedef enum
{
	S_READY = 0,
	S_RUNNING,
    S_WAITING,
    S_SUSPENDED
} task_state_e;

typedef enum
{
	kFromISR = 0, kFromNormalExec
} task_switch_type_e;

typedef enum
{
	FALSE,
	TRUE
} boolean_t;

/*! @brief Task handle type, used to identify a task */
typedef int8_t rtos_task_handle_t;

/*! @brief Tick type, used for time measurement */
typedef uint64_t rtos_tick_t;

/*!
 * @brief Starts the scheduler, from this point the RTOS takes control
 * on the processor
 *
 * @param none
 * @retval none
 */

void rtos_start_scheduler(void);

/*!
 * @brief Create task API function
 *
 * @param task_body pointer to the body of the task
 * @param priority number for the RMS algorithm
 * @param autostart either autostart or start suspended
 * @retval task_handle of the task created
 */
rtos_task_handle_t rtos_create_task(void (*task_body)(), uint8_t priority,
        rtos_autostart_e autostart);

/*!
 * @brief Suspends the task calling this function
 *
 * @param none
 * @retval none
 */
void rtos_suspend_task(void);

/*!
 * @brief Activates the task identified by the task handle
 *
 * @param task handle of the task to be activated
 * @retval none
 */
void rtos_activate_task(rtos_task_handle_t task);

/*!
 * @brief Returns the rtos global tick
 *
 * @param no parameters
 * @retval clock value
 */
rtos_tick_t rtos_get_clock(void);

/*!
 * @brief Suspends the task calling this function by a certain
 * amount of time specified by the parameter ticks
 *
 * @param ticks amount of ticks for the delay
 * @retval none
 */
void rtos_delay(rtos_tick_t ticks);

#endif /* SOURCE_RTOS_H_ */
