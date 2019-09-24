/**
 * @file rtos.c
 * @author ITESO
 * @date Feb 2018
 * @brief Implementation of rtos API
 *
 * This is the implementation of the rtos module for the
 * embedded systems II course at ITESO
 */

#include "rtos.h"
#include "rtos_config.h"
#include "clock_config.h"

#ifdef RTOS_ENABLE_IS_ALIVE
#include "fsl_gpio.h"
#include "fsl_port.h"
#endif
/**********************************************************************************/
// Module defines
/**********************************************************************************/

#define FORCE_INLINE 	__attribute__((always_inline)) inline

#define STACK_FRAME_SIZE			8
#define STACK_LR_OFFSET				2
#define STACK_PSR_OFFSET			1
#define STACK_PSR_DEFAULT			0x01000000
/*Define created for return in case of an invalid task*/
#define INVALID_TASK 				-1
#define MAX_PRIORITY				-1
#define LOCALCLK_TIMEOUT			0
#define LOCALCLK_INIT 				0


/**********************************************************************************/
// IS ALIVE definitions
/**********************************************************************************/

#ifdef RTOS_ENABLE_IS_ALIVE
#define CAT_STRING(x,y)  		x##y
#define alive_GPIO(x)			CAT_STRING(GPIO,x)
#define alive_PORT(x)			CAT_STRING(PORT,x)
#define alive_CLOCK(x)			CAT_STRING(kCLOCK_Port,x)
static void init_is_alive(void);
static void refresh_is_alive(void);
#endif

/**********************************************************************************/
// Type definitions
/**********************************************************************************/

typedef struct
{
	uint8_t priority;
	task_state_e state;
	uint32_t *sp;
	void (*task_body)();
	rtos_tick_t local_tick;
	uint32_t reserved[10];
	uint32_t stack[RTOS_STACK_SIZE];
} rtos_tcb_t;

/**********************************************************************************/
// Global (static) task list
/**********************************************************************************/

struct
{
	uint8_t nTasks;
	rtos_task_handle_t current_task;
	rtos_task_handle_t next_task;
	rtos_tcb_t tasks[RTOS_MAX_NUMBER_OF_TASKS + 1];
	rtos_tick_t global_tick;
	boolean_t context_switch_state;
} task_list =
{ 0 };

/**********************************************************************************/
// Local methods prototypes
/**********************************************************************************/

static void reload_systick(void);
static void dispatcher(task_switch_type_e type);
static void activate_waiting_tasks();
FORCE_INLINE static void context_switch(task_switch_type_e type);
static void idle_task(void);

/**********************************************************************************/
// API implementation
/**********************************************************************************/

void rtos_start_scheduler(void)
{
#ifdef RTOS_ENABLE_IS_ALIVE
	init_is_alive();
	/*Init sys_tick*/
	task_list.global_tick = 0;
	/*Create a task for the processor*/
	rtos_create_task(idle_task,PRIORITY_0,kAutoStart);
	/**/
	task_list.current_task = INVALID_TASK;
#endif
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk
	        | SysTick_CTRL_ENABLE_Msk;
	reload_systick();
	for (;;)
	{

	};
}

rtos_task_handle_t rtos_create_task(void (*task_body)(), uint8_t priority,
		rtos_autostart_e autostart)
{
	/*hecho en clase*/
	rtos_task_handle_t task_handle_val;

	if(RTOS_MAX_NUMBER_OF_TASKS - 1 < task_list.nTasks)
	{
		task_handle_val = INVALID_TASK;
	}
	else
	{
		/*
		 * Si autostart es igual a kAutoStart entonces
		 * autostart es S_READY
		 * sino
		 * autostart es S_SUSPEND
		 */
		task_list.tasks[task_list.nTasks].state = autostart == kAutoStart ? S_READY : S_SUSPENDED;
		/*
		 * Pasar la prioridad de la tarea
		 */
		task_list.tasks[task_list.nTasks].priority = priority;
		/*
		 * We pass the pointer to funct of the task
		 */
		task_list.tasks[task_list.nTasks].task_body = task_body;
		/*
		 * Initilize ticker counter for the task
		 */
		task_list.tasks[task_list.nTasks].local_tick = 0;
		/*
		 * Dejamos un espacio (operacion al final de la funcion)  en el stack para el manejo de los registros
		 */
		task_list.tasks[task_list.nTasks].sp = &(task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE - 1]) - STACK_FRAME_SIZE;
		/*
		 *
		 */
		task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE - STACK_LR_OFFSET] = (uint32_t)task_body;
		/*
		 *
		 */
		task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE - STACK_PSR_OFFSET] = STACK_PSR_DEFAULT;
		/*
		 * Stores number of index of the task
		 */
		task_handle_val = task_list.nTasks;
		/*
		 * Increases the index of number of tasks
		 */
		task_list.nTasks++;
	}
	return task_handle_val;
}

rtos_tick_t rtos_get_clock(void)
{
	/*return uint16_t system clk value*/
	return (SysTick->VAL);
}

void rtos_delay(rtos_tick_t ticks)
{
	/*Send actual Task into WAITING state*/
	task_list.tasks[task_list.current_task].state = S_WAITING;
	/*Assign TICKS to current task*/
	task_list.tasks[task_list.current_task].local_tick = ticks;
	/*Call DISPACHER*/
	dispatcher(kFromNormalExec);
}

void rtos_suspend_task(void)
{
	/*Send actual TASK to SUSPENDED state*/
	task_list.tasks[task_list.current_task].state = S_SUSPENDED;
	/*Call DISPACHER*/
	dispatcher(kFromNormalExec);
}

void rtos_activate_task(rtos_task_handle_t task)
{
	/*Send actual TASK to RUNNING state*/
	task_list.tasks[task_list.current_task].state = S_READY;
	/*Call DISPACHER*/
	dispatcher(kFromNormalExec);
}

/**********************************************************************************/
// Local methods implementation
/**********************************************************************************/

static void reload_systick(void)
{
	SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US,
	        CLOCK_GetCoreSysClkFreq());
	SysTick->VAL = 0;
}

static void dispatcher(task_switch_type_e type)
{
	rtos_task_handle_t next_task = INVALID_TASK;
	uint8_t index = 0;
	uint8_t high_priority = MAX_PRIORITY;

	for(index = 0;index < task_list.nTasks ;index++)
	{
		if(high_priority < task_list.tasks[index].priority
				&& (S_READY == task_list.tasks[index].state
				|| S_RUNNING == task_list.tasks[index].state))
		{
			high_priority = task_list.tasks[index].priority;
			next_task = index;
			task_list.next_task = next_task;
		}
		else
		{
			/*Do nothing*/
		}
	}
	/* Realiza cambio de contexto, si la siguiente tarea es diferente a la tarea actual */
	if(task_list.nTasks != task_list.current_task)
	{
		context_switch(type);
	}
	else
	{
		/*Do nothing*/
	}
}

FORCE_INLINE static void context_switch(task_switch_type_e type)
{
	register uint32_t *sp asm("sp");

	/*
	 * Apuntamos al siguiente (o era anterior no recuerdo) sp
	 * al regresar (o avanzar 9 posiciones en memoria)
	 */

	if(FALSE == task_list.context_switch_state)
	{
		task_list.context_switch_state = TRUE;

	}
	else
	{
		task_list.tasks[task_list.current_task].sp = sp;
	}

	task_list.current_task = task_list.next_task;

	task_list.tasks[task_list.current_task].state = S_RUNNING;

	/*Llamamos al pendsv por medio del bit para el handler*/
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

static void activate_waiting_tasks()
{
	/*
		Decrementa el reloj local de cada tarea en uno. El objetivo de realizar esto,
		es para fijar un limite de tiempo a cada tarea en espera.
	*/

	uint8_t index = 0;

	for(index = 0; index < task_list.nTasks; index++)
	{
		if(S_WAITING == task_list.tasks[index].state)
		{
			/*
			 * Disminuye el locka_tick en 1
			 */
			task_list.tasks[index].local_tick--;
			/*
			 *
			 */
			if(0 == task_list.tasks[index].local_tick)
			{
				/*
				 * Pone la tarea en estado S_READY
				 */
				task_list.tasks[index].state = S_READY;
			}
		}
	}
}

/**********************************************************************************/
// IDLE TASK
/**********************************************************************************/

static void idle_task(void)
{
	for (;;)
	{

	}
}

/****************************************************/
// ISR implementation
/****************************************************/

void SysTick_Handler(void)
{
#ifdef RTOS_ENABLE_IS_ALIVE
	refresh_is_alive();
#endif
	/*Increase g_counter + 1*/
	task_list.global_tick++;
	activate_waiting_tasks();
	/*Call DISPACHER*/
	dispatcher(kFromISR);
	reload_systick();
}

void PendSV_Handler(void)
{
	/*
	   Carga el stack pointer del procesador con el stack pointer de la tarea actual.
	   Cuando ocurre una interrupción/excepción, el compilador ARM guarda una copia
	   del stack pointer del procesador en el registro R7. Al cambiar de contexto,
	   el RTOS debe hacer una copia del stack pointer en el espacio de memoria que
	   corresponde con el stack pointer de la tarea. Para realizar esta operación
	   se requiere el uso del registro R0.
	*/
	/*
	 * Lo usamos para que el procesador nos ayude con los movimientod del sp
	 */
	/*Loads StackPointer of the processor with the one of the actual TASK*/
	/*r0 funciona como un "puntero" hacia r7 para decidir que es lo que almacenara despues r7*/
	register uint32_t r0 asm("r0");

	(void) r0;

	SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;

	/*Se hace necesario un cast ya que r0 es un reg de 32b*/
	r0 = (int32_t)task_list.tasks[task_list.current_task].sp;
	/*
	 * Pasa de r0 a r7 ya que estamos en una excepcion, en donde se almacena en r7
	 * en lugar de r0
	 */
	asm("mov r7,r0");
}

/**********************************************************************************/
// IS ALIVE SIGNAL IMPLEMENTATION
/**********************************************************************************/

#ifdef RTOS_ENABLE_IS_ALIVE
static void init_is_alive(void)
{
	gpio_pin_config_t gpio_config =
	{ kGPIO_DigitalOutput, 1, };

	port_pin_config_t port_config =
	{ kPORT_PullDisable, kPORT_FastSlewRate, kPORT_PassiveFilterDisable,
	        kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
	        kPORT_UnlockRegister, };
	CLOCK_EnableClock(alive_CLOCK(RTOS_IS_ALIVE_PORT));
	PORT_SetPinConfig(alive_PORT(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
	        &port_config);
	GPIO_PinInit(alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
	        &gpio_config);
}

static void refresh_is_alive(void)
{
	static uint8_t state = 0;
	static uint32_t count = 0;
	SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US,
	        CLOCK_GetCoreSysClkFreq());
	SysTick->VAL = 0;
	if (RTOS_IS_ALIVE_PERIOD_IN_US / RTOS_TIC_PERIOD_IN_US - 1 == count)
	{
		GPIO_PinWrite(alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
		        state);
		state = state == 0 ? 1 : 0;
		count = 0;
	} else //
	{
		count++;
	}
}
#endif
///
