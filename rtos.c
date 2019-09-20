/*
	\file 	  rtos.c
	\brief	  Implementation file of the RTOS API
	\authors: César Villarreal Hernández, ie707560
	          Luís Fernando Rodríguez Gutiérrez, ie705694
	\date	  17/09/2019
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

typedef enum
{
	kFromISR = 0, kFromNormalExec
} task_switch_type_e;

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
	rtos_create_task(idle_task, PRIORITY_0, kAutoStart);
#endif
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
	reload_systick();

	for (;;)
	{

	}
}

rtos_task_handle_t rtos_create_task(void (*task_body)(), uint8_t priority, rtos_autostart_e autostart)
{
	rtos_task_handle_t task_ret_index;

	if((RTOS_MAX_NUMBER_OF_TASKS - 1) > task_list.nTasks)
	{
		task_handle_val = INVALID_TASK;
	}
	else
	{
		/* le indica a la tarea si empieza en listo o suspendido */
		task_list.tasks[task_list.nTasks].state = autostart == kAutoStart ? S_READY : S_SUSPENDED;

		/* asigna la prioridad de la tarea */
		task_list.tasks[task_list.nTasks].priority = priority;

		/* asignamos el apuntador de la función a la tarea*/
		task_list.tasks[task_list.nTasks].task_body = task_body;

		/* inicializamos el reloj local */
		task_list.tasks[task_list.nTasks].local_tick = LOCALCLK_INIT;

		/* Dejamos un espacio (operacion al final de la funcion) en el stack para el manejo de los registros */
		task_list.tasks[task_list.nTasks].sp = &(task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE - 1]) - STACK_FRAME_SIZE;
		
		/* Almacena el índice de la tarea actual*/
		task_ret_index = task_list.nTasks;

		/* incrementa el número de tareas */
		task_list.nTasks++;
	}

	return task_ret_index;
}

rtos_tick_t rtos_get_clock(void)
{
	return (SysTick->VAL); //regresa el valor actual del conteo del reloj
}

void rtos_delay(rtos_tick_t ticks)
{
	/* pone la tarea actual en estado de espera */
	task_list.tasks[task_list.current_task].state = S_WAITING;
	/* asigna el numero de ticks que tiene que esperar la tarea */
	task_list.tasks[task_list.current_task].local_tick = ticks;
	/* llama al despachador para continuar la ejecución de tareas*/
	dispatcher(kFromNormalExec);
}

void rtos_suspend_task(void)
{
	/* Pone la tarea actual en estado suspendido*/
	task_list.tasks[task_list.current_task].state = S_SUSPENDED;
	/* llama al despachador para continuar la ejecución de tareas*/
	dispatcher(kFromNormalExec);
}

void rtos_activate_task(rtos_task_handle_t task)
{
	/* pone la tarea actual en estado de listo */
	task_list.tasks[task_list.current_task].state = S_READY;
	/* llama al despachador para continuar la ejecución de tareas*/
	dispatcher(kFromNormalExec);
}

/**********************************************************************************/
// Local methods implementation
/**********************************************************************************/

static void reload_systick(void)
{
	/* se recarga el valor del systick */
	SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US, CLOCK_GetCoreSysClkFreq());
	SysTick->VAL = 0;
}

static void dispatcher(task_switch_type_e type)
{
	rtos_task_handle_t next_task;
	uint8_t index;
	uint8_t high_priority;

	next_task = INVALID_TASK;
	high_priority = MAX_PRIORITY; 
	index = 0;

	for(index = 0; index < task_list.nTasks; index++)
	{
		/* Encuentra la tarea con prioridad más alta, y verifica que su estado sea listo o corriendo */
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
			/* do-nothing */
		}
		
	}

	/* Realiza cambio de contexto, si la siguiente tarea es diferente a la tarea actual */
	if(task_list.nTasks != task_list.current_task)
	{
		context_switch(type);
	}
	else
	{
		/* do-nothing */
	}
	
}

FORCE_INLINE static void context_switch(task_switch_type_e type)
{
	register uint32_t *sp asm("sp");

	/*
	 * Apuntamos al siguiente (o era anterior no recuerdo) sp
	 * al regresar (o avanzar 9 posiciones en memoria)
	 */
	task_list.tasks[task_list.current_task].sp = sp - 9; // VERIFICAR

	/* cambia la tarea actual por la siguiente tarea */
	task_list.current_task = task_list.next_task;
	/* pone la siguiente tarea en estado ejecución (running state) */
	task_list.tasks[task_list.current_task].state = S_RUNNING;
}

static void activate_waiting_tasks()
{
	/*
		Decrementa el reloj local de cada tarea en uno. El objetivo de realizar esto,
		es para fijar un limite de tiempo a cada tarea en espera.
	*/

	uint8_t index;

	for(index = 0; index < task_list.nTasks; index++)
	{
		/* disminuye el reloj local de la tarea en uno */
		task_list.tasks[index].local_tick--;
	
		/* verifica si el reloj local de la tarea está en cero */
		if(LOCALCLK_TIMEOUT == task_list.tasks[index].local_tick)
		{
			/* pone la tarea en estado de listo (ready state) */
			task_list.tasks[index].state = S_READY;
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
	/* incrementa el reloj global en 1 */
	task_list.global_tick++;
	/* activa las tareas en espera */
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

	register uint32_t r0 asm("r0");
	(void) r0;

	/* Se indica al procesador que ya se atendió la interrupción */
	SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk; //remueve el 'pending state' de la excepción PendSV

	/* Copiamos la dirección del stack pointer de la tarea actual en R0*/
	r0 = (int32_t) task_list.tasks[task_list.current_task].sp;  //Se hace un cast ya que r0 es un registro de 32-bits.

	/* guardamos el stack pointer actual en memoria */
	asm("mov r7, r0");
}

/**********************************************************************************/
// IS ALIVE SIGNAL IMPLEMENTATION
/**********************************************************************************/
#ifdef RTOS_ENABLE_IS_ALIVE
static void init_is_alive(void)
{
	gpio_pin_config_t gpio_config = { kGPIO_DigitalOutput, 1, };

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
	static uint8_t state;
	static uint32_t count;

	state = 0;
	count = 0;
	SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US, CLOCK_GetCoreSysClkFreq());
	SysTick->VAL = 0;

	if ((RTOS_IS_ALIVE_PERIOD_IN_US / (RTOS_TIC_PERIOD_IN_US - 1)) == count)
	{
		GPIO_PinWrite(alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
		        state);
		state = state == 0 ? 1 : 0;
		count = 0;
	} 
	else 
	{
		count++;
	}
}
#endif
