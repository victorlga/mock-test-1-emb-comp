#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#define RTT_ALARM_TIME_MS 5

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

// Botão OLED 1
#define BUT_1_PIO		PIOD
#define BUT_1_PIO_ID	ID_PIOD
#define BUT_1_IDX		28
#define BUT_1_PIO_PIN_MASK (1 << BUT_1_IDX)

// Botão OLED 2
#define	BUT_2_PIO      PIOC
#define BUT_2_PIO_ID   ID_PIOC
#define BUT_2_IDX      31
#define BUT_2_PIO_PIN_MASK (1 << BUT_2_IDX)

// Botão OLED 3
#define BUT_3_PIO      PIOA
#define BUT_3_PIO_ID   ID_PIOA
#define BUT_3_IDX      19
#define BUT_3_PIO_PIN_MASK (1 << BUT_3_IDX)

// Motor 1
#define MOT_1_PIO		PIOD
#define MOT_1_PIO_ID	ID_PIOD
#define MOT_1_IDX		30
#define MOT_1_PIO_PIN_MASK (1 << BUT_1_IDX)

// Motor 2
#define	MOT_2_PIO      PIOA
#define MOT_2_PIO_ID   ID_PIOA
#define MOT_2_IDX      6
#define MOT_2_PIO_PIN_MASK (1 << BUT_2_IDX)

// Motor 3
#define MOT_3_PIO      PIOC
#define MOT_3_PIO_ID   ID_PIOC
#define MOT_3_IDX      19
#define MOT_3_PIO_PIN_MASK (1 << BUT_3_IDX)

// Motor 4
#define MOT_4_PIO      PIOA
#define MOT_4_PIO_ID   ID_PIOA
#define MOT_4_IDX      2
#define MOT_4_PIO_PIN_MASK (1 << BUT_3_IDX)

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

QueueHandle_t xQueueModo;
QueueHandle_t xQueueSteps;

SemaphoreHandle_t xSemaphoreRTT;

/** prototypes */
void but_callback(void);
void but_3_callback(void);
void but_2_callback(void);
void but_1_callback(void);
static void BUT_init(void);
static void MOT_init(void);
static void task_modo(void *pvParameters);
static void task_motor(void *pvParameters);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_1_callback(void) {
	uint32_t angle = 180;
	xQueueSendFromISR(xQueueModo, &angle, 0);
}

void but_2_callback(void) {
	uint32_t angle = 90;
	xQueueSendFromISR(xQueueModo, &angle, 0);
}

void but_3_callback(void) {
	uint32_t angle = 45;
	xQueueSendFromISR(xQueueModo, &angle, 0);
}

void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);


	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphoreRTT, &xHigherPriorityTaskWoken);
	}
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		uint32_t alarm_time = IrqNPulses+ul_previous_time;
		rtt_write_alarm_time(RTT, alarm_time);
	}


	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);


	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
		rtt_enable_interrupt(RTT, rttIRQSource);
	else
		rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_modo(void *pvParameters) {
	gfx_mono_ssd1306_init();
	gfx_mono_draw_string("Motor de passos", 0, 0, &sysfont);
	uint32_t angle;
	float angle_step = 0.17578125;
	uint32_t steps;
	char str;
	
	for (;;)  {
		if (xQueueReceive(xQueueModo, &angle, (TickType_t) 0)) {
			gfx_mono_draw_string("                 ", 0, 0, &sysfont);
			sprintf(str, "%d degrees", angle);
			gfx_mono_draw_string(str, 0, 0, &sysfont);
			
			steps = angle / angle_step;
			xQueueSend(xQueueSteps, &steps, 0);
		}
	}
}

static void task_motor(void *pvParameters) {
	
	uint32_t steps;
	uint32_t phase = 0;
	
	for (;;)  {
		if (xQueueReceive(xQueueSteps, &steps, (TickType_t) 0)) {
			printf("Steps: %d\n", steps);
			RTT_init(1000, 5, RTT_MR_ALMIEN);
			while (steps > 0) {
				if (xSemaphoreTake(xSemaphoreRTT, 1000)) {
					steps--;
					
					if (phase == 0) {
						pio_set(MOT_1_PIO, MOT_1_PIO_PIN_MASK);
						pio_clear(MOT_2_PIO, MOT_2_PIO_PIN_MASK);
						pio_clear(MOT_3_PIO, MOT_3_PIO_PIN_MASK);
						pio_clear(MOT_4_PIO, MOT_4_PIO_PIN_MASK);
						phase++;
					} else if (phase == 1) {
						pio_set(MOT_2_PIO, MOT_2_PIO_PIN_MASK);
						pio_clear(MOT_1_PIO, MOT_1_PIO_PIN_MASK);
						pio_clear(MOT_3_PIO, MOT_3_PIO_PIN_MASK);
						pio_clear(MOT_4_PIO, MOT_4_PIO_PIN_MASK);
						phase++;
					} else if (phase == 2) {
						pio_set(MOT_3_PIO, MOT_3_PIO_PIN_MASK);
						pio_clear(MOT_2_PIO, MOT_2_PIO_PIN_MASK);
						pio_clear(MOT_1_PIO, MOT_1_PIO_PIN_MASK);
						pio_clear(MOT_4_PIO, MOT_4_PIO_PIN_MASK);
						phase++;
					} else {
						pio_set(MOT_4_PIO, MOT_4_PIO_PIN_MASK);
						pio_clear(MOT_2_PIO, MOT_2_PIO_PIN_MASK);
						pio_clear(MOT_3_PIO, MOT_3_PIO_PIN_MASK);
						pio_clear(MOT_1_PIO, MOT_1_PIO_PIN_MASK);
						phase = 0;
					}
				}
			}
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void BUT_init(void) {
	
	pmc_enable_periph_clk(BUT_1_PIO_ID);
	
	NVIC_EnableIRQ(BUT_1_PIO_ID);
	NVIC_SetPriority(BUT_1_PIO_ID, 4);

	pio_configure(BUT_1_PIO, PIO_INPUT, BUT_1_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_1_PIO, BUT_1_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_1_PIO, BUT_1_PIO_PIN_MASK);
	pio_handler_set(BUT_1_PIO, BUT_1_PIO_ID, BUT_1_PIO_PIN_MASK, PIO_IT_FALL_EDGE , &but_1_callback);
	
	//////////////////////////////
	
	pmc_enable_periph_clk(BUT_2_PIO_ID);
	
	NVIC_EnableIRQ(BUT_2_PIO_ID);
	NVIC_SetPriority(BUT_2_PIO_ID, 4);

	pio_configure(BUT_2_PIO, PIO_INPUT, BUT_2_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_2_PIO, BUT_2_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_2_PIO, BUT_2_PIO_PIN_MASK);
	pio_handler_set(BUT_2_PIO, BUT_2_PIO_ID, BUT_2_PIO_PIN_MASK, PIO_IT_FALL_EDGE , &but_2_callback);
	
	//////////////////////////////
	
	pmc_enable_periph_clk(BUT_3_PIO_ID);
	
	NVIC_EnableIRQ(BUT_3_PIO_ID);
	NVIC_SetPriority(BUT_3_PIO_ID, 4);

	pio_configure(BUT_3_PIO, PIO_INPUT, BUT_3_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_3_PIO, BUT_3_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_3_PIO, BUT_3_PIO_PIN_MASK);
	pio_handler_set(BUT_3_PIO, BUT_3_PIO_ID, BUT_3_PIO_PIN_MASK, PIO_IT_FALL_EDGE , &but_3_callback);
}

static void MOT_init(void) {
	pmc_enable_periph_clk(MOT_1_PIO_ID);
	pio_configure(MOT_1_PIO, PIO_OUTPUT_0, MOT_1_PIO_PIN_MASK, PIO_DEFAULT);
	
	pmc_enable_periph_clk(MOT_2_PIO_ID);
	pio_configure(MOT_2_PIO, PIO_OUTPUT_0, MOT_2_PIO_PIN_MASK, PIO_DEFAULT);
	
	pmc_enable_periph_clk(MOT_3_PIO_ID);
	pio_configure(MOT_3_PIO, PIO_OUTPUT_0, MOT_3_PIO_PIN_MASK, PIO_DEFAULT);
	
	pmc_enable_periph_clk(MOT_4_PIO_ID);
	pio_configure(MOT_4_PIO, PIO_OUTPUT_0, MOT_4_PIO_PIN_MASK, PIO_DEFAULT);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	MOT_init();
	BUT_init();

	/* Initialize the console uart */
	configure_console();
	
	xQueueModo = xQueueCreate(32, sizeof(uint32_t));
	if (xQueueModo == NULL)
		printf("falha em criar o fila modo\n");
	
	xQueueSteps = xQueueCreate(32, sizeof(uint32_t));
	if (xQueueSteps == NULL)
		printf("falha em criar o fila steps\n");

	xSemaphoreRTT = xSemaphoreCreateBinary();
	if (xSemaphoreRTT == NULL)
		printf("falha em criar o semaforo RTT\n");

	/* Create task to control oled */
	if (xTaskCreate(task_modo, "modo", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create modo task\r\n");
	}
	
	if (xTaskCreate(task_motor, "motor", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create motor task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1);

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
