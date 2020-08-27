/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "Timer.h"
#include "BlinkLed.h"

/* Sempre deve-se incluir o arquivo pt.h ao utilizar protothreads. */
#include "pt.h"
/* Incluir pt-sem.h quando se utiliza semáforos. */
#include "pt-sem.h"

// ----------------------------------------------------------------------------
//
// Standalone STM32F4 led blink sample (trace via DEBUG).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// Then demonstrates how to blink a led with 1 Hz, using a
// continuous loop and SysTick delays.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 3 / 4)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


// Constantes requeridas para manipular o temporizador NVIC SysTick
#define NVIC_SYSTICK_CLK        		0x00000004
#define NVIC_SYSTICK_INT        		0x00000002
#define NVIC_SYSTICK_ENABLE     		0x00000001

// Registradores do ARM Cortex-Mx para o NVIC Systick
#define NVIC_SYSTICK_CTRL       		( ( volatile unsigned long *) 0xe000e010)
#define NVIC_SYSTICK_LOAD       		( ( volatile unsigned long *) 0xe000e014)

// Define se as chamadas a funções printf serão permitidas no exemplo
#define USE_PRINTF						1



/* Estruturas de controle das protothreads utilizadas. */
static struct pt pt1, pt2;
/*---------------------------------------------------------------------------*/
/*
 * A definição a seguir é uma biblioteca simples de temporização
 * utilizada nesse exemplo. A implementação real das funções é
 * encontrada no final desse arquivo.
 */
struct timer { int start, interval; };


/* Funções para implementar o temporizador no sistema. */
int tick_counter = 0;
void SysTick_Handler(void){
	tick_counter++;
}


static int clock_time(void)
{ return (int)tick_counter; }

static int timer_expired(struct timer *t)
{ return (int)(clock_time() - t->start) >= (int)t->interval; }

static void timer_set(struct timer *t, int interval)
{ t->interval = interval; t->start = clock_time(); }

/*---------------------------------------------------------------------------*/
/*
 * Dois temporizadores são utilizados, um para cada tarefa pisca led.
 */
static struct timer timer1, timer2;





/**
 * Primeira tarefa de pisca led.
 * As funções de protothread são executadas pelo laço principal.
 */
static PT_THREAD(protothread1(struct pt *pt)){
  /* Qualquer protothread deve começar com PT_BEGIN(). que recebe
   * o ponteiro de uma estrutura pt. */
  PT_BEGIN(pt);
  /* Como as macros PT_YIELD e PT_YIELD_UNTIL não são utilizadas, evita o warning
   * de compilador devido a variável PT_YIELD_FLAG não estar sendo utilizada.*/
  (void)PT_YIELD_FLAG;

  // Enable GPIO Peripheral clock
  RCC->AHB1ENR |= BLINK_RCC_MASKx(BLINK_PORT_NUMBER);

  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure pin in output push/pull mode
  GPIO_InitStructure.Pin = BLINK_PIN_MASK(BLINK_PIN_NUMBER_GREEN);
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BLINK_GPIOx(BLINK_PORT_NUMBER), &GPIO_InitStructure);

  // Start with led turned off
  blink_led_off();

  /* Em laço para sempre */
  while(1) {
		HAL_GPIO_WritePin(BLINK_GPIOx(BLINK_PORT_NUMBER),
			BLINK_PIN_MASK(BLINK_PIN_NUMBER_GREEN), GPIO_PIN_SET);
	timer_set(&timer1, 500);
    PT_WAIT_UNTIL(pt, timer_expired(&timer1));

    HAL_GPIO_WritePin(BLINK_GPIOx(BLINK_PORT_NUMBER),
        BLINK_PIN_MASK(BLINK_PIN_NUMBER_GREEN), GPIO_PIN_RESET);
	timer_set(&timer1, 500);
    PT_WAIT_UNTIL(pt, timer_expired(&timer1));

    trace_printf("Protothread 1 está executando.\n\r");
  }

  /* Qualquer protothread deve terminar com PT_END(), que recebe
     o ponteiro de uma estrutura pt. */
  PT_END(pt);
}





/**
 * Segunda protothread. Essa função é praticamente a mesma da primeira.
 */
volatile int counter = 0;
static PT_THREAD(protothread2(struct pt *pt)){
	PT_BEGIN(pt);
	(void)PT_YIELD_FLAG;

	// Enable GPIO Peripheral clock
	RCC->AHB1ENR |= BLINK_RCC_MASKx(BLINK_PORT_NUMBER);

	GPIO_InitTypeDef GPIO_InitStructure;

	// Configure pin in output push/pull mode
	GPIO_InitStructure.Pin = BLINK_PIN_MASK(BLINK_PIN_NUMBER_RED);
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(BLINK_GPIOx(BLINK_PORT_NUMBER), &GPIO_InitStructure);

	// Start with led turned off
	blink_led_off();

  while(1) {
	HAL_GPIO_WritePin(BLINK_GPIOx(BLINK_PORT_NUMBER),
		BLINK_PIN_MASK(BLINK_PIN_NUMBER_RED), GPIO_PIN_SET);
	timer_set(&timer2, 1000);
    PT_WAIT_UNTIL(pt, timer_expired(&timer2));

    HAL_GPIO_WritePin(BLINK_GPIOx(BLINK_PORT_NUMBER),
        BLINK_PIN_MASK(BLINK_PIN_NUMBER_RED), GPIO_PIN_RESET);
    timer_set(&timer2, 1000);
    PT_WAIT_UNTIL(pt, timer_expired(&timer2));

    trace_printf("Protothread 2 está executando.\n\r");

    // Conta a cada execução das duas protothreads pisca LEDs.
    counter++;
  }
  PT_END(pt);
}



int main(int argc, char* argv[])
{
  // Send a greeting to the trace device (skipped on Release).
  trace_puts("Hello ARM World!");

  // At this stage the system clock should have already been configured
  // at high speed.
  trace_printf("System clock: %u Hz\n", SystemCoreClock);

  // Desabilita o temporizador NVIC systick
  *(NVIC_SYSTICK_CTRL) = 0;
  // Configura o módulo que determina o período do temporizador
  *(NVIC_SYSTICK_LOAD) = 167999;
  // Inicializa o temporizador systick e sua interrupção
  *(NVIC_SYSTICK_CTRL) = NVIC_SYSTICK_CLK | NVIC_SYSTICK_INT | NVIC_SYSTICK_ENABLE;

  /* Inicializa as variáveis de estado das protothreads com PT_INIT(). */
  PT_INIT(&pt1);
  PT_INIT(&pt2);

	/*
	 * As protothreads são escalonadas repetidamente chamando-se suas
	 * funções e passando o ponteiro da variável de estado com seus
	 * argumentos.
	 */
	while(1) {
	  protothread1(&pt1);
	  protothread2(&pt2);
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
