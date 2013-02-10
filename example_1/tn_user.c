/*
    Example project for SATNKernel

    Two tasks write to the USB devcart FIFO.
    FIFO access is protected by a semaphore.
    The CPU free-running timer's compare match
    interrupt is used for the timer tick.

 */

#include <stdint.h>
#include "tn.h"
#include "tn_port_config.h"

// 10 ms tick
#define TICK_MS_INTERVAL 10

// FRT compare match interrupt vector
// (see System Program User's Manual)
#define FRT_TIMER_VECTOR             101

#define TASK_A_PRIORITY 5
#define TASK_B_PRIORITY 7

// Task stacks, in words
#define TASK_A_STK_SIZE 128
#define TASK_B_STK_SIZE 128

TN_KERN_CTX kernelcontext;
TN_SEM      port_lock;
TN_TCB      task_A;
TN_TCB      task_B;

unsigned int task_A_stack[TASK_A_STK_SIZE];
unsigned int task_B_stack[TASK_B_STK_SIZE];

// ISR wrapper/dispatch
extern void frt_compare_dispatch(void);

// VDP2 video mode registers
#define VDP2_BASE           0x25e00000
#define VDP2_REGISTER_BASE  (VDP2_BASE+0x180000)
#define TVMD    (*(volatile uint16_t *)(VDP2_REGISTER_BASE+0x00))
#define HRESO0  (1<<0)
#define TVSTAT  (*(volatile uint16_t *)(VDP2_REGISTER_BASE+0x04))
#define PAL     (1<<0)

// SH7604 hardware registers
#define TIER        (*(volatile uint8_t *)0xfffffe10)
    #define OCIAE       (1<<3)
#define FTCSR       (*(volatile uint8_t *)0xfffffe11)
    #define OCFA        (1<<3)
    #define CCLRA       (1<<0)
#define FRCH        (*(volatile uint8_t *)0xfffffe12)
#define FRCL        (*(volatile uint8_t *)0xfffffe13)
#define OCRAH       (*(volatile uint8_t *)0xfffffe14)
#define OCRAL       (*(volatile uint8_t *)0xfffffe15)
#define TCR         (*(volatile uint8_t *)0xfffffe16)
    #define CKS1        (1<<1)
    #define CKS0        (1<<0)
#define TOCR        (*(volatile uint8_t *)0xfffffe17)
    #define OCRS        (1<<4)
#define VCRC        (*(volatile uint16_t*)0xfffffe66)

//----------------------------------------------------------------------------
// SH7604 timer init function
//----------------------------------------------------------------------------
void frt_init(void)
{
    // Precalculated timer tick values for /8 prescaler
    static const uint16_t tickValues[4] =
    {
        26874100/(8 * 1000.0f/TICK_MS_INTERVAL), /* NTSC 320/640 */
        26687499/(8 * 1000.0f/TICK_MS_INTERVAL), /* PAL 320/640  */
        28636400/(8 * 1000.0f/TICK_MS_INTERVAL), /* NTSC 352/704 */
        28437500/(8 * 1000.0f/TICK_MS_INTERVAL), /* PAL 352/704  */
    };

    uint16_t tickValue;

    // Disable FRT interrupts
    TIER = 0;

    // Set interrupt vector - should be done by system init code
    VCRC = (VCRC & 0xFF00) | FRT_TIMER_VECTOR;

    tickValue = tickValues[((TVMD & HRESO0) << 1) | (TVSTAT & PAL)];

    // Clock source is internal clock/8
    TCR &=  ~(CKS1|CKS0);

    // Clear FRC on compare match A
    FTCSR |= CCLRA;

    // Map OCRA register
    TOCR &= ~OCRS;

    // Set match A value
    OCRAH = tickValue >> 8;
    OCRAL = tickValue;

    // Clear FRC value
    FRCH = 0;
    FRCL = 0;

    // Enable output compare A interrupt
    TIER = OCIAE;
}

//----------------------------------------------------------------------------
// SH7604 timer interrupt handler
//----------------------------------------------------------------------------
void frt_irq_handler(void)
{
    unsigned char st = FTCSR;

    if (st & OCFA)
    {
        tn_tick_int_processing();
        FTCSR = (~OCFA)|CCLRA;
    }
}

void tn_cpu_int_enable(void)
{
    tn_cpu_restore_sr(0);
}

#define USB_FLAGS (*(volatile unsigned char*)(0x22200001))
#define USB_RXF     (1 << 0)
#define USB_TXE     (1 << 1)
#define USB_PWREN   (1 << 7)
#define USB_FIFO (*(volatile unsigned char*)(0x22100001))

void printmessage(const char *message)
{
    while (*message != '\0')
    {
        while ((USB_FLAGS & USB_TXE) != 0) ;
        USB_FIFO = *message++;
    }
    return;
}

int main(void)
{
    // disable interrupts
    tn_cpu_save_sr();

    // hook timer interrupt vector
    tn_hook_vec(FRT_TIMER_VECTOR, frt_compare_dispatch);

    // init timer
    frt_init();

    // start kernel, does not return
    tn_start_system(&kernelcontext);

    return 1;
}

void task_A_func(void *par)
{
    for(;;)
    {
        tn_sem_acquire(&port_lock, TN_WAIT_INFINITE);
        printmessage("task A\n");
        tn_sem_signal(&port_lock);
        tn_task_sleep((unsigned long)par);
    }
}

void task_B_func(void *par)
{
    for(;;)
    {
        tn_sem_acquire(&port_lock, TN_WAIT_INFINITE);
        printmessage("task B\n");
        tn_sem_signal(&port_lock);
        tn_task_sleep((unsigned long)par);
    }
}

void tn_app_init(void)
{
    //--- USB FIFO lock
    tn_sem_create(&port_lock, 1, 1);
    //--- Task A
    task_A.id_task = 0;
    tn_task_create(&task_A,                   //-- task TCB
                   task_A_func,               //-- task function
                   TASK_A_PRIORITY,           //-- task priority
                   &(task_A_stack             //-- task stack first addr in memory
                     [TASK_A_STK_SIZE-1]),
                   TASK_A_STK_SIZE,           //-- task stack size (in int,not bytes)
                   (void*)100,                //-- task function parameter
                   TN_TASK_START_ON_CREATION  //-- Creation option
                   );
    //--- Task B
    task_B.id_task = 0;
    tn_task_create(&task_B,                   //-- task TCB
                   task_B_func,               //-- task function
                   TASK_B_PRIORITY,           //-- task priority
                   &(task_B_stack             //-- task stack first addr in memory
                     [TASK_B_STK_SIZE-1]),
                   TASK_B_STK_SIZE,           //-- task stack size (in int,not bytes)
                   (void*)150,                //-- task function parameter
                   TN_TASK_START_ON_CREATION  //-- Creation option
                   );
}
