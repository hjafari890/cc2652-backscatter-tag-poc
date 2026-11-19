/*
 * main_nortos.c
 * NoRTOS startup entry point for LAUNCHXL-CC26X2R1 Backscatter Tag PoC.
 */
#include <stdint.h>
#include <stddef.h>

#include <NoRTOS.h>
#include <ti/drivers/Board.h>

extern void *mainThread(void *arg0);

int main(void)
{
    Board_init();
    NoRTOS_start();
    mainThread(NULL);

    while (1) {}
}
