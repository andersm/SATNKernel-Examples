/*

    Example project for SATNKernel

*/

#include "tn_port_asm.h"

    .text

    ! External references
    .extern _frt_irq_handler

    ! Public functions declared in this file
    .global _frt_compare_dispatch

!-----------------------------------------------------------------------------
! Wrapper for FRT timer compare match interrupt
_frt_compare_dispatch:
    _tn_cpu_irq_isr _frt_irq_handler

!-----------------------------------------------------------------------------
!-----------------------------------------------------------------------------
!-----------------------------------------------------------------------------
