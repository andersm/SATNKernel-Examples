! Sega Saturn startup code by Anders Montonen
! Based on code released by Jeff Frohwein (for GBA) and
! Bart Trzynadlowski (Saturn m68k debugger)
!
! v0.1 - Initial release
!
! This code is placed in the public domain without restrictions

    .section .text

    .extern _main

    .GLOBAL _start
    .ALIGN  2
_start:
    mov.l   stack_init, r15

    ! zero bss
    mov.l   bss_start, r0
    mov.l   bss_end, r1
    mov     #0, r2
start_1:
    cmp/ge  r0, r1
    bt      start_2
    mov.l   r2, @r0
    bra     start_1
    add     #4, r0
start_2:

    ! call the mainline
    mov     #0, r0
    mov     #0, r4              ! int argc
    mov     #0, r5              ! char *argv[]
    mov.l   main_ptr,r0
    jsr     @r0
    or      r0,r0

    ! call exit
    mov     r0,r4
    mov.l   exit_ptr,r0
    jsr     @r0
    or      r0,r0

    ! Someday we may do something useful here..
    ! For now, just jump back to start.
    .ALIGN  2
    .GLOBAL __exit
__exit:
    mov.l   crt_start, r0
    jmp     @r0

    .ALIGN 2
exit_ptr:
    .long   __exit
bss_start:
    .long   __bss_start
bss_end:
    .long   __bss_end
crt_start:
    .long   _start
main_ptr:
    .long    _main
stack_init:
    .long    0x6004000
