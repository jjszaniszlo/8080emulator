#pragma once

#include "cpu.h"

typedef u8 (*func)();

extern func opcodes[56];
extern func addresing_modes[13];
extern u8 instructions[256][3];
extern u8 instruction_names[256];

// addresing modes
// there are 13 of them.
u8 addr_acc(cpu_state *state);
u8 addr_abs(cpu_state *state);
u8 addr_abx(cpu_state *state);
u8 addr_aby(cpu_state *state);
u8 addr_imm(cpu_state *state);
u8 addr_imp(cpu_state *state);
u8 addr_ind(cpu_state *state);
u8 addr_izx(cpu_state *state);
u8 addr_izy(cpu_state *state);
u8 addr_rel(cpu_state *state);
u8 addr_zp0(cpu_state *state);
u8 addr_zpy(cpu_state *state);
u8 addr_zpx(cpu_state *state);

// opcodes
// there are only 56 opcodes
u8 op_adc(cpu_state *state); // add with carry
u8 op_and(cpu_state *state); // and (accumulator)
u8 op_asl(cpu_state *state); // arithmetic shift left
u8 op_bcc(cpu_state *state); // branch on carry clear
u8 op_bcs(cpu_state *state); // branch on carry set
u8 op_beq(cpu_state *state); // branch on equal (zero set)
u8 op_bit(cpu_state *state); // bit test
u8 op_bmi(cpu_state *state); // branch on minus (negative set)
u8 op_bne(cpu_state *state); // branch on not equal (zero clear)
u8 op_bpl(cpu_state *state); // branch on plus (zero clear)
u8 op_brk(cpu_state *state); // break / interrupt
u8 op_bvc(cpu_state *state); // branch on overflow clear
u8 op_bvs(cpu_state *state); // branch on overflow set
u8 op_clc(cpu_state *state); // clear carry
u8 op_cld(cpu_state *state); // clear decimal
u8 op_cli(cpu_state *state); // clear interrupt disable
u8 op_clv(cpu_state *state); // clear overflow
u8 op_cmp(cpu_state *state); // compare (accumulator)
u8 op_cpx(cpu_state *state); // compare with x
u8 op_cpy(cpu_state *state); // compare with y
u8 op_dec(cpu_state *state); // decrement
u8 op_dex(cpu_state *state); // decrement x
u8 op_dey(cpu_state *state); // decrement y
u8 op_eor(cpu_state *state); // exclusive or (accumulator)
u8 op_inc(cpu_state *state); // increment
u8 op_inx(cpu_state *state); // increment x
u8 op_iny(cpu_state *state); // increment y
u8 op_jmp(cpu_state *state); // jump
u8 op_jsr(cpu_state *state); // jump subroutine
u8 op_lda(cpu_state *state); // load accumulator
u8 op_ldx(cpu_state *state); // load x
u8 op_ldy(cpu_state *state); // load y
u8 op_lsr(cpu_state *state); // logicial shift right
u8 op_nop(cpu_state *state); // no operation
u8 op_ora(cpu_state *state); // or with accumulator
u8 op_pha(cpu_state *state); // push accumulator
u8 op_php(cpu_state *state); // push processor status (sr)
u8 op_pla(cpu_state *state); // pull acuumulator
u8 op_plp(cpu_state *state); // pull processor status (sr)
u8 op_rol(cpu_state *state); // rotate left
u8 op_ror(cpu_state *state); // rotate right
u8 op_rti(cpu_state *state); // return from interrupt
u8 op_rts(cpu_state *state); // return from subroutine
u8 op_sbc(cpu_state *state); // subtract with carry
u8 op_sec(cpu_state *state); // set carry
u8 op_sed(cpu_state *state); // set decimal
u8 op_sei(cpu_state *state); // set interrupt disable
u8 op_sta(cpu_state *state); // store accumulator
u8 op_stx(cpu_state *state); // store x
u8 op_sty(cpu_state *state); // store y
u8 op_tax(cpu_state *state); // transfer accumulator to x
u8 op_tay(cpu_state *state); // transfer accumulator to y
u8 op_tsx(cpu_state *state); // transfer sp to x
u8 op_txa(cpu_state *state); // transfer x to accumulator
u8 op_txs(cpu_state *state); // transfer x to sp
u8 op_tya(cpu_state *state); // transfer y to accumulator

