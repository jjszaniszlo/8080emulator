#include "instruction.h"
#include "cpu.h"

func opcodes[56] = {
    op_adc, // 0  add with carry
    op_and, // 1  and (accumulator)
    op_asl, // 2  arithmetic shift left
    op_bcc, // 3  branch on carry clear
    op_bcs, // 4  branch on carry set
    op_beq, // 5  branch on equal (zero set)
    op_bit, // 6  bit test
    op_bmi, // 7  branch on minus (negative set)
    op_bne, // 8  branch on not equal (zero clear)
    op_bpl, // 9 branch on plus (zero clear)
    op_brk, // 10 break / interrupt
    op_bvc, // 11 branch on overflow clear
    op_bvs, // 12 branch on overflow set
    op_clc, // 13 clear carry
    op_cld, // 14 clear decimal
    op_cli, // 15 clear interrupt disable
    op_clv, // 16 clear overflow
    op_cmp, // 17 compare (accumulator)
    op_cpx, // 18 compare with x
    op_cpy, // 19 compare with y
    op_dec, // 20 decrement
    op_dex, // 21 decrement x
    op_dey, // 22 decrement y
    op_eor, // 23 exclusive or (accumulator)
    op_inc, // 24 increment
    op_inx, // 25 increment x
    op_iny, // 26 increment y
    op_jmp, // 27 jump
    op_jsr, // 28 jump subroutine
    op_lda, // 29 load accumulator
    op_ldx, // 30 load x
    op_ldy, // 31 load y
    op_lsr, // 32 logicial shift right
    op_nop, // 33 no operation
    op_ora, // 34 or with accumulator
    op_pha, // 35 push accumulator
    op_php, // 36 push processor status (sr)
    op_pla, // 37 pull acuumulator
    op_plp, // 38 pull processor status (sr)
    op_rol, // 39 rotate left
    op_ror, // 40 rotate right
    op_rti, // 41 return from interrupt
    op_rts, // 42 return from subroutine
    op_sbc, // 43 subtract with carry
    op_sec, // 44 set carry
    op_sed, // 45 set decimal
    op_sei, // 46 set interrupt disable
    op_sta, // 47 store accumulator
    op_stx, // 48 store x op_sty, // 49 store y
    op_tax, // 50 transfer accumulator to x
    op_tay, // 51 transfer accumulator to y
    op_tsx, // 52 transfer sp to x
    op_txa, // 53 transfer x to accumulator
    op_txs, // 54 transfer x to sp
    op_tya  // 55 transfer y to accumulator
};

func addresing_modes[13] = {
    addr_acc, // 0  accumulator
    addr_abs, // 1  absolute
    addr_abx, // 2  absolute, x indexed
    addr_aby, // 3  absolute, y indexed
    addr_imm, // 4  immediate
    addr_imp, // 5  implied
    addr_ind, // 6  indirect
    addr_izx, // 7  x indexed, indirect
    addr_izy, // 8  indirect, y indexed
    addr_rel, // 9  relative
    addr_zp0, // 10 zero page
    addr_zpx, // 11 zero page, x indexed
    addr_zpy, // 12 zero page, y indexed
};

u8 instructions[256][3] = {
    // row 0
    {10, 5, 7},  // $00 brk imp 7
    {34, 7, 6},  // $01 ora izx 6
    {33, 5, 2},  // $02 nop     2
    {33, 5, 2},  // $03 nop     2
    {33, 5, 2},  // $04 nop     2
    {34, 10, 3}, // $05 ora zp0 3
    {2, 10, 5},  // $06 asl zp0 5
    {33, 5, 2},  // $07 nop     2
    {36, 5, 3},  // $08 php imp 3
    {34, 4, 2},  // $09 ora imm 2
    {2, 0, 2},   // $0A asl acc 2
    {33, 5, 2},  // $0B nop     2
    {33, 5, 2},  // $0C nop     2
    {34, 1, 4},  // $0D ora abs
    {2, 1, 6},   // $0E asl abs 6
    {33, 5, 2},  // $0F nop     2
    // -- row 1
    {9, 9, 2},   // $10 bpl rel 2
    {34, 8, 5},  // $11 ora izy 5
    {33, 5, 2},  // $12 nop     2
    {33, 5, 2},  // $13 nop     2
    {33, 5, 2},  // $14 nop     2
    {34, 11, 4}, // $15 ora zpx 4
    {2, 11, 6},  // $16 asl zpx 6
    {33, 5, 2},  // $17 nop     2
    {13, 5, 2},  // $18 clc imp 2
    {34, 3, 4},  // $19 ora aby 4
    {33, 5, 2},  // $1A nop     2
    {33, 5, 2},  // $1B nop     2
    {33, 5, 2},  // $1C nop     2
    {34, 2, 4},  // $1D ora abx 4
    {2, 2, 7},   // $1E asl abx 7
    {33, 2, 2},  // $1F nop     2
    // -- row 2
    {28, 1, 6},  // $20 jsr abs 6
    {1, 7, 6},   // $21 and izx 6
    {33, 5, 2},  // $22 nop imp 2
    {33, 5, 2},  // $23 nop imp 2
    {6, 10, 3},  // $24 bit zp0 3
    {1, 10, 3},  // $25 and zp0 3
    {39, 10, 5}, // $26 rol zp0 5
    {33, 5, 2},  // $27 nop imp 2
    {38, 5, 4},  // $28 plp imp 4
    {1, 4, 2},   // $29 and imm 2
    {39, 0, 2},  // $2A rol acc 2
    {33, 5, 2},  // $2B nop imp 2
    {6, 1, 4},   // $2C bit abs 4
    {1, 1, 4},   // $2D and abs 4
    {39, 1, 6},  // $2E rol abs 6
    {33, 5, 2},  // $2F nop imp 2
    // -- row 3
    {7, 9, 2},   // $30 bmi rel 2
    {1, 8, 5},   // $31 and izy 5
    {33, 5, 2},  // $32 nop imp 2
    {33, 5, 2},  // $33 nop imp 2
    {33, 5, 2},  // $34 nop imp 2
    {1, 11, 4},  // $35 and zpx 4
    {39, 11, 6}, // $36 rol zpx 6
    {33, 5, 2},  // $37 nop imp 2
    {44, 5, 2},  // $38 sec imp 2
    {1, 3, 4},   // $39 and aby 4
    {33, 5, 2},  // $3A nop imp 2
    {33, 5, 2},  // $3B nop imp 2
    {33, 5, 2},  // $3C nop imp 2
    {1, 2, 4},   // $3D and abx 4
    {39, 2, 7},  // $3E rol abx 7
    {33, 5, 2},  // $3F nop imp 2
    // -- row 4
    {41, 5, 6},  // $40 rti imp 6
    {23, 7, 6},  // $41 eor izx 6
    {33, 5, 2},  // $42 nop imp 2
    {33, 5, 2},  // $43 nop imp 2
    {33, 5, 2},  // $44 nop imp 2
    {23, 10, 3}, // $45 eor zp0 3
    {32, 10, 5}, // $46 lsr zp0 5
    {33, 5, 2},  // $47 nop imp 2
    {35, 5, 3},  // $48 pha imp 3
    {23, 4, 2},  // $49 eor imm 2
    {32, 0, 2},  // $4A lsr acc 2
    {33, 5, 2},  // $4B nop imp 2
    {27, 1, 3},  // $4C jmp abs 3
    {23, 1, 4},  // $4D eor abs 4
    {32, 1, 6},  // $4E lsr abs 6
    {33, 5, 2},  // $4F nop imp 2
    // -- row 5
    {11, 9, 2},  // $50 bvc rel 2
    {23, 8, 5},  // $51 eor izy 5
    {33, 5, 2},  // $52 nop imp 2
    {33, 5, 2},  // $53 nop imp 2
    {33, 5, 2},  // $54 nop imp 2
    {23, 11, 4}, // $55 eor zpx 4
    {32, 11, 6}, // $56 lsr zpx 6
    {33, 5, 2},  // $57 nop imp 2
    {0, 5, 2},   // $58 cli imp 2
    {23, 3, 4},  // $59 eor aby 4
    {33, 5, 2},  // $5A nop imp 2
    {33, 5, 2},  // $5B nop imp 2
    {33, 5, 2},  // $5C nop imp 2
    {23, 2, 4},  // $5D eor abx 4
    {32, 2, 7},  // $5E lsr abx 7
    {33, 5, 2},  // $5F nop imp 2
    // -- row 6
    {42, 5, 6},  // $60 rts imp 6
    {0, 7, 6},   // $61 adc izx 6
    {33, 5, 2},  // $62 nop imp 2
    {33, 5, 2},  // $63 nop imp 2
    {33, 5, 2},  // $64 nop imp 2
    {0, 10, 3},  // $65 adc zp0 3
    {40, 10, 5}, // $66 ror zp0 5
    {33, 5, 2},  // $67 nop imp 2
    {37, 5, 4},  // $68 pla imp 4
    {0, 4, 2},   // $69 adc imm 2
    {40, 0, 2},  // $6A ror acc 2
    {33, 5, 2},  // $6B nop imp 2
    {27, 6, 5},  // $6C jmp ind 5
    {0, 1, 4},   // $6D adc abs 4
    {40, 1, 6},  // $6E ror abs 6
    {33, 5, 2},  // $6F nop imp 2
    // -- row 7
    {12, 9, 2},  // $70 bvs rel 2
    {0, 8, 5},   // $71 adc izy 5
    {33, 5, 2},  // $72 nop imp 2
    {33, 5, 2},  // $73 nop imp 2
    {33, 5, 2},  // $74 nop imp 2
    {0, 11, 4},  // $75 adc zpx 4
    {40, 11, 6}, // $76 ror zpx 6
    {33, 5, 2},  // $77 nop imp 2
    {46, 5, 2},  // $78 sei imp 2
    {0, 3, 4},   // $79 adc aby 4
    {33, 5, 2},  // $7A nop imp 2
    {33, 5, 2},  // $7B nop imp 2
    {33, 5, 2},  // $7C nop imp 2
    {0, 2, 4},   // $7D adc abx 4
    {40, 2, 7},  // $7E ror abx 7
    {33, 5, 2},  // $7F nop imp 2
    // -- row 8
    {33, 5, 2},  // $80 nop imp 2
    {47, 7, 6},  // $81 sta izx 6
    {33, 5, 2},  // $82 nop imp 2
    {33, 5, 2},  // $83 nop imp 2
    {49, 10, 3}, // $84 sty zp0 3
    {47, 10, 3}, // $85 sta zp0 3
    {48, 10, 3}, // $86 stx zp0 3
    {33, 5, 2},  // $87 nop imp 2
    {22, 5, 2},  // $88 dey imp 2
    {33, 5, 2},  // $89 nop imp 2
    {53, 5, 2},  // $8A txa imp 2
    {33, 5, 2},  // $8B nop imp 2
    {49, 1, 4},  // $8C sty abs 4
    {47, 1, 4},  // $8D sta abs 4
    {48, 1, 4},  // $8E stx abs 4
    {33, 5, 2},  // $8F nop imp 2
    // -- row 9
    {3, 9, 2},   // $90 bcc rel 2
    {47, 8, 6},  // $91 sta izy 6
    {33, 5, 2},  // $92 nop imp 2
    {33, 5, 2},  // $93 nop imp 2
    {49, 11, 4}, // $94 sty zpx 4
    {47, 11, 4}, // $95 sta zpx 4
    {48, 12, 4}, // $96 stx zpy 4
    {33, 5, 2},  // $97 nop imp 2
    {55, 5, 2},  // $98 tya imp 2
    {47, 3, 5},  // $99 sta aby 5
    {54, 5, 2},  // $9A txs imp 2
    {33, 5, 2},  // $9B nop imp 2
    {33, 5, 2},  // $9C nop imp 2
    {47, 2, 5},  // $9D sta abx 5
    {33, 5, 2},  // $9E nop imp 2
    {33, 5, 2},  // $9F nop imp 2
    // -- row A
    {31, 4, 2},  // $A0 ldy imm 2
    {29, 7, 6},  // $A1 lda izx 6
    {30, 4, 2},  // $A2 ldx imm 2
    {33, 5, 2},  // $A3 nop imp 2
    {31, 10, 3}, // $A4 ldy zp0 3
    {29, 10, 3}, // $A5 lda zp0 3
    {30, 10, 3}, // $A6 ldx zp0 3
    {33, 5, 2},  // $A7 nop imp 2
    {51, 5, 2},  // $A8 tay imp 2
    {29, 4, 2},  // $A9 lda imm 2
    {50, 0, 2},  // $AA tax imp 2
    {33, 5, 2},  // $AB nop imp 2
    {31, 1, 4},  // $AC ldy abs 4
    {29, 1, 4},  // $AD lda abs 4
    {30, 1, 4},  // $AE ldx abs 4
    {33, 5, 2},  // $AF nop imp 2
    // -- row B
    {4, 9, 2},   // $B0 bcs rel 2
    {29, 8, 5},  // $B1 lda izy 5
    {33, 5, 2},  // $B2 nop imp 2
    {33, 5, 2},  // $B3 nop imp 2
    {31, 11, 4}, // $B4 ldy zpx 4
    {29, 11, 4}, // $B5 lda zpx 4
    {30, 12, 4}, // $B6 ldx zpy 4
    {33, 5, 2},  // $B7 nop imp 2
    {0, 5, 2},   // $B8 clv imp 2
    {29, 3, 4},  // $B9 lda aby 4
    {52, 5, 2},  // $BA tsx imp 2
    {33, 5, 2},  // $BB nop imp 2
    {31, 2, 4},  // $BC ldy abx 4
    {29, 2, 4},  // $BD lda abx 4
    {30, 3, 4},  // $BE ldx aby 4
    {33, 5, 2},  // $BF nop imp 2
    // -- row C
    {19, 4, 2},  // $C0 cpy imm 2
    {17, 7, 6},  // $C1 cmp izx 6
    {33, 5, 2},  // $C2 nop imp 2
    {33, 5, 2},  // $C3 nop imp 2
    {19, 10, 3}, // $C4 cpy zp0 3
    {17, 10, 3}, // $C5 cmp zp0 3
    {20, 10, 5}, // $C6 dec zp0 5
    {33, 5, 2},  // $C7 nop imp 2
    {26, 5, 2},  // $C8 iny imp 2
    {17, 4, 2},  // $C9 cmp imm 2
    {21, 5, 2},  // $CA dex imp 2
    {33, 5, 2},  // $CB nop imp 2
    {19, 1, 4},  // $CC cpy abs 4
    {17, 1, 4},  // $CD cmp abs 4
    {20, 1, 6},  // $CE dec abs 6
    {33, 5, 2},  // $CF nop imp 2
    // -- row D
    {8, 9, 2},   // $D0 bne rel 2
    {17, 8, 5},  // $D1 cmp izy 5
    {33, 5, 2},  // $D2 nop imp 2
    {33, 5, 2},  // $D3 nop imp 2
    {33, 5, 2},  // $D4 nop imp 2
    {17, 11, 4}, // $D5 cmp zpx 4
    {20, 11, 6}, // $D6 dec zpx 6
    {33, 5, 2},  // $D7 nop imp 2
    {0, 5, 2},   // $D8 cld imp 2
    {17, 3, 4},  // $D9 cmp aby 4
    {33, 5, 2},  // $DA nop imp 2
    {33, 5, 2},  // $DB nop imp 2
    {33, 5, 2},  // $DC nop imp 2
    {17, 2, 4},  // $DD cmp abx 4
    {20, 2, 7},  // $DE dec abx 7
    {33, 5, 2},  // $DF nop imp 2
    // -- row E
    {18, 4, 2},  // $E0 cpx imm 2
    {43, 7, 6},  // $E1 sbc izx 6
    {33, 5, 2},  // $E2 nop imp 2
    {33, 5, 2},  // $E3 nop imp 2
    {18, 10, 3}, // $E4 cpx zp0 3
    {43, 10, 3}, // $E5 sbc zp0 3
    {24, 10, 5}, // $E6 inc zp0 5
    {33, 5, 2},  // $E7 nop imp 2
    {25, 5, 2},  // $E8 inx imp 2
    {43, 4, 2},  // $E9 sbc imm 2
    {33, 5, 2},  // $EA nop imp 2
    {33, 5, 2},  // $EB nop imp 2
    {18, 1, 4},  // $EC cpx abs 4
    {43, 1, 4},  // $ED sbc abs 4
    {24, 1, 6},  // $EE inc abs 6
    {33, 5, 2},  // $EF nop imp 2
    // -- row F
    {5, 9, 2},   // $F0 beq rel 2
    {43, 8, 5},  // $F1 sbc izy 5
    {33, 5, 2},  // $F2 nop imp 2
    {33, 5, 2},  // $F3 nop imp 2
    {33, 5, 2},  // $F4 nop imp 2
    {43, 11, 4}, // $F5 sbc zpx 4
    {24, 11, 6}, // $F6 inc zpx 6
    {33, 5, 2},  // $F7 nop imp 2
    {45, 5, 2},  // $F8 sed imp 2
    {43, 3, 4},  // $F9 sbc aby 4
    {33, 5, 2},  // $FA nop imp 2
    {33, 5, 2},  // $FB nop imp 2
    {33, 5, 2},  // $FC nop imp 2
    {43, 2, 4},  // $FD sbc abx 4
    {24, 2, 7},  // $FE inc abx 7
    {33, 5, 2},  // $FF nop imp 2
};

// addresing modes
// there are 13 of them.

// accumulator is operand.
u8 addr_acc(cpu_state *state) { return 0; }

u8 addr_abs(cpu_state *state) {
  u8 lowerbyte = cpu_read_memory(state, state->pc++);
  u8 upperbyte = cpu_read_memory(state, state->pc++);

  state->address_absolute = (upperbyte << 8) | lowerbyte;
  return 0;
}

u8 addr_abx(cpu_state *state) {
  u8 lowerbyte = cpu_read_memory(state, state->pc++);
  u8 upperbyte = cpu_read_memory(state, state->pc++);

  state->address_absolute = ((upperbyte << 8) | lowerbyte) + state->x;

  if ((state->address_absolute & 0xFF00) != (upperbyte << 8))
    return 1;

  return 0;
}

u8 addr_aby(cpu_state *state) {
  u8 lowerbyte = cpu_read_memory(state, state->pc++);
  u8 upperbyte = cpu_read_memory(state, state->pc++);

  state->address_absolute = ((upperbyte << 8) | lowerbyte) + state->y;

  if ((state->address_absolute & 0xFF00) != (upperbyte << 8))
    return 1;

  return 0;
}

// immediate addressing mode.
// the operand is a value
u8 addr_imm(cpu_state *state) {
  state->address_absolute = state->pc++;
  return 0;
}
// implied.  nothing special.
u8 addr_imp(cpu_state *state) {
  state->fvalue = state->a;
  return 0;
}

// indirect. pointer shit
u8 addr_ind(cpu_state *state) {
  u8 lowerbyte = cpu_read_memory(state, state->pc++);
  u8 upperbyte = cpu_read_memory(state, state->pc++);

  u16 ptr = (upperbyte << 8) | lowerbyte;

  if (lowerbyte == 0x00FF) {
    state->address_absolute = (cpu_read_memory(state, ptr & 0xFF00) << 8) |
                              cpu_read_memory(state, ptr);
  } else {
    state->address_absolute =
        (cpu_read_memory(state, ptr + 1) << 8) | cpu_read_memory(state, ptr);
  }
  return 0;
}

u8 addr_izx(cpu_state *state) {
  u16 operand = cpu_read_memory(state, state->pc++);
  u16 lowerbyte = cpu_read_memory(state, (operand + state->x) & 0x00FF);
  u16 upperbyte = cpu_read_memory(state, (operand + state->x + 1) & 0x00FF);

  state->address_absolute = (upperbyte << 8) | lowerbyte;

  return 0;
}

u8 addr_izy(cpu_state *state) {
  u16 operand = cpu_read_memory(state, state->pc++);
  u16 lowerbyte = cpu_read_memory(state, operand & 0x00FF);
  u16 upperbyte = cpu_read_memory(state, (operand + 1) & 0x00FF);

  state->address_absolute = ((upperbyte << 8) | lowerbyte) + state->y;

  if ((state->address_absolute & 0xFF00) != (upperbyte << 8))
    return 1;
  return 0;
}

u8 addr_rel(cpu_state *state) {
  state->address_relative = cpu_read_memory(state, state->pc);
  state->pc++;
  if (state->address_relative & 0x80)
    state->address_relative |= 0xFF00;
  return 0;
}

// zero page addressing only cares about the first 256 bytes of memory.
u8 addr_zp0(cpu_state *state) {
  state->address_absolute = cpu_read_memory(state, state->pc++);
  state->address_absolute &= 0x00FF;
  return 0;
}
// zero page with an offset of y
u8 addr_zpx(cpu_state *state) {
  state->address_absolute = cpu_read_memory(state, state->pc + state->x);
  state->pc++;
  state->address_absolute &= 0x00FF;
  return 0;
}

u8 addr_zpy(cpu_state *state) {
  state->address_absolute = cpu_read_memory(state, state->pc + state->y);
  state->pc++;
  state->address_absolute &= 0x00FF;
  return 0;
}

// opcodes
// there are only 56 opcodes
// add with carry
u8 op_adc(cpu_state *state) {
  cpu_get_fvalue(state);

  state->temp = state->a + state->fvalue + cpu_get_flag(state, S_C);

  cpu_set_flag(state, S_C, state->temp > 0xFF);
  cpu_set_flag(state, S_Z, (state->temp & 0x00FF) == 0);
  cpu_set_flag(state, S_V,
               (~(state->a ^ state->fvalue) & (state->a ^ state->temp)) &
                   0x0080);
  cpu_set_flag(state, S_N, state->temp * 0x80);

  state->a = state->temp & 0x00FF;

  return 1;
}
// and (accumulator)
u8 op_and(cpu_state *state) {
  cpu_get_fvalue(state);
  state->a &= state->fvalue;

  cpu_set_flag(state, S_Z, state->a == 0x00);
  cpu_set_flag(state, S_N, state->a * 0x80);

  return 1;
}
// arithmetic shift left
u8 op_asl(cpu_state *state) {
  cpu_get_fvalue(state);
  state->temp = state->fvalue << 1;
  cpu_set_flag(state, S_C, (state->temp & 0xFF00) > 0);
  cpu_set_flag(state, S_Z, (state->temp & 0x00FF) == 0x00);
  cpu_set_flag(state, S_N, state->temp & 0x80);

  if (instructions[state->current_opcode][1] == 5)
    state->a = state->temp & 0x00FF;
  else
    cpu_write_memory(state, state->address_absolute, state->temp & 0x00FF);
  return 0;
}

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
