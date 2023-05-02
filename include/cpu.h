#pragma once

#include <stdbool.h>
#include <stdlib.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;


typedef enum { // status bits
  S_C = (1 << 0),
  S_Z = (1 << 1),
  S_I = (1 << 2),
  S_D = (1 << 3),
  S_B = (1 << 4),
  S_U = (1 << 5),
  S_V = (1 << 6),
  S_N = (1 << 7)
} status_flags;

typedef struct {
  u16 pc; // program counter
  u8 sp;  // stack pointer

  union { // registers
    u8 rg[3];
    struct {
      u8 a;
      u8 x;
      u8 y;
    };
  };

  u8 status;
  u8 memory[64 * 1024]; // memory

  u8 cycles_remaining; // cycles remaining for instruction
  u32 cycles_elapsed;   // amount of cycles done.
  u8 current_opcode;   // current working code.

  u16 address_absolute; // absolute address in memory
  u16 address_relative; // relative addres in memory corresponding to the
                        // current address
  u8 fvalue;            // helper value for various instructions.
  u16 temp;

} cpu_state;

void cpu_reset(cpu_state *state);
void cpu_cycle(cpu_state *state);

void cpu_interupt_request(cpu_state *state);
void cpu_non_maskable_interrupt(cpu_state *state);
u8 cpu_get_fvalue(cpu_state *state);

u8 cpu_get_flag(cpu_state *state, const status_flags flag);
void cpu_set_flag(cpu_state *state, const status_flags flag, const bool v);

void cpu_write_memory(cpu_state *state, const u16 address, const u8 data);
u8 cpu_read_memory(cpu_state *state, const u16 address);
