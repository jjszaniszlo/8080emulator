#include "cpu.h"
#include "instruction.h"

#include <logger.h>
#include <memory.h>
#include <stdlib.h>
#include <sys/wait.h>

void cpu_reset(cpu_state *state) {
#ifdef LOG_ENABLED
  LOG_INFO("cpu reset");
#endif
  state->address_absolute = 0xFFFC;
  u16 lowerbyte = cpu_read_memory(state, state->address_absolute + 0);
  u16 upperbyte = cpu_read_memory(state, state->address_absolute + 1);

  state->pc = (upperbyte << 8) | lowerbyte;

  state->a = 0x0;
  state->x = 0x0;
  state->y = 0x0;
  state->sp = 0x0;
  state->status = 0x0 | S_U;

  state->cycles_remaining = 8;
}

void cpu_cycle(cpu_state *state) {
  if (state->cycles_remaining == 0) {
    state->current_opcode = cpu_read_memory(state, state->pc++);
    cpu_set_flag(state, S_U, true);

    state->cycles_remaining = instructions[state->current_opcode][2];
    func addr = addresing_modes[instructions[state->current_opcode][1]];
    func opcode = opcodes[instructions[state->current_opcode][0]];

    u8 result1 = addr();
    u8 result2 = opcode();

    state->cycles_remaining += (result1 & result2);
    cpu_set_flag(state, S_U, true);
  }
  state->cycles_elapsed++;
  state->cycles_remaining--;
}

void cpu_interupt_request(cpu_state *state);
void cpu_non_maskable_interrupt(cpu_state *state);

u8 cpu_get_fvalue(cpu_state *state) {
  if (instructions[state->current_opcode][1] != 5)
    state->fvalue = cpu_read_memory(state, state->address_absolute);
  return state->fvalue;
}

u8 cpu_get_flag(cpu_state *state, const status_flags flag) {
#ifdef LOG_ENABLED
  LOG_INFO("cpu get flag $%x", flag);
#endif
  return (state->status & flag) > 0;
}


void cpu_set_flag(cpu_state *state, const status_flags flag, const bool v) {
#ifdef LOG_ENABLED
  LOG_INFO("cpu get set $%x to $d", flag, v);
#endif
  if (v)
    state->status |= flag;
  else
    state->status &= ~flag;
  // & by the inverted flag bits.
}

void cpu_write_memory(cpu_state *state, const u16 address, const u8 data) {
  if (address >= 0x0000 && address <= 0xFFFF)
    state->memory[address] = data;
}

u8 cpu_read_memory(cpu_state *state, const u16 address) {
  if (address >= 0x0000 && address <= 0xFFFF)
    return state->memory[address];
  return 0x00;
}
