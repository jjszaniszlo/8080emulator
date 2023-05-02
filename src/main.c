#include <stdio.h>

#include "cpu.h"
#include "logger.h"

cpu_state cpu = {0};

int main() {
  logger_initConsoleLogger(stderr);
  cpu_reset(&cpu);
  return 0;
}
