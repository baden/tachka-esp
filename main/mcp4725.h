#pragma once

#include <stdbool.h>

int mcp4725_init();
int mcp4725_set_voltage(int voltage, bool second_dac);
