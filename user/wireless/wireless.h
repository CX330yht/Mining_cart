#ifndef WIRELESS_H
#define WIRELESS_H
#include "control_link.h"
void Wireless_Task(void);
void Wireless_GetCommand(WirelessCommand_t *out);
#endif