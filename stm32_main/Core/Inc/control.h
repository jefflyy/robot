/* CONTROL */

#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "pid.h"
#include "wheel.h"
#include "utils.h"

void control_init();

#define BLUERXLEN 7
#define BLUETXLEN 20

void handleBlueRX(char*, char*);
void handleOpenRX(char*);

#endif
