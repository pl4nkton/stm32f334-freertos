#pragma once

#include "syscalls.h"

extern struct file_ops term_rs232_ops;

void rs232_poll_send(const char *s);
void rs232_init(void);

