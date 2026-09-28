#ifndef SYS_DRIVETRAIN_H
#define SYS_DRIVETRAIN_H

#include <stdint.h>

extern volatile uint32_t pid_deadline_misses;

void drivetrain_init(void);

#endif // SYS_DRIVETRAIN_H
