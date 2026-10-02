#ifndef SYS_DRIVETRAIN_H
#define SYS_DRIVETRAIN_H

#include <stdint.h>

extern volatile uint32_t pid_deadline_misses;

extern volatile int32_t dt_target_tps;
extern volatile int32_t dt_filtered_tps_l;
extern volatile int32_t dt_filtered_tps_r;

void drivetrain_init(void);

#endif // SYS_DRIVETRAIN_H
