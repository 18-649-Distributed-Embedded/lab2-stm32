#ifndef CORE_LOG_H
#define CORE_LOG_H

#include <zephyr/sys/printk.h>

// Debug logging macro conditioned on Kconfig
#ifdef CONFIG_APP_DEBUG_LOGGING
#define DEBUG_PRINT(...) printk(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif

#endif // CORE_LOG_H
