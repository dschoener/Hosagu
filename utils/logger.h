//////////////////////////////////////////////////////////////////////////
/*
 * Copyright (c) 2017
 *
 *  Created on: 21.04.2017
 *      Author: denis
 */
//////////////////////////////////////////////////////////////////////////

#ifndef _LOGGER_H_
#define _LOGGER_H_

#include <esp_libc.h>
#include <esp_system.h>

#define log(_SEVERITY, _FORMAT, ...) printf("[%10d] " _SEVERITY ": " _FORMAT "\n", system_get_time(), ##__VA_ARGS__)
#define log_debug(_FORMAT, ...) log("  DEBUG", _FORMAT, ##__VA_ARGS__)
#define log_info(_FORMAT, ...)  log("   INFO", _FORMAT, ##__VA_ARGS__)
#define log_warn(_FORMAT, ...)  log("WARNING", _FORMAT, ##__VA_ARGS__)
#define log_error(_FORMAT, ...) log("  ERROR", _FORMAT, ##__VA_ARGS__)

#endif // _LOGGER_H_
