/*
 * ms100.h
 *
 *  Created on: Jun 9, 2017
 *      Author: denis
 */

#ifndef MS100_MS100_H_
#define MS100_MS100_H_

#include <espressif/c_types.h>

/**
 * Reads out the MiniSennse100 current output voltage.
 * @return Current voltage in milli volt.
 */
uint32 ms100_read_mv();

#endif /* MS100_MS100_H_ */
