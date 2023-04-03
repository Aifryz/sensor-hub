/*
 * log.cpp
 *
 *  Created on: Feb 13, 2023
 *      Author: robal
 */

#include "unistd.h"

namespace log::impl{

void log_part(const char* fmt, size_t beg, size_t end)
{
	int fd = STDOUT_FILENO;
	write(fd, fmt+beg, end-beg);
}

}
