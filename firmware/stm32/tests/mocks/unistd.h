#pragma once

#include <basetsd.h>
#include <cstddef>
#include <io.h>

using ssize_t = SSIZE_T;

#define STDOUT_FILENO 1

inline ssize_t write(int fd, const void* buf, size_t count)
{
    return _write(fd, buf, static_cast<unsigned int>(count));
}
