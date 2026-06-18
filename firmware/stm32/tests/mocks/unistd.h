#pragma once

#ifdef _WIN32
    #include <BaseTsd.h>
    using ssize_t = SSIZE_T;
#else
    #include <sys/types.h>
#endif



#define STDOUT_FILENO 1

inline ssize_t write(int fd, const void* buf, size_t count)
{
   // return _write(fd, buf, static_cast<unsigned int>(count));
   return count; // pretend we wrote everything
}
