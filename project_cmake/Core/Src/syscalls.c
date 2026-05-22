#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>

#include "bsp_debug_uart.h"

#ifndef STDIN_FILENO
#define STDIN_FILENO 0
#endif

#ifndef STDOUT_FILENO
#define STDOUT_FILENO 1
#endif

#ifndef STDERR_FILENO
#define STDERR_FILENO 2
#endif

extern char _end;
extern char _estack;

static char *s_heap_end;

/**
 * @brief Extend the newlib heap while keeping a guard below the stack.
 * @param incr Number of bytes requested by newlib.
 * @return Previous heap end on success, or `(void *)-1` on failure.
 */
void *_sbrk(ptrdiff_t incr)
{
    char *previous_heap_end; /* Keep the address returned to newlib. */
    char *next_heap_end;     /* Keep the candidate heap boundary. */
    char *stack_limit;       /* Reserve the linker stack area. */

    if (s_heap_end == 0) /* Initialize from the linker-provided end symbol. */
    {
        s_heap_end = &_end;
    }

    previous_heap_end = s_heap_end; /* Save the current heap end. */
    next_heap_end = s_heap_end + incr; /* Calculate the requested heap end. */
    stack_limit = &_estack - 0x400; /* Match the linker minimum stack size. */

    if (next_heap_end > stack_limit) /* Refuse to grow into the stack guard. */
    {
        errno = ENOMEM;
        return (void *)-1;
    }

    s_heap_end = next_heap_end; /* Commit the heap extension. */
    return previous_heap_end;
}

/**
 * @brief Write stdout and stderr bytes through the existing debug UART path.
 * @param file Newlib file descriptor.
 * @param ptr Buffer to write.
 * @param len Buffer length in bytes.
 * @return Number of bytes written, or `-1` for unsupported descriptors.
 */
int _write(int file, char *ptr, int len)
{
    int i; /* Byte index for the transmit loop. */

    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) /* Only route console output. */
    {
        errno = EBADF;
        return -1;
    }

    for (i = 0; i < len; i++) /* Send each byte through the existing BSP UART. */
    {
        if (ptr[i] == '\n') /* Keep terminal line endings readable. */
        {
            BSP_DebugUart_SendChar('\r');
        }
        BSP_DebugUart_SendChar((uint8_t)ptr[i]);
    }

    return len;
}

/**
 * @brief Read is unsupported because this firmware has no stdin backend.
 * @param file Newlib file descriptor.
 * @param ptr Destination buffer.
 * @param len Requested byte count.
 * @return Always `-1`.
 */
int _read(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    (void)len;
    errno = ENOSYS;
    return -1;
}

/**
 * @brief Close is unsupported for bare-metal descriptors.
 * @param file Newlib file descriptor.
 * @return Always `-1`.
 */
int _close(int file)
{
    (void)file;
    errno = ENOSYS;
    return -1;
}

/**
 * @brief Report terminal status for stdin, stdout, and stderr.
 * @param file Newlib file descriptor.
 * @return `1` for console descriptors, otherwise `0`.
 */
int _isatty(int file)
{
    if ((file == STDIN_FILENO) || (file == STDOUT_FILENO) || (file == STDERR_FILENO))
    {
        return 1;
    }

    errno = EBADF;
    return 0;
}

/**
 * @brief Seek is unsupported for bare-metal descriptors.
 * @param file Newlib file descriptor.
 * @param ptr Offset requested by newlib.
 * @param dir Seek direction.
 * @return Always `-1`.
 */
int _lseek(int file, int ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    errno = ENOSYS;
    return -1;
}

/**
 * @brief Return a character-device stat block for console descriptors.
 * @param file Newlib file descriptor.
 * @param st Output stat structure.
 * @return `0` on success, or `-1` for unsupported descriptors.
 */
int _fstat(int file, struct stat *st)
{
    if ((file == STDIN_FILENO) || (file == STDOUT_FILENO) || (file == STDERR_FILENO))
    {
        st->st_mode = S_IFCHR;
        return 0;
    }

    errno = EBADF;
    return -1;
}

/**
 * @brief Return a dummy process id for newlib.
 * @return Always `1`.
 */
int _getpid(void)
{
    return 1;
}

/**
 * @brief Signal delivery is unsupported on bare metal.
 * @param pid Requested process id.
 * @param sig Requested signal number.
 * @return Always `-1`.
 */
int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = ENOSYS;
    return -1;
}

/**
 * @brief Trap unexpected process exit requests.
 * @param status Exit status from newlib.
 */
void _exit(int status)
{
    (void)status;
    while (1)
    {
    }
}
