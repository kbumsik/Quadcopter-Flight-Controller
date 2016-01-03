/**
*****************************************************************************
**
**  File        : syscalls.c
**
**  Abstract    : Atollic TrueSTUDIO Minimal System calls file
**
**              For more information about which c-functions
**                need which of these lowlevel functions
**                please consult the Newlib libc-manual
**
**  Environment : Atollic TrueSTUDIO
**
**  Distribution: The file is distributed "as is", without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. This file may only be built (assembled or compiled and linked)
**  using the Atollic TrueSTUDIO(R) product. The use of this file together
**  with other tools than Atollic TrueSTUDIO(R) is not permitted.
**
*****************************************************************************
*/

/* Includes */
#include <stdint.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>

/* Custom Includes */
#include "config.h"

/* Variables */
#undef errno
extern int errno;

uint8_t *__env[1] = { 0 };
uint8_t **environ = __env;

/* Global variables */
/* They are located in stm32f4xx_it.c */
UART_HandleTypeDef xUARTHandle;
/* They are located in main.cpp */
QueueHandle_t qUARTReceive;


/* Functions */
void initialise_monitor_handles()
{
}

int _getpid(void)
{
  return 1;
}

int _kill(int32_t pid, int32_t sig)
{
  errno = EINVAL;
  return -1;
}

void _exit (int32_t status)
{
  _kill(status, -1);
  while (1) {}    /* Make sure we hang here */
}

/**
 * One of the newlib system call function used for printf(). In this project
 * ::_write() is implemented using HAL's ::HAL_UART_Transmit().
 * @param file
 * @param ptr
 * @param len
 * @return
 */
int _write(int32_t file, uint8_t *ptr, int32_t len) {
  HAL_StatusTypeDef result;
  result = HAL_UART_Transmit(&xUARTHandle, ptr, (uint16_t)len, 500);
  if (result == HAL_OK)
    {
      return len;
    }
  else
    {
      return 0;
    }
}

caddr_t _sbrk(int32_t incr)
{
  extern uint32_t _Min_Heap_Size; /* _Min_Heap_Size symbol defined in the
                                     linker script. */
  extern uint8_t end asm("end");
  const uint8_t *max_heap = (uint8_t*)((uint32_t)&end + (uint32_t)&_Min_Heap_Size);
  static uint8_t *heap_end;
  uint8_t *prev_heap_end;

  if (heap_end == 0)
    heap_end = &end;

  prev_heap_end = heap_end;
  if (heap_end + incr > max_heap)
  {
//    write(1, "Heap and stack collision\n", 25);
//    abort();
    errno = ENOMEM;
    return (caddr_t) -1;
  }

  heap_end += incr;

  return (caddr_t) prev_heap_end;
}

int _close(int32_t file)
{
  return -1;
}


int _fstat(int32_t file, struct stat *st)
{
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int32_t file)
{
  return 1;
}

int _lseek(int32_t file, int32_t ptr, int32_t dir)
{
  return 0;
}
/**
 * One of the newlib system call function used for scanf(). In this project
 * ::_read() is implemented using FreeRTOS's ::xQeueReceive().
 * @param file
 * @param ptr
 * @param len
 * @return number of characters received.
 */
int _read(int32_t file, uint8_t *ptr, int32_t len) {
    int n = 0;
    char buffer_input[confUART_RECEIVE_BUFFER_SIZE];

    switch (file)
    {
        case 0: //stdin
          xQueueReceive(qUARTReceive, (void*) buffer_input, portMAX_DELAY);
          /* Find the number of character */
          for (n = 0; n < confUART_RECEIVE_BUFFER_SIZE; n++)
          {
            *ptr++ = buffer_input[n];
            if( buffer_input[n] == '\0')
            {
              break;
            }
          }
        break;
    default:
        return 0;
    }
    return n;
}

int _open(uint8_t *path, int32_t flags, int32_t mode)
{
  /* Pretend like we always fail */
  return -1;
}

int _wait(int32_t *status)
{
  errno = ECHILD;
  return -1;
}

int _unlink(const uint8_t *name)
{
  errno = ENOENT;
  return -1;
}

int _times(struct tms *buf)
{
  return -1;
}

int _stat(const uint8_t *file, struct stat *st)
{
  st->st_mode = S_IFCHR;
  return 0;
}

int _link(const uint8_t *old, const uint8_t *new)
{
  errno = EMLINK;
  return -1;
}

int _fork(void)
{
  errno = EAGAIN;
  return -1;
}

int _execve(const uint8_t *name, uint8_t * const *argv, uint8_t * const *env)
{
  errno = ENOMEM;
  return -1;
}
