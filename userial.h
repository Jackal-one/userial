#pragma once

#include <inttypes.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

struct userial_port_t {
  uint64_t handle;
};

struct userial_api_i {
  int (*open)(const char* serial_port, uint32_t baud_rate, struct userial_port_t* port);
  int (*close)(struct userial_port_t* port);
  int (*write_byte)(struct userial_port_t* port, uint8_t data);
  int (*write)(struct userial_port_t* port, uint8_t* data, size_t num_bytes);
  int (*read)(struct userial_port_t* port, uint8_t* data, size_t num_bytes);
};

int userial_create_api(struct userial_api_i** userial_api, uint32_t flags);

#ifdef __cplusplus
}
#endif

#ifdef USERIAL_IMPLEMENTATION

#if defined(__unix__)
  #define USERIAL_PLATFORM_POSIX
#elif defined(__APPLE__) && defined(__MACH__)
  #include "TargetConditionals.h"
  #if TARGET_OS_MAC == 1
    #define USERIAL_PLATFORM_POSIX
  #endif
#elif defined(_WIN32)
  #define USERIAL_PLATFORM_WINDOWS
#endif

#if defined(USERIAL_PLATFORM_POSIX)

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h> 
#include <termios.h>
#include <sys/ioctl.h>

speed_t select_baud_rate(uint32_t baud_rate) {
  switch(baud_rate) {
    case 4800: return B4800;
    case 9600: return B9600;
  #ifdef B14400
    case 14400: return B14400;
  #endif
    case 19200: return B19200;
  #ifdef B28800
    case 28800: return B28800;
  #endif
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
  }

  return baud_rate;
}

int32_t posix_userial_open(const char* serial_port, uint32_t baud_rate, struct userial_port_t* port) {
  int fd = open(serial_port, O_RDWR | O_NONBLOCK);
  if (fd < 0) {
    printf("userial: couldn't open %s port!\n", serial_port);
    return 0;
  }

  struct termios options;
  tcgetattr(fd, &options);

  speed_t baud = select_baud_rate(baud_rate);
  
  if (cfsetispeed(&options, baud) < 0) {
    printf("userial: failed to set baud rate of %d!\n", baud_rate);
    return 0;
  }

  if (cfsetospeed(&options, baud) < 0) {
     printf("userial: failed to set baud rate of %d!\n", baud_rate);
    return 0;   
  }

  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);
  options.c_oflag &= ~(OPOST);
  options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IGNBRK | INPCK | ISTRIP);
  options.c_cflag &= ~(CSIZE | CSTOPB | PARENB | PARODD);
  options.c_cflag |= (CLOCAL | CREAD | CS8);

  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &options) < 0) {
    printf("userial: failed to set port attributes!\n");
    return 0;
  }

  port->handle = fd;

  return 1;
}

int32_t posix_close(struct userial_port_t* port) {
  return close(port->handle);
}

int posix_write_byte(struct userial_port_t* port, uint8_t data) {
  const int written_bytes = write(port->handle, &data, 1u);
  return (written_bytes == 1);
}

int posix_write_bytes(struct userial_port_t* port, uint8_t* data, size_t num_bytes) {
  int written_bytes = 0;
  while (written_bytes < num_bytes) {
    const size_t remaining_bytes = num_bytes - written_bytes;
    const int ret = write(port->handle, &data[written_bytes], remaining_bytes);
    
    if (ret == -1) {
      return -1;
    }

    written_bytes += ret;
  }

  return written_bytes;
}

int posix_read(struct userial_port_t* port, uint8_t* data, size_t num_bytes) {
  return read(port->handle, data, num_bytes);
}

static struct userial_api_i g_userial_api = {
  .open = posix_userial_open,
  .close = posix_close,
  .write_byte = posix_write_byte,
  .write = posix_write_bytes,
  .read = posix_read,
};

#elif defined(USERIAL_PLATFORM_WINDOWS)

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdio.h>

int32_t win32_userial_open(const char* serial_port, uint32_t baud_rate, struct userial_port_t* port) {
  HANDLE handle = CreateFile(serial_port,
    GENERIC_READ | GENERIC_WRITE,
    0, NULL, OPEN_EXISTING, 0, NULL);

  if (handle == INVALID_HANDLE_VALUE) {
    printf("userial: couldn't open %s port!\n", serial_port);
    return 0;
  }

  DCB dcb_params = { 0 };
  dcb_params.DCBlength = sizeof(dcb_params);

  GetCommState(handle, &dcb_params);
  dcb_params.BaudRate = baud_rate;
  dcb_params.ByteSize = 8u;
  dcb_params.StopBits = ONESTOPBIT;
  dcb_params.Parity = NOPARITY;
  dcb_params.fDtrControl = DTR_CONTROL_ENABLE;

  if (!SetCommState(handle, &dcb_params)) {
    printf("userial: failed to configure port!\n");
    return 0;
  }

  PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
  port->handle = (uint64_t)handle;

  return 1;
}

int32_t win32_close(struct userial_port_t* port) {
  return CloseHandle((HANDLE)port->handle);
}

int win32_write_byte(struct userial_port_t* port, uint8_t data) {
  DWORD written_bytes = 0u;
  WriteFile((HANDLE)port->handle, &data, 1u, &written_bytes, NULL);
  return (written_bytes == 1);
}

int win32_write_bytes(struct userial_port_t* port, uint8_t* data, size_t num_bytes) {
  size_t total_written_bytes = 0;
  while (total_written_bytes < num_bytes) {
    const size_t remaining_bytes = num_bytes - total_written_bytes;
    DWORD written_bytes = 0u;

    if (!WriteFile((HANDLE)port->handle, &data[written_bytes],
      remaining_bytes, &written_bytes, NULL)) {
      return -1;
    }
    
    total_written_bytes += written_bytes;
  }

  return total_written_bytes;
}

int win32_read(struct userial_port_t* port, uint8_t* data, size_t num_bytes) {
  DWORD read_bytes = 0u;
  ReadFile((HANDLE)port->handle, data, num_bytes, &read_bytes, NULL);
  return read_bytes;
}

static struct userial_api_i g_userial_api = {
  win32_userial_open,
  win32_close,
  win32_write_byte,
  win32_write_bytes,
  win32_read,
};

#endif

int userial_create_api(struct userial_api_i** userial_api, uint32_t flags) {
  (*userial_api) = &g_userial_api;
  return 1;
}

#endif