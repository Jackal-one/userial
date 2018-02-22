#pragma once

#include <inttypes.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

struct userial_port_t {
  uint32_t handle;
};

struct userial_api_i {
  int (*open)(const char* serial_port, uint32_t baud_rate, struct userial_port_t* port);
  int (*close)(struct userial_port_t* port);
  int (*write)(struct userial_port_t* port, uint8_t data);
  int (*read)(struct userial_port_t* port, uint8_t* data, size_t num_bytes);
};

int userial_create_api(struct userial_api_i** userial_api, uint32_t flags);

#ifdef __cplusplus
}
#endif

#ifdef USERIAL_IMPL

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
  if (fd != 0) {
    printf("userial: couldn't open %s port!\n", serial_port);
    return 0;
  }

  struct termios options;
  tcgetattr(fd, &options);

  speed_t baud = select_baud_rate(baud_rate);
  
  if (cfsetispeed(&options, baud) != 0) {
    printf("userial: failed to set baud rate of %d!\n", baud_rate);
    return 0;
  }

  if (cfsetospeed(&options, baud) != 0) {
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

  if (tcsetattr(fd, TCSANOW, &options) != 0) {
    printf("userial: failed to set port attributes!\n");
    return 0;
  }

  port->handle = fd;

  return 1;
}

int32_t posix_close(struct userial_port_t* port) {
  return close(port->handle);
}

int posix_write(struct userial_port_t* port, uint8_t data) {
  const int written_bytes = write(port->handle, &data, 1u);
  return (written_bytes == 1);
}

int posix_read(struct userial_port_t* port, uint8_t* data, size_t num_bytes) {
  return read(port->handle, data, num_bytes);
}

static struct userial_api_i g_userial_api = {
  .open = posix_userial_open,
  .close = posix_close,
  .write = posix_write,
  .read = posix_read,
};

int userial_create_api(struct userial_api_i** userial_api, uint32_t flags) {
  (*userial_api) = &g_userial_api;
  return 1;
}

#endif