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

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/serial/ioss.h>

#include <stdio.h>
#include <termios.h>
#include <sys/ioctl.h>

int32_t osx_userial_open(const char* serial_port, uint32_t baud_rate, struct userial_port_t* port) {
  int fd = open(serial_port, O_RDWR | O_NONBLOCK);
  if (fd == -1) {
    printf("userial: couldn't open %s port!\n", serial_port);
    return 0;
  }

  struct termios options;
  tcgetattr(fd, &options);

  speed_t baud = baud_rate;
  switch(baud) {
    case 4800:    baud = B4800; break;
    case 9600:    baud = B9600; break;
#ifdef B14400
    case 14400:   baud = B14400; break;
#endif
    case 19200:   baud = B19200; break;
#ifdef B28800
    case 28800:   baud = B28800; break;
#endif
    case 38400:   baud = B38400; break;
    case 57600:   baud = B57600; break;
    case 115200:  baud = B115200; break;
  }

  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);

  if (ioctl(fd, IOSSIOSPEED, &baud, 1) == -1) {
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

  if (tcsetattr(fd, TCSANOW, &options) == -1) {
    printf("userial: failed to set port attributes!\n");
    return 0;
  }

  port->handle = fd;

  return 1;
}

int32_t osx_close(struct userial_port_t* port) {
  return close(port->handle);
}

int osx_write(struct userial_port_t* port, uint8_t data) {
  const int written_bytes = write(port->handle, &data, 1u);
  return (written_bytes == 1);
}

int osx_read(struct userial_port_t* port, uint8_t* data, size_t num_bytes) {
  return read(port->handle, data, num_bytes);
}

static struct userial_api_i g_userial_api = {
  .open = osx_userial_open,
  .close = osx_close,
  .write = osx_write,
  .read = osx_read,
};

int userial_create_api(struct userial_api_i** userial_api, uint32_t flags) {
  (*userial_api) = &g_userial_api;
  return 1;
}

#endif