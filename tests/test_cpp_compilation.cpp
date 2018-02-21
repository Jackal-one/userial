#define USERIAL_IMPL
#include "userial.h"

int main(int argc, char** argv) {
  struct userial_api_i* serial;
  userial_create_api(&serial, 0u);

  return 0;
}