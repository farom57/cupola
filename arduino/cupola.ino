#include "cupola.h"

int main(void)
{
  init();
  initVariant();

#if defined(SERIAL_CDC)
  PluggableUSBD().begin();
  SerialUSB.begin(115200);
#endif

  setup();

  for (;;) {
    loop();
    if (arduino::serialEventRun) arduino::serialEventRun();
  }

  return 0;
}
