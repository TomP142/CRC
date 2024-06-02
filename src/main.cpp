#include <functions.h>

void setup() {
  startUp();
}

void loop() {
  update();
  if (status > 2 && status < 6) {
    dataLog();
  }
}
