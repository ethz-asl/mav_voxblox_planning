#include "loco/loco.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  loco::Loco<10> loco(3);

  return 0;
}
