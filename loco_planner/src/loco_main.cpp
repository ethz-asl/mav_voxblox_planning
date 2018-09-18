#include "loco_planner/loco.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  loco_planner::Loco<10> loco(3);

  return 0;
}
