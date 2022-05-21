#include "gtest/gtest.h"
#include "onboard/global/init_qcraft.h"

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  qcraft::InitQCraft(&argc, &argv);
  return RUN_ALL_TESTS();
}
