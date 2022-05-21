// Copyright @2021 QCraft AI Inc. All rights reserved.
#include <iostream>

#include "onboard/global/init_qcraft.h"
#include "onboard/utils/id_generator.h"

int main(int argc, char* argv[]) {
  qcraft::InitQCraft(&argc, &argv);
  std::cout << qcraft::GetUId();
  return 0;
}
