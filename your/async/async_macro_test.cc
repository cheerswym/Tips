#include "onboard/async/async_macro.h"

#include "gtest/gtest.h"

namespace qcraft::async {

TEST(AsyncMarcoTest, MoveDestroyContainer) {
  int i = 1;
  MOVE_DESTROY_CONTAINER_ASYNC_DISPOSAL(i);
}

}  // namespace qcraft::async
