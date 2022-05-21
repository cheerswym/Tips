
#include "onboard/lite/transport/shm/lite_shm_header.h"

#include <iostream>
#include <memory>
#include <utility>

#include "gtest/gtest.h"

namespace qcraft {
namespace shm {
TEST(LiteShmHeaderTest, test) {
  LiteHeader header;
  header.set_seq_number(123);
  header.set_module_id(2);
  header.set_timestamp(2000);
  LiteShmHeader lite_shm_header("abcd", 22, header);
  size_t size = lite_shm_header.ByteSize();
  // std::cout << "size:" << size << std::endl;
  auto buf = new char[size * 2];
  EXPECT_TRUE(lite_shm_header.SerializeTo(buf));
  EXPECT_TRUE(lite_shm_header.SerializeTo(buf + size));

  LiteShmHeader lite_shm_header_1;
  EXPECT_TRUE(lite_shm_header_1.DeserializeFrom(buf));
  EXPECT_EQ(lite_shm_header_1.domain_channel(), "abcd");
  EXPECT_EQ(lite_shm_header_1.module_id(), 2);
  EXPECT_EQ(lite_shm_header_1.seq_num(), 123);
  EXPECT_EQ(lite_shm_header_1.tag_number(), 22);
  EXPECT_EQ(lite_shm_header_1.timestamp(), 2000);
  EXPECT_EQ(lite_shm_header_1.DebugString(),
            "Module_id=2 seq_num=123 tag_num=22 domain_channel=abcd");

  LiteShmHeader lite_shm_header_2;
  EXPECT_TRUE(lite_shm_header_2.DeserializeFrom(buf + size));
  EXPECT_EQ(lite_shm_header_2.domain_channel(), "abcd");
  EXPECT_EQ(lite_shm_header_2.module_id(), 2);
  EXPECT_EQ(lite_shm_header_2.seq_num(), 123);
  EXPECT_EQ(lite_shm_header_2.tag_number(), 22);
  EXPECT_EQ(lite_shm_header_2.timestamp(), 2000);
  EXPECT_EQ(lite_shm_header_2.DebugString(),
            "Module_id=2 seq_num=123 tag_num=22 domain_channel=abcd");
  EXPECT_EQ(lite_shm_header_1, lite_shm_header);
  EXPECT_EQ(lite_shm_header_1, lite_shm_header_2);

  LiteShmHeader lite_shm_header_3;
  lite_shm_header_3 = lite_shm_header_2;
  EXPECT_TRUE(lite_shm_header_3 == lite_shm_header_2);
  EXPECT_FALSE(lite_shm_header_3 != lite_shm_header_2);

  std::string str(1024, 'a');
  // not sure
  EXPECT_DEATH(LiteShmHeader lite_shm_header4(str, 22, header), "");

  delete[] buf;
}
}  // namespace shm
}  // namespace qcraft
