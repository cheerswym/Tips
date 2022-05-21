#include "onboard/utils/objects_view.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace {

TEST(ObjectsViewTest, TestObjects) {
  ObjectsView objects_view;
  auto objects = std::make_shared<ObjectsProto>();
  auto* obj = objects->add_objects();
  obj->set_id("id1");
  obj->set_timestamp(100.0);
  obj = objects->add_objects();
  obj->set_id("id2");
  obj->set_timestamp(100.0);

  objects_view.UpdateObjects(ObjectsProto::SCOPE_REAL, objects);

  auto export_objects = objects_view.ExportAllObjectsProto();

  ASSERT_EQ(export_objects->objects_size(), 2);
  EXPECT_EQ(export_objects->objects(0).id(), "id1");
  EXPECT_EQ(export_objects->objects(0).timestamp(), 100.0);
  EXPECT_EQ(export_objects->objects(1).id(), "id2");

  // Add more objects.
  objects = std::make_shared<ObjectsProto>();
  obj = objects->add_objects();
  obj->set_id("id1");
  obj->set_timestamp(101.1);
  obj = objects->add_objects();
  obj->set_id("id3");
  obj->set_timestamp(102.0);
  objects_view.UpdateObjects(ObjectsProto::SCOPE_VIRTUAL, objects);

  export_objects = objects_view.ExportAllObjectsProto();
  ASSERT_EQ(export_objects->objects_size(), 4);
  EXPECT_EQ(export_objects->objects(0).id(), "id1");
  EXPECT_EQ(export_objects->objects(0).timestamp(), 100);
  EXPECT_EQ(export_objects->objects(1).id(), "id2");
  EXPECT_EQ(export_objects->objects(2).id(), "id1");
  EXPECT_EQ(export_objects->objects(3).id(), "id3");

  objects = std::make_shared<ObjectsProto>();
  obj = objects->add_objects();
  obj->set_id("id1");
  obj->set_timestamp(102);
  objects_view.UpdateObjects(ObjectsProto::SCOPE_REAL, objects);

  export_objects = objects_view.ExportAllObjectsProto();
  ASSERT_EQ(export_objects->objects_size(), 3);
  EXPECT_EQ(export_objects->objects(0).id(), "id1");
  EXPECT_EQ(export_objects->objects(0).timestamp(), 102);
  EXPECT_EQ(export_objects->objects(1).id(), "id1");
  EXPECT_EQ(export_objects->objects(2).id(), "id3");

  export_objects =
      objects_view.ExportObjectsProtoByScope(ObjectsProto::SCOPE_VIRTUAL);
  ASSERT_EQ(export_objects->objects_size(), 2);
  EXPECT_EQ(export_objects->objects(0).id(), "id1");
  EXPECT_EQ(export_objects->objects(1).id(), "id3");
}

}  // namespace
}  // namespace qcraft
