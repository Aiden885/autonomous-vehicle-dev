
#include <unistd.h>
#include <chrono>
#include "gtest/gtest.h"

#include "modules/examples/module_empty/src/module_empty.h"

#include "perception_obstacles.pb.h"

TEST(module_empty_Init, test_init) {
    pangu::modules::module_empty demo{"../../configs/module_configs/module_empty/"};
    //EXPECT_FALSE(demo.Init("./test_data/fake.pt"));
    EXPECT_TRUE(demo.Init("../../configs/module_configs/module_empty/module_empty.pt"));
    demo.Proc();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    pangu::common::MsgNode::Instance()->Stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    SETLOGLEVEL(DEBUG);
    SETCONFIG(root_folder, "../../");
    SETCONFIG(tf_pt, "../../configs/global_configs/tf.pt");
    MSG_TRANSFORM_INIT("ControlData_test", argc, argv,
                       "../../configs/global_configs/preseted_channels.pt");
    pangu::common::MsgNode::Instance();
    return RUN_ALL_TESTS();
}