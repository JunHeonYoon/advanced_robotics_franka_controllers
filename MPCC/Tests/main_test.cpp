#include "gtest/gtest.h"
#include "model_integrator_test.h"
#include "robot_model_test.h"
#include "self_collision_test.h"
#include "spline_test.h"
#include "constraints_test.h"
#include "cost_test.h"

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}