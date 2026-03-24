#include <gtest/gtest.h>
#include "FrontEnd.hpp"

TEST(FrontEndTest, Test){
    SlamFrontEnd fe;
    fe.Start();
    fe.Join();

}