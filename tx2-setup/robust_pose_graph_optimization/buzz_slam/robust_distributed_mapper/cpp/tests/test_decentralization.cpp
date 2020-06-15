/**
 * @file test_decentralization.cpp
 * @author Pierre-Yves Lajoie (lajoie.py@gmail.com)
 * @brief unit tests for decentralization of the multi robot case simulated by loading the datasets
 */
#include <distributed_mapper/run_distributed_mapper.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>
#include <cmath>

using namespace std;
using namespace gtsam;
using namespace distributed_mapper;
using namespace evaluation_utils;

/******************************************************************************/

TEST(DistributedMapper, test_decentralized_distributed_estimation_without_outliers_2robots) {
    // Parameters

    // Call decentralized optimization

    // Check if the test worked
}

/****************************************************************************** */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
//******************************************************************************
