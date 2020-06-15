/**
 * @file test_robust_distributed_mapper.cpp
 * @author Pierre-Yves Lajoie (lajoie.py@gmail.com)
 * @brief unit tests for test multi robot case with outliers simulated by loading the datasets
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

TEST(DistributedMapper, test_distributed_estimation_without_outliers_2robots) {
    // Parameters
    size_t nr_robots = 2; // number of robots
    string log_dir("/tmp/"); // log directory
    string data_dir("../../../test_data/pairwise_consistency_maximization/clean/simulation/example_2robots/"); // data directory
    string trace_file("/tmp/runG2o"); // data directory
    bool use_XY = false;
    bool use_OP = false;
    bool debug = false;
    noiseModel::Diagonal::shared_ptr prior_model = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
    size_t max_iter = 1000; // Maximum number of iterations of optimizer
    double rotation_estimate_change_threshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
    double pose_estimate_change_threshold = 1e-1; // Difference between pose estimate provides an early stopping condition
    double gamma = 1.0f; // Gamma value for over relaxation methods
    bool use_flagged_init = true; // to use flagged initialization or not
    distributed_mapper::DistributedMapper::UpdateType update_type = distributed_mapper::DistributedMapper::incUpdate; // update_type differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
    bool use_between_noise = false; // use between factor noise or not
    bool use_chr_less_full_graph = false; // whether full graph has character indexes or not
    bool use_landmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
    double pcm_threshold = 0.90; // Confidence probability for the pairwise consistency computation
    bool use_covariance = false; // use covariance in dataset file.
    bool use_PCM = true; // Use pairwise consistency maximization.

    // Call distributed optimization
    std::tuple<double, double, int> results = runDistributedMapper(nr_robots, log_dir, data_dir, trace_file,
                                                                   use_XY, use_OP, debug, prior_model, model,
                                                                   max_iter, rotation_estimate_change_threshold, pose_estimate_change_threshold,
                                                                   gamma, use_flagged_init, update_type, use_between_noise,
                                                                   use_chr_less_full_graph, use_landmarks, pcm_threshold, use_covariance, use_PCM, false);
    // Compare centralized and distributed pose estimates
    double tolerance = 1e-1;
    EXPECT(assert_equal(0.0, std::get<0>(results), tolerance));
    EXPECT(assert_equal(std::get<0>(results), std::get<1>(results), tolerance));
    EXPECT(std::get<2>(results) == 10);
}

TEST(DistributedMapper, test_distributed_estimation_2robots) {
    // Parameters
    size_t nr_robots = 2; // number of robots
    string log_dir("/tmp/"); // log directory
    string data_dir("../../../test_data/pairwise_consistency_maximization/spoiled/simulation/example_2robots/"); // data directory
    string trace_file("/tmp/runG2o"); // data directory
    bool use_XY = false;
    bool use_OP = false;
    bool debug = false;
    noiseModel::Diagonal::shared_ptr prior_model = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
    size_t max_iter = 1000; // Maximum number of iterations of optimizer
    double rotation_estimate_change_threshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
    double pose_estimate_change_threshold = 1e-1; // Difference between pose estimate provides an early stopping condition
    double gamma = 1.0f; // Gamma value for over relaxation methods
    bool use_flagged_init = true; // to use flagged initialization or not
    distributed_mapper::DistributedMapper::UpdateType update_type = distributed_mapper::DistributedMapper::incUpdate; // update_type differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
    bool use_between_noise = false; // use between factor noise or not
    bool use_chr_less_full_graph = false; // whether full graph has character indexes or not
    bool use_landmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
    double pcm_threshold = 0.90; // Confidence probability for the pairwise consistency computation
    bool use_covariance = false; // use covariance in dataset file.
    bool use_PCM = true; // Use pairwise consistency maximization.
    // Call distributed optimization
    std::tuple<double, double, int> results = runDistributedMapper(nr_robots, log_dir, data_dir, trace_file,
                                                                   use_XY, use_OP, debug, prior_model, model,
                                                                   max_iter, rotation_estimate_change_threshold, pose_estimate_change_threshold,
                                                                   gamma, use_flagged_init, update_type, use_between_noise,
                                                                   use_chr_less_full_graph, use_landmarks, pcm_threshold, use_covariance, use_PCM, false);
    // Compare centralized and distributed pose estimates
    double tolerance = 2e-1;
    EXPECT(assert_equal(0.0, std::get<0>(results), tolerance));
    EXPECT(assert_equal(std::get<0>(results), std::get<1>(results), tolerance));
    EXPECT(std::get<2>(results) <= 10);
}

/* Test where the loop closure measurements are mixed with the odometry measurements
 * and not in the same order in the 2 files as we can expect in a asynchronous system */
TEST(DistributedMapper, test_distributed_estimation_not_in_order_2robots) {
    // Parameters
    size_t nr_robots = 2; // number of robots
    string log_dir("/tmp/"); // log directory
    string data_dir("../../../test_data/pairwise_consistency_maximization/spoiled/simulation_not_in_order/example_2robots/"); // data directory
    string trace_file("/tmp/runG2o"); // data directory
    bool use_XY = false;
    bool use_OP = false;
    bool debug = false;
    noiseModel::Diagonal::shared_ptr prior_model = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
    size_t max_iter = 1000; // Maximum number of iterations of optimizer
    double rotation_estimate_change_threshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
    double pose_estimate_change_threshold = 1e-1; // Difference between pose estimate provides an early stopping condition
    double gamma = 1.0f; // Gamma value for over relaxation methods
    bool use_flagged_init = true; // to use flagged initialization or not
    distributed_mapper::DistributedMapper::UpdateType update_type = distributed_mapper::DistributedMapper::incUpdate; // update_type differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
    bool use_between_noise = false; // use between factor noise or not
    bool use_chr_less_full_graph = false; // whether full graph has character indexes or not
    bool use_landmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
    double pcm_threshold = 0.90; // Confidence probability for the pairwise consistency computation
    bool use_covariance = false; // use covariance in dataset file.
    bool use_PCM = true; // Use pairwise consistency maximization.
    // Call distributed optimization
    std::tuple<double, double, int> results = runDistributedMapper(nr_robots, log_dir, data_dir, trace_file,
                                                                   use_XY, use_OP, debug, prior_model, model,
                                                                   max_iter, rotation_estimate_change_threshold, pose_estimate_change_threshold,
                                                                   gamma, use_flagged_init, update_type, use_between_noise,
                                                                   use_chr_less_full_graph, use_landmarks, pcm_threshold, use_covariance, use_PCM, false);
    // Compare centralized and distributed pose estimates
    double tolerance = 2e-1;
    EXPECT(assert_equal(0.0, std::get<0>(results), tolerance));
    EXPECT(assert_equal(std::get<0>(results), std::get<1>(results), tolerance));
    EXPECT(std::get<2>(results) == 10);
}

TEST(DistributedMapper, test_distributed_estimation_no_PCM_2robots) {
    // Parameters
    size_t nr_robots = 2; // number of robots
    string log_dir("/tmp/"); // log directory
    string data_dir("../../../test_data/pairwise_consistency_maximization/spoiled/simulation/example_2robots/"); // data directory
    string trace_file("/tmp/runG2o"); // data directory
    bool use_XY = false;
    bool use_OP = false;
    bool debug = false;
    noiseModel::Diagonal::shared_ptr prior_model = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
    size_t max_iter = 1000; // Maximum number of iterations of optimizer
    double rotation_estimate_change_threshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
    double pose_estimate_change_threshold = 1e-1; // Difference between pose estimate provides an early stopping condition
    double gamma = 1.0f; // Gamma value for over relaxation methods
    bool use_flagged_init = true; // to use flagged initialization or not
    distributed_mapper::DistributedMapper::UpdateType update_type = distributed_mapper::DistributedMapper::incUpdate; // update_type differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
    bool use_between_noise = false; // use between factor noise or not
    bool use_chr_less_full_graph = false; // whether full graph has character indexes or not
    bool use_landmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
    double pcm_threshold = 0.90; // Confidence probability for the pairwise consistency computation
    bool use_covariance = false; // use covariance in dataset file.
    bool use_PCM = false; // Use pairwise consistency maximization.
    // Call distributed optimization
    std::tuple<double, double, int> results = runDistributedMapper(nr_robots, log_dir, data_dir, trace_file,
                                                                   use_XY, use_OP, debug, prior_model, model,
                                                                   max_iter, rotation_estimate_change_threshold, pose_estimate_change_threshold,
                                                                   gamma, use_flagged_init, update_type, use_between_noise,
                                                                   use_chr_less_full_graph, use_landmarks, pcm_threshold, use_covariance, use_PCM, false);
    // Compare centralized and distributed pose estimates
    double tolerance = 1e-1;
    EXPECT(assert_equal(27.8, std::get<0>(results), tolerance));
    EXPECT(assert_equal(27.8, std::get<1>(results), tolerance));
    EXPECT(std::get<2>(results) == 0);
}

TEST(DistributedMapper, test_distributed_estimation_4robots) {
    // Parameters
    size_t nr_robots = 4; // number of robots
    string log_dir("/tmp/"); // log directory
    string data_dir("../../../test_data/pairwise_consistency_maximization/spoiled/simulation/example_4robots/"); // data directory
    string trace_file("/tmp/runG2o"); // data directory
    bool use_XY = false;
    bool use_OP = false;
    bool debug = false;
    noiseModel::Diagonal::shared_ptr prior_model = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
    size_t max_iter = 1000; // Maximum number of iterations of optimizer
    double rotation_estimate_change_threshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
    double pose_estimate_change_threshold = 1e-1; // Difference between pose estimate provides an early stopping condition
    double gamma = 1.0f; // Gamma value for over relaxation methods
    bool use_flagged_init = true; // to use flagged initialization or not
    distributed_mapper::DistributedMapper::UpdateType update_type = distributed_mapper::DistributedMapper::incUpdate; // update_type differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
    bool use_between_noise = false; // use between factor noise or not
    bool use_chr_less_full_graph = false; // whether full graph has character indexes or not
    bool use_landmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
    double pcm_threshold = 0.90; // Confidence probability for the pairwise consistency computation
    bool use_covariance = false; // use covariance in dataset file.
    bool use_PCM = true; // Use pairwise consistency maximization.
    // Call distributed optimization
    std::tuple<double, double, int> results = runDistributedMapper(nr_robots, log_dir, data_dir, trace_file,
                                                                   use_XY, use_OP, debug, prior_model, model,
                                                                   max_iter, rotation_estimate_change_threshold, pose_estimate_change_threshold,
                                                                   gamma, use_flagged_init, update_type, use_between_noise,
                                                                   use_chr_less_full_graph, use_landmarks, pcm_threshold, use_covariance, use_PCM, false);
    // Compare centralized and distributed pose estimates
    double tolerance = 1e-1;
    EXPECT(assert_equal(0.0, std::get<0>(results), tolerance));
    EXPECT(assert_equal(std::get<0>(results), std::get<1>(results), tolerance));
    EXPECT(std::get<2>(results) == 24);
}

TEST(DistributedMapper, test_distributed_estimation_8robots) {
    // Parameters
    size_t nr_robots = 8; // number of robots
    size_t nr_separators_by_pair = 4; // number of separators by pair of robots
    string log_dir("/tmp/"); // log directory
    string data_dir("../../../test_data/pairwise_consistency_maximization/spoiled/simulation/example_8robots/"); // data directory
    string trace_file("/tmp/runG2o"); // data directory
    bool use_XY = false;
    bool use_OP = false;
    bool debug = false;
    noiseModel::Diagonal::shared_ptr prior_model = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
    size_t max_iter = 1000; // Maximum number of iterations of optimizer
    double rotation_estimate_change_threshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
    double pose_estimate_change_threshold = 1e-1; // Difference between pose estimate provides an early stopping condition
    double gamma = 1.0f; // Gamma value for over relaxation methods
    bool use_flagged_init = true; // to use flagged initialization or not
    distributed_mapper::DistributedMapper::UpdateType update_type = distributed_mapper::DistributedMapper::incUpdate; // update_type differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
    bool use_between_noise = false; // use between factor noise or not
    bool use_chr_less_full_graph = false; // whether full graph has character indexes or not
    bool use_landmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
    double pcm_threshold = 0.90; // Confidence probability for the pairwise consistency computation
    bool use_covariance = false; // use covariance in dataset file.
    bool use_PCM = true; // Use pairwise consistency maximization.
    // Call distributed optimization
    std::tuple<double, double, int> results = runDistributedMapper(nr_robots, log_dir, data_dir, trace_file,
                                                                   use_XY, use_OP, debug, prior_model, model,
                                                                   max_iter, rotation_estimate_change_threshold, pose_estimate_change_threshold,
                                                                   gamma, use_flagged_init, update_type, use_between_noise,
                                                                   use_chr_less_full_graph, use_landmarks, pcm_threshold, use_covariance, use_PCM, false);
    // Compare centralized and distributed pose estimates
    double tolerance = 1e-1;
    EXPECT(assert_equal(std::get<0>(results), std::get<1>(results), tolerance));
    EXPECT(std::get<2>(results) == (nr_robots-1)*nr_robots*nr_separators_by_pair/2);
}


TEST(DistributedMapper, test_distributed_estimation_16robots) {
    // Parameters
    size_t nr_robots = 16; // number of robots
    size_t nr_separators_by_pair = 4; // number of separators by pair of robots
    string log_dir("/tmp/"); // log directory
    string data_dir("../../../test_data/pairwise_consistency_maximization/spoiled/simulation/example_16robots/"); // data directory
    string trace_file("/tmp/runG2o"); // data directory
    bool use_XY = false;
    bool use_OP = false;
    bool debug = false;
    noiseModel::Diagonal::shared_ptr prior_model = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
    size_t max_iter = 1000; // Maximum number of iterations of optimizer
    double rotation_estimate_change_threshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
    double pose_estimate_change_threshold = 1e-1; // Difference between pose estimate provides an early stopping condition
    double gamma = 1.0f; // Gamma value for over relaxation methods
    bool use_flagged_init = true; // to use flagged initialization or not
    distributed_mapper::DistributedMapper::UpdateType update_type = distributed_mapper::DistributedMapper::incUpdate; // update_type differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
    bool use_between_noise = false; // use between factor noise or not
    bool use_chr_less_full_graph = false; // whether full graph has character indexes or not
    bool use_landmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
    double pcm_threshold = 0.90; // Confidence probability for the pairwise consistency computation
    bool use_covariance = false; // use covariance in dataset file.
    bool use_PCM = true; // Use pairwise consistency maximization.
    // Call distributed optimization
    std::tuple<double, double, int> results = runDistributedMapper(nr_robots, log_dir, data_dir, trace_file,
                                                                   use_XY, use_OP, debug, prior_model, model,
                                                                   max_iter, rotation_estimate_change_threshold, pose_estimate_change_threshold,
                                                                   gamma, use_flagged_init, update_type, use_between_noise,
                                                                   use_chr_less_full_graph, use_landmarks, pcm_threshold, use_covariance, use_PCM, false);
    // Compare centralized and distributed pose estimates
    double tolerance = 2e0;
    EXPECT(assert_equal(std::get<0>(results), std::get<1>(results), tolerance));
    EXPECT(std::abs((int)(std::get<2>(results)-(nr_robots-1)*nr_robots*nr_separators_by_pair/2)) < 5);
}

TEST(DistributedMapper, test_distributed_estimation_no_outliers_argos_2robots) {
    // Parameters
    size_t nr_robots = 2; // number of robots
    // size_t nr_separators_by_pair = 4; // number of separators by pair of robots
    string log_dir("/tmp/"); // log directory
    string data_dir("../../../test_data/argos_simulation/clean/example_2robots/"); // data directory
    string trace_file("/tmp/runG2o"); // data directory
    bool use_XY = false;
    bool use_OP = false;
    bool debug = false;
    noiseModel::Diagonal::shared_ptr prior_model = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
    size_t max_iter = 1000; // Maximum number of iterations of optimizer
    double rotation_estimate_change_threshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
    double pose_estimate_change_threshold = 1e-1; // Difference between pose estimate provides an early stopping condition
    double gamma = 1.0f; // Gamma value for over relaxation methods
    bool use_flagged_init = true; // to use flagged initialization or not
    distributed_mapper::DistributedMapper::UpdateType update_type = distributed_mapper::DistributedMapper::incUpdate; // update_type differentiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
    bool use_between_noise = false; // use between factor noise or not
    bool use_chr_less_full_graph = false; // whether full graph has character indexes or not
    bool use_landmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
    double pcm_threshold = 0.99; // Confidence probability for the pairwise consistency computation
    bool use_covariance = false; // use covariance in dataset file.
    bool use_PCM = false; // Use pairwise consistency maximization.
    // Call distributed optimization
    std::tuple<double, double, int> results = runDistributedMapper(nr_robots, log_dir, data_dir, trace_file,
                                                                   use_XY, use_OP, debug, prior_model, model,
                                                                   max_iter, rotation_estimate_change_threshold, pose_estimate_change_threshold,
                                                                   gamma, use_flagged_init, update_type, use_between_noise,
                                                                   use_chr_less_full_graph, use_landmarks, pcm_threshold, use_covariance, use_PCM, false);
    // Compare centralized and distributed pose estimates
    double tolerance = 1e-1;
    EXPECT(assert_equal(std::get<0>(results), std::get<1>(results), tolerance));
}

TEST(DistributedMapper, test_distributed_estimation_no_outliers_argos_4robots) {
    // Parameters
    size_t nr_robots = 4; // number of robots
    string log_dir("/tmp/"); // log directory
    string data_dir("../../../test_data/argos_simulation/clean/example_4robots/"); // data directory
    string trace_file("/tmp/runG2o"); // data directory
    bool use_XY = false;
    bool use_OP = false;
    bool debug = false;
    noiseModel::Diagonal::shared_ptr prior_model = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
    size_t max_iter = 1000; // Maximum number of iterations of optimizer
    double rotation_estimate_change_threshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
    double pose_estimate_change_threshold = 1e-1; // Difference between pose estimate provides an early stopping condition
    double gamma = 1.0f; // Gamma value for over relaxation methods
    bool use_flagged_init = true; // to use flagged initialization or not
    distributed_mapper::DistributedMapper::UpdateType update_type = distributed_mapper::DistributedMapper::incUpdate; // update_type differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
    bool use_between_noise = false; // use between factor noise or not
    bool use_chr_less_full_graph = false; // whether full graph has character indexes or not
    bool use_landmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
    double pcm_threshold = 0.99; // Confidence probability for the pairwise consistency computation
    bool use_covariance = false; // use covariance in dataset file.
    bool use_PCM = false; // Use pairwise consistency maximization.
    // Call distributed optimization
    std::tuple<double, double, int> results = runDistributedMapper(nr_robots, log_dir, data_dir, trace_file,
                                                                   use_XY, use_OP, debug, prior_model, model,
                                                                   max_iter, rotation_estimate_change_threshold, pose_estimate_change_threshold,
                                                                   gamma, use_flagged_init, update_type, use_between_noise,
                                                                   use_chr_less_full_graph, use_landmarks, pcm_threshold, use_covariance, use_PCM, false);
    // Compare centralized and distributed pose estimates
    double tolerance = 5e-1;
    EXPECT(assert_equal(std::get<0>(results), std::get<1>(results), tolerance*std::get<0>(results)));
}


TEST(DistributedMapper, test_distributed_estimation_no_outliers_argos_10robots) {
    // Parameters
    size_t nr_robots = 10; // number of robots
    string log_dir("/tmp/"); // log directory
    string data_dir("../../../test_data/argos_simulation/clean/example_10robots/"); // data directory
    string trace_file("/tmp/runG2o"); // data directory
    bool use_XY = false;
    bool use_OP = false;
    bool debug = false;
    noiseModel::Diagonal::shared_ptr prior_model = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
    size_t max_iter = 1000; // Maximum number of iterations of optimizer
    double rotation_estimate_change_threshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
    double pose_estimate_change_threshold = 1e-1; // Difference between pose estimate provides an early stopping condition
    double gamma = 1.0f; // Gamma value for over relaxation methods
    bool use_flagged_init = true; // to use flagged initialization or not
    distributed_mapper::DistributedMapper::UpdateType update_type = distributed_mapper::DistributedMapper::incUpdate; // update_type differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
    bool use_between_noise = false; // use between factor noise or not
    bool use_chr_less_full_graph = false; // whether full graph has character indexes or not
    bool use_landmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
    double pcm_threshold = 0.99; // Confidence probability for the pairwise consistency computation
    bool use_covariance = false; // use covariance in dataset file.
    bool use_PCM = false; // Use pairwise consistency maximization.
    // Call distributed optimization
    std::tuple<double, double, int> results = runDistributedMapper(nr_robots, log_dir, data_dir, trace_file,
                                                                   use_XY, use_OP, debug, prior_model, model,
                                                                   max_iter, rotation_estimate_change_threshold, pose_estimate_change_threshold,
                                                                   gamma, use_flagged_init, update_type, use_between_noise,
                                                                   use_chr_less_full_graph, use_landmarks, pcm_threshold, use_covariance, use_PCM, false);
    // Compare centralized and distributed pose estimates
    double tolerance = 10; // That's the best I can get right now with that many poses.
    EXPECT(assert_equal(std::get<0>(results), std::get<1>(results), tolerance*std::get<0>(results)));
}

TEST(DistributedMapper, test_distributed_estimation_argos_2robots) {
    // Parameters
    size_t nr_robots = 2; // number of robots
    string log_dir("/tmp/"); // log directory
    string data_dir("../../../test_data/argos_simulation/spoiled/example_2robots/"); // data directory
    string trace_file("/tmp/runG2o"); // data directory
    bool use_XY = false;
    bool use_OP = false;
    bool debug = false;
    noiseModel::Diagonal::shared_ptr prior_model = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
    size_t max_iter = 1000; // Maximum number of iterations of optimizer
    double rotation_estimate_change_threshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
    double pose_estimate_change_threshold = 1e-1; // Difference between pose estimate provides an early stopping condition
    double gamma = 1.0f; // Gamma value for over relaxation methods
    bool use_flagged_init = true; // to use flagged initialization or not
    distributed_mapper::DistributedMapper::UpdateType update_type = distributed_mapper::DistributedMapper::incUpdate; // update_type differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
    bool use_between_noise = false; // use between factor noise or not
    bool use_chr_less_full_graph = false; // whether full graph has character indexes or not
    bool use_landmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
    double pcm_threshold = 0.99; // Confidence probability for the pairwise consistency computation
    bool use_covariance = false; // use covariance in dataset file.
    bool use_PCM = true; // Use pairwise consistency maximization.
    // Call distributed optimization
    std::tuple<double, double, int> results = runDistributedMapper(nr_robots, log_dir, data_dir, trace_file,
                                                                   use_XY, use_OP, debug, prior_model, model,
                                                                   max_iter, rotation_estimate_change_threshold, pose_estimate_change_threshold,
                                                                   gamma, use_flagged_init, update_type, use_between_noise,
                                                                   use_chr_less_full_graph, use_landmarks, pcm_threshold, use_covariance, use_PCM, false);
    // Compare centralized and distributed pose estimates
    double tolerance = 10; // That's the best I can get right now with that many poses.
    EXPECT(assert_equal(std::get<0>(results), std::get<1>(results), tolerance*std::get<0>(results)));
    std::cout << "Max clique size: " << std::get<2>(results) << std::endl;
}

TEST(DistributedMapper, test_distributed_estimation_argos_4robots) {
    // Parameters
    size_t nr_robots = 4; // number of robots
    string log_dir("/tmp/"); // log directory
    string data_dir("../../../test_data/argos_simulation/spoiled/example_4robots/"); // data directory
    string trace_file("/tmp/runG2o"); // data directory
    bool use_XY = false;
    bool use_OP = false;
    bool debug = false;
    noiseModel::Diagonal::shared_ptr prior_model = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
    size_t max_iter = 1000; // Maximum number of iterations of optimizer
    double rotation_estimate_change_threshold = 1e-1; // Difference between rotation estimate provides an early stopping condition
    double pose_estimate_change_threshold = 1e-1; // Difference between pose estimate provides an early stopping condition
    double gamma = 1.0f; // Gamma value for over relaxation methods
    bool use_flagged_init = true; // to use flagged initialization or not
    distributed_mapper::DistributedMapper::UpdateType update_type = distributed_mapper::DistributedMapper::incUpdate; // update_type differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
    bool use_between_noise = false; // use between factor noise or not
    bool use_chr_less_full_graph = false; // whether full graph has character indexes or not
    bool use_landmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
    double pcm_threshold = 0.99; // Confidence probability for the pairwise consistency computation
    bool use_covariance = false; // use covariance in dataset file.
    bool use_PCM = true; // Use pairwise consistency maximization.
    // Call distributed optimization
    std::tuple<double, double, int> results = runDistributedMapper(nr_robots, log_dir, data_dir, trace_file,
                                                                   use_XY, use_OP, debug, prior_model, model,
                                                                   max_iter, rotation_estimate_change_threshold, pose_estimate_change_threshold,
                                                                   gamma, use_flagged_init, update_type, use_between_noise,
                                                                   use_chr_less_full_graph, use_landmarks, pcm_threshold, use_covariance, use_PCM, false);
    // Compare centralized and distributed pose estimates
    double tolerance = 10; // That's the best I can get right now with that many poses.
    EXPECT(assert_equal(std::get<0>(results), std::get<1>(results), tolerance*std::get<0>(results)));
    std::cout << "Max clique size: " << std::get<2>(results) << std::endl;
}

/****************************************************************************** */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
//******************************************************************************
