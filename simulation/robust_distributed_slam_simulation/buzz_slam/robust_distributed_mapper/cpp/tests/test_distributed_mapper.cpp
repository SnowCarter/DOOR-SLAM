/**
 * @file test_distributed_mapper.cpp
 * @author Siddharth Choudhary
 * @brief unit tests for test multi robot case simulated by loading the datasets
 */

#include <distributed_mapper/distributed_mapper.h>
#include <distributed_mapper/distributed_mapper_utils.h>
#include <distributed_mapper/evaluation_utils.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>
#include "distributed_pcm/distributed_pcm.h"

using namespace std;
using namespace gtsam;
using namespace distributed_mapper;
using namespace evaluation_utils;

noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);

// Prior
noiseModel::Diagonal::shared_ptr priorModel = //
        noiseModel::Isotropic::Variance(6, 1e-12);

string robotNames_("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ");
static const Matrix zero33= Matrix::Zero(3,3);


////////////////////////////////////////////////////////////////////////////////////////////
vector<Values>
distributedEstimation(size_t nrRobots, string dataPath, string traceFile, Values centralized, size_t maxIter = 5000,
                      bool useFlaggedInit = true, double rotationEstimateChangeThreshold = 1e-7,
                      double poseEstimateChangeThreshold = 1e-7, bool useBetweenNoise = false,
                      bool contains_odometry = false){

    ////////////////////////////////////////////////////////////////////////////////
    // Read Graph
    ////////////////////////////////////////////////////////////////////////////////
    string dataFile = dataPath + "/fullGraph.g2o";
    GraphAndValues graphAndValuesWithoutPrior = readG2o(dataFile, true);
    NonlinearFactorGraph graphCentralized = *(graphAndValuesWithoutPrior.first);

    ////////////////////////////////////////////////////////////////////////////////
    // Distributed Estimation
    ////////////////////////////////////////////////////////////////////////////////
    vector< boost::shared_ptr<DistributedMapper> > distMappers;

    // Load subgraph and construct distMapper optimizers
    for(size_t robot = 0; robot < nrRobots; robot++){
        boost::shared_ptr<DistributedMapper> distMapper(new DistributedMapper(robotNames_[robot], true));

        // read robot subgraphs
        string dataFile_i = dataPath + boost::lexical_cast<string>(robot) + ".g2o";
        GraphAndValues graphAndValues = readG2o(dataFile_i, true);
        distMapper->loadSubgraphAndCreateSubgraphEdge(graphAndValues);

        // Add prior
        if(robot==0){
            Symbol sym(robotNames_[robot],0);
            distMapper->addPrior(sym, Pose3(), priorModel);
        }

        // Verbosity level
        distMapper->setVerbosity(DistributedMapper::ERROR);

        distMapper->setUseBetweenNoiseFlag(useBetweenNoise);

        // Push to the set of optimizers
        distMappers.push_back(distMapper);
    }

    // Get estimates
    vector < Values > rotationTrace;
    vector < Values > poseTrace;
    vector < Values > subgraphRotationTrace;
    vector < Values > subgraphPoseTrace;
    vector < VectorValues > rotationVectorValuesTrace;

    double gamma = 1.0f;
    int max_clique_size = 0;
    vector< Values > estimates = distributedOptimizer(distMappers, maxIter, max_clique_size, DistributedMapper::incUpdate,
                                                      gamma, rotationEstimateChangeThreshold, poseEstimateChangeThreshold,
                                                      useFlaggedInit, false, false, contains_odometry, 0.99, false, true, true,
                                                      boost::none, rotationTrace, poseTrace,
                                                      subgraphRotationTrace, subgraphPoseTrace, rotationVectorValuesTrace);


    // Aggregate distributed estimates into values
    Values distributed;
    for(size_t i = 0; i< estimates.size(); i++){
        for(const Values::ConstKeyValuePair& key_value: estimates[i]){
            Key key = key_value.key;
            size_t keyIndex = symbolIndex(key);
            distributed.insert(keyIndex, estimates[i].at<Pose3>(key));
        }
    }

    // Save it
    string distributed_file = traceFile + "_distributed.g2o";
    writeG2o(graphCentralized, distributed, distributed_file);

    ////////////////////////////////////////////////////////////////////////////////
    // Centralized Rotation Estimation for Trace Comparisons
    ////////////////////////////////////////////////////////////////////////////////
    NonlinearFactorGraph pose3Graph = InitializePose3::buildPose3graph(graphCentralized);
    GaussianFactorGraph centralizedLinearGraph;
    noiseModel::Unit::shared_ptr rotationGraphModel = noiseModel::Unit::Create(9);

    for(const boost::shared_ptr<NonlinearFactor>& factor: pose3Graph) {
        Matrix3 Rij;

        boost::shared_ptr<BetweenFactor<Pose3> > pose3Between =
                boost::dynamic_pointer_cast<BetweenFactor<Pose3> >(factor);
        if (pose3Between)
            Rij = pose3Between->measured().rotation().matrix();
        else
            std::cout << "Error in buildLinearOrientationGraph" << std::endl;

        const FastVector<Key>& keys = factor->keys();
        Key key1 = keys[0], key2 = keys[1];
        Matrix M9 = Matrix::Zero(9,9);
        M9.block(0,0,3,3) = Rij;
        M9.block(3,3,3,3) = Rij;
        M9.block(6,6,3,3) = Rij;
        centralizedLinearGraph.add(key1, -I9, key2, M9, zero9, rotationGraphModel);
    }
    GaussianFactorGraph centralizedLinearGraphWithoutPrior = centralizedLinearGraph.clone();

    pair<NonlinearFactorGraph, Values> graphAndValues = loadGraphWithPrior(dataFile, priorModel);
    NonlinearFactorGraph graph = (graphAndValues.first);
    pose3Graph = InitializePose3::buildPose3graph(graph);
    centralizedLinearGraph = InitializePose3::buildLinearOrientationGraph(pose3Graph);
    VectorValues rotEstCentralized = centralizedLinearGraph.optimize();

    ////////////////////////////////////////////////////////////////////////////////
    // Log overall error w.r.t centralized graph
    ////////////////////////////////////////////////////////////////////////////////

    // Log distributed rotation error
    std::string traceFileOverallError = traceFile+ "_overall_error.txt";
    fstream traceStreamOverallError(traceFileOverallError.c_str(), fstream::out);
    for(size_t i=0; i<rotationVectorValuesTrace.size(); i++){
        traceStreamOverallError << centralizedLinearGraphWithoutPrior.error(rotationVectorValuesTrace[i]) << " ";
    }
    traceStreamOverallError << "-1" << endl;

    // Log distributed pose error
    for(size_t i=0; i<poseTrace.size(); i++){
        traceStreamOverallError << graphCentralized.error(poseTrace[i]) << " ";
    }
    traceStreamOverallError << "-1" << endl;

    // Log centralized pose error
    traceStreamOverallError << graphCentralized.error(centralized) << endl;
    traceStreamOverallError.close();

    // Log centralized rotation error
    double centralizedRotationError = centralizedLinearGraphWithoutPrior.error(rotEstCentralized);
    std::string traceFileCentralizedRotation = traceFile+ "_centralizedRotation.txt";
    fstream traceStreamCentralizedRotation(traceFileCentralizedRotation.c_str(), fstream::out);
    traceStreamCentralizedRotation << centralizedRotationError << endl;
    traceStreamCentralizedRotation.close();

    // Log each robot errors
    logResults(nrRobots, traceFile, centralized, distMappers);

    return estimates;
}


/******************************************************************************/
TEST(DistributedMapper, testdistributedEstimation_2robots) {

    // Read centralized graph
    string dataFile("../../../test_data/distributed_mapper/blocks_world/2robots/fullGraph.g2o");
    pair<NonlinearFactorGraph, Values> graphAndValues = loadGraphWithPrior(dataFile, priorModel);
    NonlinearFactorGraph graph = (graphAndValues.first);
    Values centralized = centralizedEstimation(graph, model, priorModel);

    size_t nrRobots = 2;
    string dataPath = "../../../test_data/distributed_mapper/blocks_world/2robots/";
    string traceFile = "../../../log/testdistributedEstimation_2robots";
    vector<Values> distributed = distributedEstimation(nrRobots, dataPath, traceFile, centralized, 1000, true, 1e-3, 1e-3);

    // Compare centralized and distributed pose estimates
    COMPARE_VALUES_DATASET(nrRobots, centralized, distributed, 1e-1);
}

/******************************************************************************/
TEST(DistributedMapper, testdistributedEstimation_4robots) {

  // Read centralized graph
  string dataFile("../../../test_data/distributed_mapper/blocks_world/4robots/fullGraph.g2o");
  pair<NonlinearFactorGraph, Values> graphAndValues = loadGraphWithPrior(dataFile, priorModel);
  NonlinearFactorGraph graph = (graphAndValues.first);
  Values centralized = centralizedEstimation(graph, model, priorModel);

  // Distributed Mapper
  size_t nrRobots = 4;
  string dataPath = "../../../test_data/distributed_mapper/blocks_world/4robots/";
  string traceFile = "../../../log/testdistributedEstimation_4robots";
  vector<Values> distributed = distributedEstimation(nrRobots, dataPath, traceFile, centralized, 1000, true, 1e-3, 1e-3);

  // TEST
  COMPARE_VALUES_DATASET(nrRobots, centralized, distributed, 1e-1);
}

/******************************************************************************/
TEST(DistributedMapper, testdistributedEstimation_9robots) {

  // Read centralized graph
  string dataFile("../../../test_data/distributed_mapper/blocks_world/9robots/fullGraph.g2o");
  pair<NonlinearFactorGraph, Values> graphAndValues = loadGraphWithPrior(dataFile, priorModel);
  NonlinearFactorGraph graph = (graphAndValues.first);
  Values centralized = centralizedEstimation(graph, model, priorModel);

  // Distributed Mapper
  size_t nrRobots = 9;
  string dataPath = "../../../test_data/distributed_mapper/blocks_world/9robots/";
  string traceFile = "../../../log/testdistributedEstimation_9robots";
  vector<Values> distributed = distributedEstimation(nrRobots, dataPath, traceFile, centralized, 1000, true, 1e-3, 1e-3);

  // TEST
  COMPARE_VALUES_DATASET(nrRobots, centralized, distributed, 1e-1);
}

/******************************************************************************/
TEST(DistributedMapper, testdistributedEstimation_16robots) {

  // Read centralized graph
  string dataFile("../../../test_data/distributed_mapper/blocks_world/16robots/fullGraph.g2o");
  pair<NonlinearFactorGraph, Values> graphAndValues = loadGraphWithPrior(dataFile, priorModel);
  NonlinearFactorGraph graph = (graphAndValues.first);
  Values centralized = centralizedEstimation(graph, model, priorModel);

  // Distributed Mapper
  size_t nrRobots = 16;
  string dataPath = "../../../test_data/distributed_mapper/blocks_world/16robots/";
  string traceFile = "../../../log/testdistributedEstimation_16robots";
  vector<Values> distributed = distributedEstimation(nrRobots, dataPath, traceFile, centralized, 1000);

  // TEST
  COMPARE_VALUES_DATASET(nrRobots, centralized, distributed, 1e-1);
}


/******************************************************************************/
TEST(DistributedMapper, testdistributedEstimation_25robots) {

  // Read centralized graph
  string dataFile("../../../test_data/distributed_mapper/blocks_world/25robots/fullGraph.g2o");
  pair<NonlinearFactorGraph, Values> graphAndValues = loadGraphWithPrior(dataFile, priorModel);
  NonlinearFactorGraph graph = (graphAndValues.first);
  Values centralized = centralizedEstimation(graph, model, priorModel);

  // Distributed Mapper
  size_t nrRobots = 25;
  string dataPath = "../../../test_data/distributed_mapper/blocks_world/25robots/";
  string traceFile = "../../../log/testdistributedEstimation_25robots";
  vector<Values> distributed = distributedEstimation(nrRobots, dataPath, traceFile, centralized);

  // TEST
  COMPARE_VALUES_DATASET(nrRobots, centralized, distributed, 1e-1);
}


/******************************************************************************/
TEST(DistributedMapper, testdistributedEstimation_36robots) {

  // Read centralized graph
  string dataFile("../../../test_data/distributed_mapper/blocks_world/36robots/fullGraph.g2o");
  pair<NonlinearFactorGraph, Values> graphAndValues = loadGraphWithPrior(dataFile, priorModel);
  NonlinearFactorGraph graph = (graphAndValues.first);
  Values centralized = centralizedEstimation(graph, model, priorModel);

  // Distributed Mapper
  size_t nrRobots = 36;
  string dataPath = "../../../test_data/distributed_mapper/blocks_world/36robots/";
  string traceFile = "../../../log/testdistributedEstimation_36robots";
  vector<Values> distributed = distributedEstimation(nrRobots, dataPath, traceFile, centralized);

  // TEST
  COMPARE_VALUES_DATASET(nrRobots, centralized, distributed, 1e-0);
}


/******************************************************************************/
TEST(DistributedMapper, testdistributedEstimation_49robots) {

  // Read centralized graph
  string dataFile("../../../test_data/distributed_mapper/blocks_world/49robots/fullGraph.g2o");
  pair<NonlinearFactorGraph, Values> graphAndValues = loadGraphWithPrior(dataFile, priorModel);
  NonlinearFactorGraph graph = (graphAndValues.first);
  Values centralized = centralizedEstimation(graph, model, priorModel);

  // Distributed Mapper
  size_t nrRobots = 49;
  string dataPath = "../../../test_data/distributed_mapper/blocks_world/49robots/";
  string traceFile = "../../../log/testdistributedEstimation_49robots";
  vector<Values> distributed = distributedEstimation(nrRobots, dataPath, traceFile, centralized);

  // TEST
  COMPARE_VALUES_DATASET(nrRobots, centralized, distributed, 1e-0);
}

/****************************************************************************** */
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
//******************************************************************************
