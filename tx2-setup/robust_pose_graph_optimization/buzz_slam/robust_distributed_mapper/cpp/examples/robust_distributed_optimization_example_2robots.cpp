
#include "distributed_mapper/run_distributed_mapper.h"

using namespace std;
using namespace gtsam;

int main(int argc, char *argv[]) {
    ///////////////////////////////////////////////////////////////////
    //Command line arguments
    ///////////////////////////////////////////////////////////////////
    size_t nr_robots = 2; // number of robots
    string log_dir("/tmp/"); // log directory
    string data_dir("/tmp/"); // data directory
    string trace_file("/tmp/runG2o"); // data directory
    bool use_XY = false;
    bool use_OP = false;
    bool debug = false;

    ///////////////////////////////////////////////////////////////////
    // Config (specify noise covariances and maximum number iterations)
    ///////////////////////////////////////////////////////////////////
    noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Isotropic::Variance(6, 1e-12); // prior noise
    noiseModel::Isotropic::shared_ptr model = noiseModel::Isotropic::Variance(12, 1);
    size_t max_iter = 1000;  // Maximum number of iterations of optimizer
    double rotation_estimate_change_threshold = 1e-1;  // Difference between rotation estimate provides an early stopping condition
    double pose_estimate_change_threshold = 1e-1; // Difference between pose estimate provides an early stopping condition
    double gamma = 1.0f; // Gamma value for over relaxation methods
    bool use_flagged_init = true; // to use flagged initialization or not
    distributed_mapper::DistributedMapper::UpdateType update_type = distributed_mapper::DistributedMapper::incUpdate; // update_type differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
    bool use_between_noise = false; // use between factor noise or not
    bool use_chr_less_full_graph = false; // whether full graph has character indexes or not
    bool use_landmarks = false; // use landmarks -- landmarks are given symbols as upper case of robot name, for eg: if robot is 'a', landmark will be 'A'
    double pcm_threshold = 0.99; // confidence probability for the pairwise consistency computation.
    bool use_covariance = false; // use covariance in dataset file.
    bool use_PCM = false; // Use pairwise consistency maximization.
    bool use_heuristics = true; // Use heuristics-based algorithm for the max-clique solver.

    try {
        // Parse program options
        namespace po = boost::program_options;
        po::options_description desc("Options");
        desc.add_options()
                ("help", "Print help messages")
                ("nrRobots, n", po::value<size_t>(&nr_robots), "number of robots (default: 2)")
                ("dataDir, l", po::value<string>(&data_dir), "data directory (default: /tmp)")
                ("traceFile, t", po::value<string>(&trace_file), "trace file (default: runG2o)")
                ("logDir, l", po::value<string>(&log_dir), "log directory (default: /tmp)")
                ("useXY, u", po::value<bool>(&use_XY), "use x,y,z as naming convention or a,b,c (default: x,y,z)")
                ("useOP, o", po::value<bool>(&use_OP), "use o,p,q as naming convention (default: x,y,z)")
                ("useFlaggedInit, f", po::value<bool>(&use_flagged_init),
                 "use flagged initialization or not (default: true)")
                ("useBetweenNoise, b", po::value<bool>(&use_between_noise),
                 "use the given factor between noise instead of unit noise(default: false)")
                ("useChrLessFullGraph", po::value<bool>(&use_chr_less_full_graph),
                 "whether full graph has character indexes or not (default: false)")
                ("useLandmarks, l", po::value<bool>(&use_landmarks), "use landmarks or not (default: false)")
                ("rthresh, r", po::value<double>(&rotation_estimate_change_threshold),
                 "Specify difference between rotation estimate provides an early stopping condition (default: 1e-2)")
                ("pthresh, p", po::value<double>(&pose_estimate_change_threshold),
                 "Specify difference between pose estimate provides an early stopping condition (default: 1e-2)")
                ("maxIter, m", po::value<size_t>(&max_iter), "maximum number of iterations (default: 100000)")
                ("confidence, c", po::value<double>(&pcm_threshold), "confidence probability for the pairwise consistency computation (default: 0.99)")
                ("useCovariance, i", po::value<bool>(&use_covariance), "use covariance in dataset file (default: false)")
                ("debug, d", po::value<bool>(&debug), "debug (default: false)")
                ("usePCM", po::value<bool>(&use_PCM), "use pairwise consistency maximization (default: true)")
                ("useHeuristics", po::value<bool>(&use_heuristics), "use heuristics-based algorithm for the max-clique solver. (default: true)");

        po::variables_map vm;
        try {
            po::store(po::parse_command_line(argc, argv, desc), vm); // can throw
            if (vm.count("help")) { // --help option
                cout << "Run Distributed-Mapper" << endl
                     << "Example: ./rung2o --dataDir ../../../example/ --nrRobots 4"
                     << endl << desc << endl;
                return 0;
            }
            po::notify(vm);  // throws on error, so do after help in case
        }
        catch (po::error &e) {
            cerr << "ERROR: " << e.what() << endl << endl;
            cerr << desc << endl;
            return 1;
        }
    }
    catch (exception &e) {
        cerr << "Unhandled Exception reached the top of main: "
             << e.what() << ", application will now exit" << endl;
        return 2;
    }
    std::tuple<double, double, int> results = distributed_mapper::runDistributedMapper(nr_robots, log_dir, data_dir, trace_file,
            use_XY, use_OP, debug, priorModel, model,
            max_iter, rotation_estimate_change_threshold, pose_estimate_change_threshold,
            gamma, use_flagged_init, update_type, use_between_noise,
            use_chr_less_full_graph, use_landmarks, pcm_threshold, use_covariance, use_PCM, use_heuristics);

    return 0;
}