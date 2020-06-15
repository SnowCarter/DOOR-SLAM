// Copyright (C) 2019 by Pierre-Yves Lajoie <lajoie.py@gmail.com>

#include "pairwise_consistency_maximization/distributed_pcm/distributed_pcm.h"

namespace distributed_pcm {

    std::pair<int, int> DistributedPCM::solveCentralized(std::vector< boost::shared_ptr<distributed_mapper::DistributedMapper> >& dist_mappers,
            std::vector<gtsam::GraphAndValues>& graph_and_values_vector,
            const double& pcm_threshold, const bool& use_covariance, const bool& use_heuristics) {

        std::vector<graph_utils::LoopClosures> separators_by_robot;
        std::vector<graph_utils::Transforms> transforms_by_robot;
        std::map<std::pair<char, char>,graph_utils::Transforms> separators_transforms_by_pair;

        fillInRequiredInformationCentralized(separators_by_robot, transforms_by_robot, separators_transforms_by_pair,
                dist_mappers, use_covariance);

        // Apply PCM for each pair of robots
        int total_max_clique_sizes = 0;
        int total_outliers_rejected = 0;
        for (int roboti = 0; roboti < dist_mappers.size(); roboti++) {
            for (int robotj = roboti+1; robotj < dist_mappers.size(); robotj++) {

                auto max_clique_info = executePCMCentralized(roboti, robotj, transforms_by_robot, separators_by_robot,
                separators_transforms_by_pair, dist_mappers,
                graph_and_values_vector, pcm_threshold, use_heuristics);

                total_max_clique_sizes += max_clique_info.first;
                total_outliers_rejected += max_clique_info.second;
            }
        }

        return std::make_pair(total_max_clique_sizes, total_outliers_rejected);
    }

    std::pair<std::pair<int, int>, std::pair<std::set<std::pair<gtsam::Key, gtsam::Key>>, std::set<std::pair<gtsam::Key, gtsam::Key>>>> 
                DistributedPCM::solveDecentralized(const int& other_robot_id,
                boost::shared_ptr<distributed_mapper::DistributedMapper>& dist_mapper,
                gtsam::GraphAndValues& local_graph_and_values,
                robot_measurements::RobotLocalMap& robot_local_map,
                const graph_utils::Trajectory& other_robot_trajectory,
                const double& pcm_threshold,
                const bool& is_prior_added, 
                const bool& use_heuristics) {

        graph_utils::Transforms empty_transforms;
        auto other_robot_local_info = robot_measurements::RobotLocalMap(other_robot_trajectory, empty_transforms, robot_local_map.getLoopClosures());

        // Apply PCM for each pair of robots
        auto max_clique_info = executePCMDecentralized(other_robot_id, robot_local_map, other_robot_local_info,
                                            dist_mapper, local_graph_and_values,
                                            pcm_threshold, is_prior_added, use_heuristics);

        return max_clique_info;
    }

    void DistributedPCM::fillInRequiredInformationCentralized(std::vector<graph_utils::LoopClosures>& separators_by_robot,
                        std::vector<graph_utils::Transforms>& transforms_by_robot,
                        std::map<std::pair<char, char>,graph_utils::Transforms>& separators_transforms_by_pair,
                        const std::vector< boost::shared_ptr<distributed_mapper::DistributedMapper> >& dist_mappers,
                        const bool& use_covariance){
        // Intialization of the pairs
        for (int i = 0; i < dist_mappers.size(); i++) {
            for (int j = i+1; j < dist_mappers.size(); j++) {
                graph_utils::Transforms transforms;
                separators_transforms_by_pair.insert(std::make_pair(std::make_pair(dist_mappers[i]->robotName(),dist_mappers[j]->robotName()), transforms));
            }
        }
        for (const auto& dist_mapper : dist_mappers) {
            // Store separators key pairs
            graph_utils::LoopClosures separators;
            for (auto id : dist_mapper->separatorEdge()) {
                auto separator_edge = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
                        dist_mapper->currentGraph().at(id));
                separators.emplace_back(std::make_pair(separator_edge->key1(), separator_edge->key2()));
            }
            separators_by_robot.emplace_back(separators);

            graph_utils::Transforms transforms;
            bool id_initialized = false;
            for (const auto& factor_ptr : dist_mapper->currentGraph()) {
                auto edge_ptr = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor_ptr);
                if (edge_ptr) { // Do nothing with the prior in the first robot graph
                    graph_utils::Transform transform;
                    transform.i = edge_ptr->key1();
                    transform.j = edge_ptr->key2();
                    transform.pose.pose = edge_ptr->measured();
                    if (use_covariance) {
                        transform.pose.covariance_matrix =
                                boost::dynamic_pointer_cast< gtsam::noiseModel::Gaussian >(edge_ptr->noiseModel())->covariance();
                    } else {
                        transform.pose.covariance_matrix = graph_utils::FIXED_COVARIANCE;
                    }
                    transform.is_separator = std::find(separators.begin(), separators.end(),
                                                       std::make_pair(edge_ptr->key1(), edge_ptr->key2())) !=
                                             separators.end();
                    if (!transform.is_separator) {
                        if (!id_initialized) {
                            transforms.start_id = transform.i;
                            transforms.end_id = transform.j;
                            id_initialized = true;
                        } else {
                            transforms.start_id = std::min(transforms.start_id, transform.i);
                            transforms.end_id = std::max(transforms.end_id, transform.j);
                        }
                        transforms.transforms.insert(
                                std::make_pair(std::make_pair(edge_ptr->key1(), edge_ptr->key2()), transform));
                    } else {
                        separators_transforms_by_pair.at(std::make_pair(gtsam::symbolChr(edge_ptr->key1()),gtsam::symbolChr(edge_ptr->key2()))).transforms.insert(
                                std::make_pair(std::make_pair(edge_ptr->key1(), edge_ptr->key2()), transform));
                    }
                }
            }
            transforms_by_robot.emplace_back(transforms);
        }
    }

    std::pair<int, int> DistributedPCM::executePCMCentralized(const int& roboti, const int& robotj, const std::vector<graph_utils::Transforms>& transforms_by_robot,
                const std::vector<graph_utils::LoopClosures>& separators_by_robot,
                const std::map<std::pair<char, char>,graph_utils::Transforms>& separators_transforms_by_pair,
                std::vector< boost::shared_ptr<distributed_mapper::DistributedMapper> >& dist_mappers,
                std::vector<gtsam::GraphAndValues>& graph_and_values_vector,
                const double& pcm_threshold,
                const bool& use_heuristics){
        auto roboti_local_map = robot_measurements::RobotLocalMap(transforms_by_robot[roboti], separators_by_robot[roboti]);
        auto robotj_local_map = robot_measurements::RobotLocalMap(transforms_by_robot[robotj], separators_by_robot[robotj]);
        auto roboti_robotj_separators_transforms = separators_transforms_by_pair.at(std::make_pair(dist_mappers[roboti]->robotName(),dist_mappers[robotj]->robotName()));
        auto interrobot_measurements = robot_measurements::InterRobotMeasurements(roboti_robotj_separators_transforms, dist_mappers[roboti]->robotName(), dist_mappers[robotj]->robotName());

        auto global_map = global_map::GlobalMap(roboti_local_map, robotj_local_map, interrobot_measurements, pcm_threshold, use_heuristics);
        auto max_clique_info = global_map.pairwiseConsistencyMaximization();
        std::vector<int> max_clique = max_clique_info.first;

        // Retrieve indexes of rejected measurements
        auto robot_pair = {roboti, robotj};
        for (auto robot : robot_pair) {
            auto separators_ids = dist_mappers[robot]->separatorEdge();
            std::vector<int> rejected_separator_ids;
            for (int i = 0; i < separators_ids.size(); i++) {
                if (isSeparatorToBeRejected(max_clique, separators_ids[i], roboti_robotj_separators_transforms,
                                            interrobot_measurements.getLoopClosures(), dist_mappers[robot])) {
                    rejected_separator_ids.emplace_back(i);
                }
            }
            // Remove measurements not in the max clique
            int number_separator_ids_removed = 0;
            for (const auto& index : rejected_separator_ids) {
                auto id = separators_ids[index] - number_separator_ids_removed;
                number_separator_ids_removed++;
                dist_mappers[robot]->eraseFactor(id);
                graph_and_values_vector.at(robot).first->erase(graph_and_values_vector.at(robot).first->begin()+id);
            }
            // Update separator ids
            std::vector<size_t> new_separator_ids;
            int number_of_edges = dist_mappers[robot]->currentGraph().size();
            if (robot == 0){
                // Do not count the prior in the first robot graph
                number_of_edges--;
            }
            for (int i = 0; i < number_of_edges; i++) {
                auto keys = dist_mappers[robot]->currentGraph().at(i)->keys();
                char robot0 = gtsam::symbolChr(keys.at(0));
                char robot1 = gtsam::symbolChr(keys.at(1));
                if (robot0 != robot1) {
                    new_separator_ids.push_back(i);
                }
            }
            dist_mappers[robot]->setSeparatorIds(new_separator_ids);
        }
        return std::make_pair(max_clique.size(), max_clique_info.second);
    }

    std::pair<std::pair<int, int>, std::pair<std::set<std::pair<gtsam::Key, gtsam::Key>>, std::set<std::pair<gtsam::Key, gtsam::Key>>>>
             DistributedPCM::executePCMDecentralized(const int& other_robot_id, robot_measurements::RobotLocalMap& robot_local_map,
                                            const robot_measurements::RobotLocalMap& other_robot_local_info,
                                            boost::shared_ptr<distributed_mapper::DistributedMapper>& dist_mapper,
                                            gtsam::GraphAndValues& local_graph_and_values,
                                            const double& pcm_threshold,
                                            const bool& is_prior_added,
                                            const bool& use_heuristics){

        graph_utils::Transforms roboti_robotj_separators_transforms;
        for (const auto& transform : robot_local_map.getTransforms().transforms) {
            auto id_1 = (int) (gtsam::Symbol(transform.second.i).chr()-97);
            auto id_2 = (int) (gtsam::Symbol(transform.second.j).chr()-97);
            if (id_1 == other_robot_id || id_2 == other_robot_id) {
                roboti_robotj_separators_transforms.transforms.insert(transform);
            }
        }

        auto interrobot_measurements = robot_measurements::InterRobotMeasurements(roboti_robotj_separators_transforms, 
                                                                                dist_mapper->robotName(), ((char) other_robot_id + 97));

        auto global_map = global_map::GlobalMap(robot_local_map, other_robot_local_info, interrobot_measurements, pcm_threshold, use_heuristics);
        auto max_clique_info = global_map.pairwiseConsistencyMaximization();
        std::vector<int> max_clique = max_clique_info.first;

        // Retrieve indexes of rejected measurements
        auto separators_ids = dist_mapper->separatorEdge();
        std::vector<int> rejected_separator_ids;
        std::set<std::pair<gtsam::Key, gtsam::Key>> accepted_key_pairs, rejected_key_pairs;
        for (int i = 0; i < separators_ids.size(); i++) {
            if (isSeparatorToBeRejected(max_clique, separators_ids[i], roboti_robotj_separators_transforms,
                                        interrobot_measurements.getLoopClosures(), dist_mapper)) {
                rejected_separator_ids.emplace_back(i);

                // Update robot local map and store keys
                auto separator_factor = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(dist_mapper->currentGraph().at(separators_ids[i]));
                robot_local_map.removeTransform(std::make_pair(separator_factor->keys().at(0), separator_factor->keys().at(1)));
                dist_mapper->eraseSeparatorsSymbols(std::make_pair(separator_factor->keys().at(0), separator_factor->keys().at(1)));
                dist_mapper->eraseSeparatorsSymbols(std::make_pair(separator_factor->keys().at(1), separator_factor->keys().at(0)));
                rejected_key_pairs.insert(std::make_pair(separator_factor->keys().at(0), separator_factor->keys().at(1)));
            } else {
                auto separator_factor = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(dist_mapper->currentGraph().at(separators_ids[i]));
                accepted_key_pairs.insert(std::make_pair(separator_factor->keys().at(0), separator_factor->keys().at(1)));
            }
        }
        // Remove measurements not in the max clique
        int number_separator_ids_removed = 0;
        for (const auto& index : rejected_separator_ids) {
            auto id = separators_ids[index] - number_separator_ids_removed;
            number_separator_ids_removed++;
            dist_mapper->eraseFactor(id);
            local_graph_and_values.first->erase(local_graph_and_values.first->begin()+id);
        }
        // Update separator ids
        std::vector<size_t> new_separator_ids;
        int number_of_edges = dist_mapper->currentGraph().size();
        if (is_prior_added){
            // Do not count the prior in the first robot graph
            number_of_edges--;
        }
        for (int i = 0; i < number_of_edges; i++) {
            auto keys = dist_mapper->currentGraph().at(i)->keys();
            char robot0 = gtsam::symbolChr(keys.at(0));
            char robot1 = gtsam::symbolChr(keys.at(1));
            if (robot0 != robot1) {
                new_separator_ids.push_back(i);
            }
        }
        dist_mapper->setSeparatorIds(new_separator_ids);

        return std::make_pair(std::make_pair(max_clique.size(), max_clique_info.second), std::make_pair(accepted_key_pairs, rejected_key_pairs));
    }

    bool DistributedPCM::isSeparatorToBeRejected(const std::vector<int>& max_clique, const int& separtor_id, const graph_utils::Transforms& separators_transforms,
                                        const graph_utils::LoopClosures& loop_closures, boost::shared_ptr<distributed_mapper::DistributedMapper>& dist_mapper) {

        auto separator_factor = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(dist_mapper->currentGraph().at(separtor_id));
        // First check if the separator is between the 2 robots
        if (!((gtsam::symbolChr(separator_factor->keys().at(0)) == gtsam::symbolChr(separators_transforms.transforms.begin()->first.first)
                && gtsam::symbolChr(separator_factor->keys().at(1)) == gtsam::symbolChr(separators_transforms.transforms.begin()->first.second)) ||
              (gtsam::symbolChr(separator_factor->keys().at(0)) == gtsam::symbolChr(separators_transforms.transforms.begin()->first.second)
                && gtsam::symbolChr(separator_factor->keys().at(1)) == gtsam::symbolChr(separators_transforms.transforms.begin()->first.first)))){
            return false;
        }
        // Check if in the maximum clique
        auto key_pair = std::make_pair(separator_factor->keys().at(0), separator_factor->keys().at(1));
        int index;
        for (index = 0; index < loop_closures.size(); index++) {
            if (key_pair == loop_closures[index]) {
                break;
            }
        }
        if (std::find(max_clique.begin(), max_clique.end(), index) != max_clique.end()) {
            return false;
        }
        return true;
    }

}