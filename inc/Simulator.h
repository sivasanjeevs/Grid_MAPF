#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "ActionModel.h"
#include "Grid.h" // Included Grid.h for the Grid class definition
#include "Logger.h"
#include "SharedEnv.h"
#include "States.h"
#include "nlohmann/json.hpp"
#include <iostream>
#include <list>
#include <vector>

class Simulator {
public:
    // MODIFIED: Changed start_locs to a const reference to fix binding errors.
    Simulator(const Grid &grid, const std::vector<int> &start_locs, ActionModelWithRotate *model);

    // MODIFIED: Changed next_actions to a const reference.
    std::vector<State> move(const std::vector<Action> &next_actions);

    // MODIFIED: Renamed from get_current_state and changed to return a const reference for efficiency.
    const std::vector<State>& get_states() const;

    int get_curr_timestep() const { return timestep; }
    bool get_all_valid() const { return all_valid; }
    void sync_shared_env(SharedEnvironment *env);
    nlohmann::ordered_json actual_path_to_json() const;
    nlohmann::ordered_json planned_path_to_json() const;
    nlohmann::ordered_json starts_to_json() const;
    nlohmann::ordered_json action_errors_to_json() const;
    int get_number_errors() const;
    void set_logger(Logger* new_logger);

private:
    const Grid& map;
    ActionModelWithRotate *model;
    Logger* logger = nullptr;

    int timestep = 0;
    std::vector<Path> paths;
    std::vector<State> starts;
    int num_of_agents;
    std::vector<State> curr_states;
    std::vector<std::list<Action>> actual_movements;
    std::vector<std::list<Action>> planner_movements;
    bool all_valid = true;
};

#endif