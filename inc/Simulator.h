#ifndef SIMULATOR_H // ADDED: Header guard to prevent multiple inclusions
#define SIMULATOR_H // ADDED: Header guard

#include "ActionModel.h"
#include "SharedEnv.h"
#include "States.h"
#include "nlohmann/json.hpp"
#include "Logger.h"     // ADDED: Include for Logger
#include <vector>       // ADDED: Include for std::vector
#include <list>         // ADDED: Include for std::list
#include <iostream>     // ADDED: Include for std::cout, std::endl

class Simulator {
public:
    Simulator(Grid &grid, std::vector<int> &start_locs, ActionModelWithRotate *model) : map(grid), model(model) {
        num_of_agents = start_locs.size();
        starts.resize(num_of_agents);
        paths.resize(num_of_agents);

        for (size_t i = 0; i < start_locs.size(); i++) {
            if (grid.map[start_locs[i]] == 1) {
                // MODIFIED: Used std::cout and std::endl for clarity.
                std::cout << "error: agent " << i << "'s start location is an obstacle(" << start_locs[i] << ")" << std::endl;
                exit(0);
            }
            starts[i] = State(start_locs[i], 0, 0);
        }

        curr_states = starts;

        actual_movements.resize(num_of_agents);
        planner_movements.resize(num_of_agents);
    }

    std::vector<State> move(std::vector<Action> &next_actions);

    std::vector<State> get_current_state() { return curr_states; }

    int get_curr_timestep() { return timestep; }

    bool get_all_valid() { return all_valid; }

    void sync_shared_env(SharedEnvironment *env);

    nlohmann::ordered_json actual_path_to_json() const;
    nlohmann::ordered_json planned_path_to_json() const;
    nlohmann::ordered_json starts_to_json() const;
    nlohmann::ordered_json action_errors_to_json() const;

    int get_number_errors() const { return model->errors.size(); }

    // ADDED: This function was missing, causing a compilation error.
    void set_logger(Logger* new_logger) { this->logger = new_logger; }

private:
    Grid map;
    ActionModelWithRotate *model;
    Logger* logger = nullptr; // ADDED: Logger member to store the logger instance.

    int timestep = 0;
    std::vector<Path> paths;
    std::vector<State> starts;
    int num_of_agents;
    std::vector<State> curr_states;
    std::vector<std::list<Action>> actual_movements;
    std::vector<std::list<Action>> planner_movements;
    bool all_valid = true;
};

#endif // ADDED: End of header guard