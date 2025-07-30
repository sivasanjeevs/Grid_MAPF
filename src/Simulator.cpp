#include "Simulator.h"
// #include "Action.h" // REMOVED: This line was incorrect and caused the error.
#include "nlohmann/json.hpp"

using json = nlohmann::ordered_json;

Simulator::Simulator(const Grid &grid, const std::vector<int> &start_locs, ActionModelWithRotate *model) : map(grid), model(model) {
    num_of_agents = start_locs.size();
    starts.resize(num_of_agents);
    paths.resize(num_of_agents);

    for (size_t i = 0; i < start_locs.size(); i++) {
        if (map.map.at(start_locs[i]) == 1) {
            std::cout << "error: agent " << i << "'s start location is an obstacle(" << start_locs[i] << ")" << std::endl;
            exit(1);
        }
        starts[i] = State(start_locs[i], 0, 0);
    }
    curr_states = starts;
    actual_movements.resize(num_of_agents);
    planner_movements.resize(num_of_agents);
}

std::vector<State> Simulator::move(const std::vector<Action> &actions_plan) {
    std::vector<Action> actions = actions_plan;
    all_valid = false;
    for (int k = 0; k < num_of_agents; k++) {
        if (k >= actions.size()) {
            planner_movements[k].push_back(Action::NA);
        } else {
            planner_movements[k].push_back(actions[k]);
        }
    }

    if (!model->is_valid(curr_states, actions, timestep)) {
        all_valid = false;
        actions = std::vector<Action>(num_of_agents, Action::W);
    } else {
        all_valid = true;
    }

    curr_states = model->result_states(curr_states, actions);
    timestep++;

    for (int k = 0; k < num_of_agents; k++) {
        paths[k].push_back(curr_states[k]);
        actual_movements[k].push_back(actions[k]);
    }
    return curr_states;
}

const std::vector<State>& Simulator::get_states() const {
    return curr_states;
}

void Simulator::sync_shared_env(SharedEnvironment *env) {
    env->curr_states = curr_states;
    env->curr_timestep = timestep;
}

json Simulator::actual_path_to_json() const {
    json apaths = json::array();
    for (int i = 0; i < num_of_agents; i++) {
        std::string path;
        bool first = true;
        for (const auto action : actual_movements[i]) {
            if (!first) {
                path += ",";
            }
            first = false;
            // Using the original if-else logic to convert actions to chars
            if (action == Action::FW) {
                path += "F";
            } else if (action == Action::CR) {
                path += "R";
            } else if (action == Action::CCR) {
                path += "C";
            } else if (action == Action::NA) {
                path += "T";
            } else {
                path += "W";
            }
        }
        apaths.push_back(path);
    }
    return apaths;
}

json Simulator::planned_path_to_json() const {
    json ppaths = json::array();
    for (int i = 0; i < num_of_agents; i++) {
        std::string path;
        bool first = true;
        for (const auto action : planner_movements[i]) {
            if (!first) {
                path += ",";
            }
            first = false;
            // Using the original if-else logic to convert actions to chars
            if (action == Action::FW) {
                path += "F";
            } else if (action == Action::CR) {
                path += "R";
            } else if (action == Action::CCR) {
                path += "C";
            } else if (action == Action::NA) {
                path += "T";
            } else {
                path += "W";
            }
        }
        ppaths.push_back(path);
    }
    return ppaths;
}

json Simulator::starts_to_json() const {
    json start = json::array();
    for (int i = 0; i < num_of_agents; i++) {
        json s = json::array();
        s.push_back(starts[i].location / map.cols);
        s.push_back(starts[i].location % map.cols);
        switch (starts[i].orientation) {
            case 0: s.push_back("E"); break;
            case 1: s.push_back("S"); break;
            case 2: s.push_back("W"); break;
            case 3: s.push_back("N"); break;
        }
        start.push_back(s);
    }
    return start;
}

json Simulator::action_errors_to_json() const {
    json errors = json::array();
    for (auto error : model->errors) {
        json e = json::array();
        e.push_back(std::get<1>(error));
        e.push_back(std::get<2>(error));
        e.push_back(std::get<3>(error));
        e.push_back(std::get<0>(error));
        errors.push_back(e);
    }
    return errors;
}

int Simulator::get_number_errors() const {
    return model->errors.size();
}

void Simulator::set_logger(Logger* new_logger) {
    this->logger = new_logger;
    model->set_logger(new_logger);
}
