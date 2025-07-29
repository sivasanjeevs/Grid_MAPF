#ifndef COMPETITIONSYSTEM_H
#define COMPETITIONSYSTEM_H

#include "ActionModel.h"
#include "Grid.h"
#include "Simulator.h"
#include "States.h"
#include <Logger.h>
#include <memory>
#include <string>
#include <vector>
#include "Entry.h"
#include "TaskManager.h"
#include "Tasks.h"
#include <future>
#include <thread>

class BaseSystem {
public:
    // MODIFIED: The constructor now accepts a vector of tasks (tasks_vec) to match the calling code in driver.cpp.
    // It then converts this vector to the internal std::list member `tasks`.
    BaseSystem(Grid& grid, Entry* planner, std::vector<int>& start_locs, const std::vector<std::list<int>>& tasks_vec, ActionModelWithRotate* model) :
            map(grid),
            planner(planner),
            num_of_agents(start_locs.size()),
            tasks(tasks_vec.begin(), tasks_vec.end()), // MODIFIED: Initialize member list by copying from the input vector.
            task_manager(start_locs.size(), this->tasks, 1, nullptr), // Pass the new list member to TaskManager.
            simulator(grid, start_locs, model) {
        env = planner->env;
        env->num_of_agents = num_of_agents;
    }

    void set_logger(Logger* logger) {
        this->logger = logger;
        task_manager.set_logger(logger);
        simulator.set_logger(logger); // This will now work after updating Simulator.h
    }

    void set_plan_time_limit(int time_limit) { plan_time_limit = time_limit; }
    void set_preprocess_time_limit(int time_limit) { preprocess_time_limit = time_limit; }

    // This now correctly sets the public member variable
    void set_num_tasks_reveal(float num) { task_manager.num_tasks_reveal = num * num_of_agents; };

    void simulate(int simulation_time);
    void saveResults(const std::string& fileName, int screen) const;

protected:
    Grid map;
    int num_of_agents;
    int plan_time_limit;
    int preprocess_time_limit;
    int simulation_time = 0;
    Logger* logger = nullptr;
    Entry* planner;
    SharedEnvironment* env = nullptr;

    TaskManager task_manager;
    Simulator simulator;

    std::vector<int> solution_costs;
    std::vector<double> planner_times;

    int total_timetous = 0;

    std::vector<Action> proposed_actions;
    std::vector<int> proposed_schedule;
    std::future<bool> future;
    std::thread task_td;
    bool started = false;

    void initialize();
    bool planner_initialize();
    void log_preprocessing(bool succ);
    // Corrected the condition to check against tasks size
    bool is_simulation_done() { return task_manager.num_of_task_finish >= tasks.size(); }
    void plan(int& timeout_timesteps);
    bool planner_wrapper();
    void sync_shared_env();
    
    // MODIFIED: Changed from a const reference to an object to hold the converted task list.
    std::list<std::list<int>> tasks;
};

#endif