#ifndef TASKMANAGER_H
#define TASKMANAGER_H

#include "Tasks.h"
#include "SharedEnv.h"
#include "Logger.h"
#include <vector>
#include <list>
#include <unordered_map>
#include <string>
#include <tuple>

// Using declarations are removed from here to prevent conflicts
using std::list;
using std::make_pair;
using std::make_tuple;
using std::pair;
using std::string;
using std::tuple;
using std::vector;


class TaskManager {
public:
    int num_of_agents;
    int num_of_task_finish = 0;
    int num_tasks_reveal;

    vector<vector<pair<int, int>>> planner_schedule; //timestep, task_id
    vector<vector<pair<int, int>>> actual_schedule;  //timestep, task_id

    vector<tuple<string, int, int, int, int>> schedule_errors;
    vector<tuple<int, int, int, int>> events; // <timestep, agent_id, task_id, seq_id>

    TaskManager(int num_of_agents, const list<list<int>> &tasks, int num_tasks_reveal, Logger *logger);

    void reveal_tasks(int timestep, const std::list<std::list<int>>& new_manual_tasks = {});
    void update_tasks(vector<State> &states, vector<int> &assignment, int timestep);
    void sync_shared_env(SharedEnvironment *env);

    int get_number_errors() const { return schedule_errors.size(); };
    nlohmann::ordered_json to_json(int map_cols) const;

    vector<vector<pair<int, int>>> get_actual_schedule() const { return actual_schedule; };
    vector<vector<pair<int, int>>> get_planner_schedule() const { return planner_schedule; };
    void set_logger(Logger* logger) { this->logger = logger; }

private:
    int task_id = 0;
    int curr_timestep = 0;
    const list<list<int>> &tasks; 
    // Explicitly use std::unordered_map to avoid conflicts
    std::unordered_map<int, Task *> ongoing_tasks;
    list<Task *> all_tasks;
    vector<list<Task *>> finished_tasks; 
    vector<int> current_assignment;      
    list<int> new_tasks;
    list<int> new_freeagents;
    Logger *logger;

    bool set_task_assignment(vector<int> &assignment);
    bool validate_task_assignment(vector<int> &assignment);
    list<int> check_finished_tasks(vector<State> &states, int timestep);
};

#endif