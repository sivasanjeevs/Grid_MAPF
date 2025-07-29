#include "TaskManager.h"
#include "Tasks.h"
#include "nlohmann/json.hpp"
#include <vector>

using json = nlohmann::ordered_json;

TaskManager::TaskManager(int num_of_agents, const list<list<int>> &tasks, int num_tasks_reveal, Logger *logger) :
        num_of_agents(num_of_agents), tasks(tasks), num_tasks_reveal(num_tasks_reveal), logger(logger) {
    current_assignment.resize(num_of_agents, -1);
    finished_tasks.resize(num_of_agents);
    planner_schedule.resize(num_of_agents);
    actual_schedule.resize(num_of_agents);
}

bool TaskManager::validate_task_assignment(vector<int> &assignment) {
    if (assignment.size() != num_of_agents) {
        schedule_errors.push_back(make_tuple("Invalid schedule size", -1, -1, -1, curr_timestep + 1));
        if(logger) logger->log_warning("Scheduler Error: assignment size does not match number of agents", curr_timestep + 1);
        return false;
    }
    std::unordered_map<int, int> idx_set;
    for (int i_agent = 0; i_agent < assignment.size(); i_agent++) {
        if (assignment[i_agent] != -1 && ongoing_tasks.find(assignment[i_agent]) == ongoing_tasks.end()) {
            schedule_errors.push_back(make_tuple("task already finished", assignment[i_agent], i_agent, -1, curr_timestep + 1));
            if(logger) logger->log_warning("Scheduler Error: agent " + std::to_string(i_agent) + " to task " + std::to_string(assignment[i_agent]) + " wrong because the task is already finished", curr_timestep + 1);
            if(logger) logger->flush();
            return false;
        }
        if (assignment[i_agent] != -1 && idx_set.find(assignment[i_agent]) != idx_set.end()) {
            schedule_errors.push_back(make_tuple("task is already assigned by the second agent at the same time", assignment[i_agent], i_agent, idx_set[assignment[i_agent]], curr_timestep + 1));
            if(logger) logger->log_warning("Scheduler Error: agent " + std::to_string(i_agent) + " to task " + std::to_string(assignment[i_agent]) + " wrong because the task is already assigned to agent " + std::to_string(idx_set[assignment[i_agent]]), curr_timestep + 1);
            return false;
        }
        if (current_assignment[i_agent] != -1) {
            if (ongoing_tasks.at(current_assignment[i_agent])->idx_next_loc > 0 && (assignment[i_agent] == -1 || assignment[i_agent] != current_assignment[i_agent])) {
                schedule_errors.push_back(make_tuple("task is already opened by the second agent", assignment[i_agent], i_agent, ongoing_tasks.at(current_assignment[i_agent])->agent_assigned, curr_timestep + 1));
                if(logger) logger->log_warning("Scheduler Error: agent " + std::to_string(i_agent) + " to task " + std::to_string(assignment[i_agent]) + " wrong because the task is already opened by agent " + std::to_string(ongoing_tasks.at(current_assignment[i_agent])->agent_assigned), curr_timestep + 1);
                return false;
            }
        }
        if (assignment[i_agent] != -1) {
            idx_set[assignment[i_agent]] = i_agent;
        }
    }
    return true;
}

bool TaskManager::set_task_assignment(vector<int> &assignment) {
    for (int a = 0; a < assignment.size(); a++) {
        if (planner_schedule[a].empty() || assignment[a] != planner_schedule[a].back().second) {
            planner_schedule[a].push_back(make_pair(curr_timestep, assignment[a]));
        }
    }
    if (!validate_task_assignment(assignment)) {
        return false;
    }
    for (int a = 0; a < assignment.size(); a++) {
        if (current_assignment[a] >= 0) {
            ongoing_tasks.at(current_assignment[a])->agent_assigned = -1;
        }
    }
    for (int a = 0; a < assignment.size(); a++) {
        int t_id = assignment[a];
        current_assignment[a] = t_id;
        if (assignment[a] < 0) {
            continue;
        }
        ongoing_tasks.at(t_id)->agent_assigned = a;
    }
    for (int a = 0; a < current_assignment.size(); a++) {
        if (actual_schedule[a].empty() || current_assignment[a] != actual_schedule[a].back().second) {
            actual_schedule[a].push_back(make_pair(curr_timestep, current_assignment[a]));
        }
    }
    return true;
}

list<int> TaskManager::check_finished_tasks(vector<State> &states, int timestep) {
    list<int> finished_tasks_this_timestep;
    new_freeagents.clear();
    for (int k = 0; k < num_of_agents; k++) {
        if (current_assignment[k] != -1 && states[k].location == ongoing_tasks.at(current_assignment[k])->get_next_loc()) {
            Task *task = ongoing_tasks.at(current_assignment[k]);
            task->idx_next_loc += 1;
            if (task->is_finished()) {
                current_assignment[k] = -1;
                ongoing_tasks.erase(task->task_id);
                task->t_completed = timestep;
                finished_tasks_this_timestep.push_back(task->task_id);
                finished_tasks[task->agent_assigned].emplace_back(task);
                num_of_task_finish++;
                new_freeagents.push_back(k);
                if(logger) logger->log_info("Agent " + std::to_string(task->agent_assigned) + " finishes task " + std::to_string(task->task_id), timestep);
                if(logger) logger->flush();
            }
            events.push_back(make_tuple(timestep, k, task->task_id, task->idx_next_loc));
        }
    }
    return finished_tasks_this_timestep;
}

void TaskManager::sync_shared_env(SharedEnvironment *env) {
    env->task_pool.clear();
    for (auto &task: ongoing_tasks) {
        env->task_pool[task.first] = *task.second;
    }
    env->curr_task_schedule = current_assignment;
    env->new_freeagents.clear();
    for(auto agent : new_freeagents) {
        env->new_freeagents.push_back(agent);
    }
    env->new_tasks.clear();
    for(auto task : new_tasks) {
        env->new_tasks.push_back(task);
    }
}

void TaskManager::reveal_tasks(int timestep, const std::list<std::list<int>>& new_manual_tasks) {
    new_tasks.clear();
    if (!new_manual_tasks.empty()) {
        for (const auto& locs : new_manual_tasks) {
            Task *task = new Task(task_id, locs, timestep);
            ongoing_tasks[task->task_id] = task;
            all_tasks.push_back(task);
            new_tasks.push_back(task->task_id);
            if(logger) logger->log_info("Task " + std::to_string(task_id) + " is revealed manually");
            task_id++;
        }
    }
    else {
        while (ongoing_tasks.size() < num_tasks_reveal) {
            if (tasks.empty() || task_id >= tasks.size()) break;
            auto it = tasks.begin();
            std::advance(it, task_id);
            list<int> locs = *it;
            Task *task = new Task(task_id, locs, timestep);
            ongoing_tasks[task->task_id] = task;
            all_tasks.push_back(task);
            new_tasks.push_back(task->task_id);
            if(logger) logger->log_info("Task " + std::to_string(task_id) + " is revealed automatically");
            task_id++;
        }
    }
}

void TaskManager::update_tasks(vector<State> &states, vector<int> &assignment, int timestep) {
    curr_timestep = timestep;
    set_task_assignment(assignment);
    check_finished_tasks(states, timestep);
}

json TaskManager::to_json(int map_cols) const {
    json tasks = json::array();
    for (auto t: all_tasks) {
        json task = json::array();
        task.push_back(t->task_id);
        task.push_back(t->t_revealed);
        json locs = json::array();
        for (auto loc: t->locations) {
            locs.push_back(loc / map_cols);
            locs.push_back(loc % map_cols);
        }
        task.push_back(locs);
        tasks.push_back(task);
    }
    return tasks;
}