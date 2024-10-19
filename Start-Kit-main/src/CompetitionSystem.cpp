#include <cmath>
#include "CompetitionSystem.h"
#include <boost/tokenizer.hpp>
#include "nlohmann/json.hpp"
#include <functional>
#include <Logger.h>

#include <algorithm>
#include <list>

#include <random>

using json = nlohmann::ordered_json;


void BaseSystem::move(vector<Action>& actions, int timestep)
{
    for (int k = 0; k < num_of_agents; k++)
    {
        // Check if the agent's delay allows him to move
        if ((timestep - env->lastTimeMove[k] < env->manufacturerDelay_FailureProbability[k].first) && (actions[k] != Action::NA))
            actions[k] = Action::W;

        else if (actions[k] != Action::NA){
            env->lastTimeMove[k] = timestep;
            env->observationDelay_TotalMoves[k].second += 1;
            env->observationDelay_TotalMoves[k].first = timestep / env->observationDelay_TotalMoves[k].second;

            // Generate a random number for the possibility of increasing the delay
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0.0, 1.0);
            if (dis(gen) < env->manufacturerDelay_FailureProbability[k].second)
                env->manufacturerDelay_FailureProbability[k].first += 1;
        }

        planner_movements[k].push_back(actions[k]);
    }

    curr_states = model->result_states(curr_states, actions);

    for (int k = 0; k < num_of_agents; k++)
    {
        // Check if the agent has reached his next task
        if (!env->goal_locations[k].empty() && curr_states[k].location == std::get<0>(env->goal_locations[k].front()))
        {
            int id_of_finish_goal = std::get<2>(env->goal_locations[k].front());
            env->goal_locations[k].erase(env->goal_locations[k].begin());
            events[k].push_back(make_tuple(id_of_finish_goal, timestep,"finished"));
            num_of_task_finish++;

            // Delete the task from the unfinished tasks.
            unfinishedTasks.erase(std::remove_if(unfinishedTasks.begin(), unfinishedTasks.end(), [id_of_finish_goal](const Task& currTask) {
                return currTask.task_id == id_of_finish_goal;
            }), unfinishedTasks.end());

        }
        paths[k].push_back(curr_states[k]);
        actual_movements[k].push_back(actions[k]);
        env->unfinishedTasks = unfinishedTasks;
    }
}


void BaseSystem::sync_shared_env() {

    if (!started)
        env->curr_states = curr_states;
    env->curr_timestep = timestep;
}


vector<Action> BaseSystem::plan_wrapper()
{
    vector<Action> actions;
    planner->plan(plan_time_limit, actions);

    return actions;
}


vector<Action> BaseSystem::plan()
{
    using namespace std::placeholders;
    if (started && future.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    {
        std::cout << started << "     " << (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) << std::endl;
        if(logger)
        {
            logger->log_info("planner cannot run because the previous run is still running", timestep);
        }

        if (future.wait_for(std::chrono::seconds(plan_time_limit)) == std::future_status::ready)
        {
            task_td.join();
            started = false;
            return future.get();
        }
        logger->log_info("planner timeout", timestep);
        return {};
    }

    std::packaged_task<std::vector<Action>()> task(std::bind(&BaseSystem::plan_wrapper, this));
    future = task.get_future();
    if (task_td.joinable())
    {
        task_td.join();
    }
    task_td = std::thread(std::move(task));
    started = true;
    if (future.wait_for(std::chrono::seconds(plan_time_limit)) == std::future_status::ready)
    {
        task_td.join();
        started = false;
        return future.get();
    }
    logger->log_info("planner timeout", timestep);
    return {};
}


bool BaseSystem::planner_initialize()
{
    using namespace std::placeholders;
    std::packaged_task<void(int)> init_task(std::bind(&MAPFPlanner::initialize, planner, _1));
    auto init_future = init_task.get_future();
    
    auto init_td = std::thread(std::move(init_task), preprocess_time_limit);
    if (init_future.wait_for(std::chrono::seconds(preprocess_time_limit)) == std::future_status::ready)
    {
        init_td.join();
        return true;
    }

    init_td.detach();
    return false;
}


void BaseSystem::log_preprocessing(bool succ)
{
    if (logger == nullptr)
        return;
    if (succ){
        logger->log_info("Preprocessing success", timestep);
    }
    else{
        logger->log_fatal("Preprocessing timeout", timestep);
    }
    logger->flush();
}


void BaseSystem::simulate(int simulation_time)
{
    initialize();

    while (unfinishedTasks.size() != 0)
    {
        sync_shared_env();

        // Check if it's time for a diagnosis
        if (timestep % env->timeToDiagnosis == 0 && timestep != 0){
            std::cout << "diagnosis" << std::endl;
            Find_Who_To_Repair_And_The_Remain_Agents();
        }

        // What is each agent's next step
        vector<Action> actions = plan();

        timestep++;
        for (int a = 0; a < num_of_agents; a++)
        {
            // If the agent has not completed its route, increase its route cost by one
            if (!env->goal_locations[a].empty())
                solution_costs[a]++;
        }

        // Move the agents according to the plan
        move(actions, timestep);
    }
}


void BaseSystem::initialize()
{
    paths.resize(num_of_agents);
    events.resize(num_of_agents);
    env->num_of_agents = num_of_agents;
    env->rows = map.rows;
    env->cols = map.cols;
    env->map = map.map;
    env->timeToDiagnosis = timeToDiagnosis;
    env->manufacturerDelay_FailureProbability = manufacturerDelay_FailureProbability;
    env->unfinishedTasks = unfinishedTasks;
    env->goal_locations.resize(num_of_agents);

    for (size_t i = 0; i < manufacturerDelay_FailureProbability.size(); ++i) {
        env->observationDelay_TotalMoves.push_back(std::make_pair(manufacturerDelay_FailureProbability[i].first, 0));
    }

    timestep = 0;
    curr_states = starts;

    bool planner_initialize_success= planner_initialize();
    
    log_preprocessing(planner_initialize_success);
    if (!planner_initialize_success)
        _exit(124);

    sync_shared_env();

    update_tasks(env->curAgents);

    actual_movements.resize(num_of_agents);
    planner_movements.resize(num_of_agents);
    solution_costs.resize(num_of_agents);
    for (int a = 0; a < num_of_agents; a++)
    {
        solution_costs[a] = 0;
    }
}

void BaseSystem::savePaths(const string &fileName, int option) const
{
    std::ofstream output;
    output.open(fileName, std::ios::out);
    for (int i = 0; i < num_of_agents; i++)
    {
        output << "Agent " << i << ": ";
        if (option == 0)
        {
            bool first = true;
            for (const auto t : actual_movements[i])
            {
                if (!first)
                {
                    output << ",";
                }
                else
                {
                    first = false;
                }
                output << t;
            }
        }
        else if (option == 1)
        {
            bool first = true;
            for (const auto t : planner_movements[i])
            {
                if (!first)
                {
                    output << ",";
                } 
                else 
                {
                    first = false;
                }
                output << t;
            }
        }
        output << endl;
    }
    output.close();
}


void BaseSystem::saveResults(const string &fileName, int screen)
{
    json js;
    // Save action model
    js["actionModel"] = "MAPF_T";

    std::string feasible = fast_mover_feasible ? "Yes" : "No";
    js["AllValid"] = feasible;

    js["teamSize"] = num_of_agents;

    // Save start locations[x,y,orientation]
    if (screen <= 2)
    {
        json start = json::array();
        for (int i = 0; i < num_of_agents; i++)
        {
            json s = json::array();
            s.push_back(starts[i].location/map.cols);
            s.push_back(starts[i].location%map.cols);
            switch (starts[i].orientation)
            {
            case 0:
                s.push_back("E");
                break;
            case 1:
                s.push_back("S");
            case 2:
                s.push_back("W");
                break;
            case 3:
                s.push_back("N");
                break;
            }
            start.push_back(s);
        }
        js["start"] = start;
    }

    js["numTaskFinished"] = num_of_task_finish;
    int sum_of_cost = 0;
    if (num_of_agents > 0)
    {
        sum_of_cost = solution_costs[0];
        for (int a = 1; a < num_of_agents; a++)
            sum_of_cost += solution_costs[a];
    }
    js["sumOfCost"] = sum_of_cost;
    js["makespan"] = timestep;
    
    if (screen <= 2)
    {
        // Save actual paths
        json apaths = json::array();
        for (int i = 0; i < num_of_agents; i++)
        {
            std::string path;
            bool first = true;
            for (const auto action : actual_movements[i])
            {
                if (!first)
                {
                    path+= ",";
                }
                else
                {
                    first = false;
                }

                if (action == Action::FW)
                {
                    path+="F";
                }
                else if (action == Action::CR)
                {
                    path+="R";
                } 
                else if (action == Action::CCR)
                {
                    path+="C";
                }
                else if (action == Action::NA)
                {
                    path+="T";
                }
                else
                {
                    path+="W";
                }
            }
            apaths.push_back(path);
        }
        js["actualPaths"] = apaths;
    }

    if (screen <=1)
    {
        //planned paths
        json ppaths = json::array();
        for (int i = 0; i < num_of_agents; i++)
        {
            std::string path;
            bool first = true;
            for (const auto action : planner_movements[i])
            {
                if (!first)
                {
                    path+= ",";
                } 
                else 
                {
                    first = false;
                }

                if (action == Action::FW)
                {
                    path+="F";
                }
                else if (action == Action::CR)
                {
                    path+="R";
                } 
                else if (action == Action::CCR)
                {
                    path+="C";
                } 
                else if (action == Action::NA)
                {
                    path+="T";
                }
                else
                {
                    path+="W";
                }
            }  
            ppaths.push_back(path);
        }
        js["plannerPaths"] = ppaths;

        // Save errors
        json errors = json::array();
        for (auto error: model->errors)
        {
            std::string error_msg;
            int agent1;
            int agent2;
            int timestep;
            std::tie(error_msg,agent1,agent2,timestep) = error;
            json e = json::array();
            e.push_back(agent1);
            e.push_back(agent2);
            e.push_back(timestep);
            e.push_back(error_msg);
            errors.push_back(e);

        }
        js["errors"] = errors;

        // Save events
        json events_json = json::array();
        for (int i = 0; i < num_of_agents; i++)
        {
            json event = json::array();
            for(auto e: events[i])
            {
                json ev = json::array();
                std::string event_msg;
                int task_id;
                int timestep;
                std::tie(task_id,timestep,event_msg) = e;
                ev.push_back(task_id);
                ev.push_back(timestep);
                ev.push_back(event_msg);
                event.push_back(ev);
            }
            events_json.push_back(event);
        }
        js["events"] = events_json;

        // Save all tasks
        json tasks = json::array();
//        all_tasks.sort([](const Task &a, const Task &b) {return a.task_id < b.task_id;});
        for (auto t: all_tasks)
        {
            json task = json::array();
            task.push_back(t.task_id);
            task.push_back(t.location/map.cols);
            task.push_back(t.location%map.cols);
            tasks.push_back(task);
        }
        js["tasks"] = tasks;
    }

    std::ofstream f(fileName,std::ios_base::trunc |std::ios_base::out);
    f << std::setw(4) << js;
}


void AllocationByMakespan::update_tasks(std::vector<int>& currentAgents)
{
    // Unassign all tasks
    for (int k = 0; k < env->num_of_agents; k++)
        env->goal_locations[k].clear();

    // Assign the tasks
    planner->updateTasks(currentAgents);

    // Save the task assignment events
    for (int i = 0; i < env->goal_locations.size(); ++i) {
        for (int j = 0; j < env->goal_locations[i].size(); ++j) {
            std::tuple<int, int, int> current_tuple = env->goal_locations[i][j];
            events[i].push_back(std::make_tuple(std::get<2>(current_tuple), std::get<1>(current_tuple), "assigned")); //(task_id, assign_timestep, assigned)
        }
    }

    sync_shared_env();
    plan();
}


void AllocationByMakespan::Find_Who_To_Repair_And_The_Remain_Agents(){

    // Create a vector of the remaining agents
    std::vector<int> remain_agents;
    for (const int &val : env->curAgents) {
        if (val != -1)
            remain_agents.push_back(val);
    }

    double min_time_to_finish = std::numeric_limits<double>::infinity();
    std::vector<int> potential_agents;

    // Create all possible combinations of remaining hazards
    int numCombinations = pow(2,remain_agents.size());
    for (int i = 0; i < numCombinations; ++i) {
        std::vector<int> combination;
        for (int j = 0; j < remain_agents.size(); ++j) {
            if (i & (1 << j))
                combination.push_back(remain_agents[j]);
        }

        if (combination.size() == 0)
            continue;

        // Find the minimum makespan of the combination
        update_tasks(combination);
        if (env->makeSpanForCurPlan < min_time_to_finish){
            min_time_to_finish = env->makeSpanForCurPlan;
            potential_agents = combination;
        }
    }

    // Assign the tasks to the combination with the minimum makespan
    update_tasks(potential_agents);

    // Update the remaining agents
    for (int &val : env->curAgents) {
        if (std::find(potential_agents.begin(), potential_agents.end(), val) == potential_agents.end())
            val = -1;
    }
}

