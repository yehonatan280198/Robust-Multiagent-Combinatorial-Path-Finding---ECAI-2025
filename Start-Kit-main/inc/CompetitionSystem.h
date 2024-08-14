#pragma once
// #include "BasicSystem.h"
#include "SharedEnv.h"
#include "Grid.h"
#include "Tasks.h"
#include "ActionModel.h"
#include "MAPFPlanner.h"
#include "Logger.h"
#include <pthread.h>
#include <future>

class BaseSystem
{
public:
    int num_tasks_reveal = 1;
    Logger* logger = nullptr;

	BaseSystem(Grid &grid, MAPFPlanner* planner, ActionModelWithRotate* model):
        map(grid), planner(planner), env(planner->env), model(model)
    {}

	virtual ~BaseSystem()
    {
        //safely exit: wait for join the thread then delete planner and exit
        if (started)
        {
            task_td.join();
        }
        if (planner != nullptr)
        {
            delete planner;
        }
    };

    void set_num_tasks_reveal(int num){num_tasks_reveal = num;};
    void set_plan_time_limit(int limit){plan_time_limit = limit;};
    void set_preprocess_time_limit(int limit){preprocess_time_limit = limit;};
    void set_logger(Logger* logger){this->logger = logger;}

	void simulate(int simulation_time);
    vector<Action> plan();
    vector<Action> plan_wrapper();

    void savePaths(const string &fileName, int option) const; //option = 0: save actual movement, option = 1: save planner movement
    //void saveSimulationIssues(const string &fileName) const;
    void saveResults(const string &fileName, int screen) const;


protected:
    Grid map;

    std::future<std::vector<Action>> future;
    std::thread task_td;
    bool started = false;

    MAPFPlanner* planner;
    SharedEnvironment* env;

    ActionModelWithRotate* model;

    // #timesteps for simulation
    int timestep;

    int preprocess_time_limit=10;

    int plan_time_limit = 3;

    std::vector<Path> paths;
    std::vector<std::list<Task > > finished_tasks; // location + finish time

    vector<State> starts;
    int num_of_agents;

    std::vector<int> delays;

    vector<State> curr_states;

    vector<list<Action>> actual_movements;
    vector<list<Action>> planner_movements;

    // tasks that haven't been finished but have been revealed to agents;
    vector< deque<Task > > assigned_tasks;

    vector<list<std::tuple<int,int,std::string>>> events;
    list<Task> all_tasks;

    //for evaluation
    vector<int> solution_costs;
    int num_of_task_finish = 0;
    list<double> planner_times; 
    bool fast_mover_feasible = true;


	void initialize();
    bool planner_initialize();
	virtual void update_tasks() = 0;

    void sync_shared_env();

    list<Task> move(vector<Action>& actions, int timestep);
    bool valid_moves(vector<State>& prev, vector<Action>& next);

    void log_preprocessing(bool succ);
    void log_event_assigned(int agent_id, int task_id, int timestep);
    void log_event_finished(int agent_id, int task_id, int timestep);

};


class ConstantDelayAndFullObservation : public BaseSystem
{
public:
	ConstantDelayAndFullObservation(Grid &grid, MAPFPlanner* planner, std::vector<int>& start_locs, std::vector<int>& tasks, ActionModelWithRotate* model, std::vector<int> delaysVec):
        BaseSystem(grid, planner, model)
    {
        int task_id = 0;
        for (auto& task_location: tasks)
        {
//            all_tasks.emplace_back(task_id++, task_location);
//            task_queue.emplace_back(all_tasks.back().task_id, all_tasks.back().location);
              task_queue.emplace_back(task_id++, task_location);
        }
        num_of_agents = start_locs.size();
        starts.resize(num_of_agents);
        for (size_t i = 0; i < start_locs.size(); i++)
        {
            starts[i] = State(start_locs[i], 0, 0);
        }

        delays = delaysVec;
    };

	~ConstantDelayAndFullObservation(){};


private:
    deque<Task> task_queue;

	void update_tasks();
};
