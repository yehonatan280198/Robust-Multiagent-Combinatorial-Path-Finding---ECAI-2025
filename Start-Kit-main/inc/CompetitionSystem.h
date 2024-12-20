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
    void saveResults(const string &fileName, int screen);

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

    vector<State> starts;
    int num_of_agents;

    std::vector<double> FailureProbability;
    int timeToDiagnosis;
    double verifyAlpha;
    double NoCollisionProbability;

    vector<State> curr_states;

    vector<list<Action>> actual_movements;
    vector<list<Action>> planner_movements;

    vector<list<std::tuple<int,int,std::string>>> events;
    std::vector<Task> unfinishedTasks;
    list<Task> all_tasks;

    //for evaluation
    vector<int> solution_costs;
    int num_of_task_finish = 0;
    list<double> planner_times; 
    bool fast_mover_feasible = true;


	void initialize();
    bool planner_initialize();
	virtual void update_tasks(std::vector<int>& currentAgents) = 0;
	virtual void Find_Who_To_Repair_And_The_Remain_Agents() = 0;

    void sync_shared_env();

    void move(vector<Action>& actions, int timestep);


    void log_preprocessing(bool succ);
};


class AllocationByMakespan : public BaseSystem
{
public:
	AllocationByMakespan(Grid &grid, MAPFPlanner* planner, std::vector<std::pair<int, int>>& start_locs, std::vector<int>& tasks, ActionModelWithRotate* model, std::vector<double>& Delay_Failure, int diagnosisTime, double alphaForVerify, double noCollisionProbability):
        BaseSystem(grid, planner, model)
    {
        int task_id = 0;
        for (auto& task_location: tasks){
            all_tasks.emplace_back(task_id++, task_location);
            unfinishedTasks.emplace_back(task_id, task_location);
        }
        num_of_agents = start_locs.size();
        starts.resize(num_of_agents);
        for (size_t i = 0; i < start_locs.size(); i++)
        {
            starts[i] = State(start_locs[i].first, 0, start_locs[i].second);
            env->curAgents.push_back(i);
            env->lastTimeMove.push_back(0);
        }

        FailureProbability = Delay_Failure;
        timeToDiagnosis = diagnosisTime;
        verifyAlpha = alphaForVerify;
        NoCollisionProbability = noCollisionProbability;
    };

	~AllocationByMakespan(){};


private:

	void update_tasks(std::vector<int>& currentAgents);
	void Find_Who_To_Repair_And_The_Remain_Agents();
};
