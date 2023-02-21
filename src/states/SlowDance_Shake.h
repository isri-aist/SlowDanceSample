#pragma once

#include <mc_control/fsm/State.h>
#include <mc_control/fsm/Controller.h>

struct SlowDance_Shake : mc_control::fsm::State
{

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;
private:
std::shared_ptr<mc_tasks::PostureTask> postureTask;
std::vector<std::vector<std::vector<double>>> desired_pos_vector;
unsigned iter_ = 0;
int index_ = 0;
int t_ = 2000;

std::string jointName = "L_WRIST_P";
int jointIndex = 0;


};