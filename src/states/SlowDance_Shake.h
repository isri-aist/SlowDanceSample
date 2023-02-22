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
std::shared_ptr<mc_tasks::PostureTask> neckTask;
std::string jointName = "NECK_Y";
int jointIndex = 0;


};