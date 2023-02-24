#include "SlowDance_Shake.h"
#include "../SlowDance.h"
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_solver/ConstraintSetLoader.h>
#include <map>

void SlowDance_Shake::start(mc_control::fsm::Controller & ctl_)

{
	std::map<std::string, int> jointIndexes = {};
	std::map<std::string, double> jointValues = {};
	std::vector<std::vector<std::vector<double>>> desired_pos_vector = {{}};
	std::vector<std::string> refJoint = ctl_.robot().refJointOrder();
   // voir les index réel de chaque joint
	for (int i=0; i<refJoint.size(); i++) {
		std::cout << i << ' ' << refJoint[i] << std::endl;
		jointIndexes.insert({ refJoint[i], ctl_.robot().jointIndexByName(refJoint[i])});
    std::cout << i << ' ' <<  ctl_.robot().jointIndexByName(refJoint[i]) << std::endl;
	}

	jointValues.insert({"R_HIP_Y", 0.0});
	jointValues.insert({"R_HIP_R", -0.02});
	jointValues.insert({"R_HIP_P", -0.95});
	jointValues.insert({"R_KNEE_P", 1.4});
	jointValues.insert({"R_ANKLE_P", 0.1});
	jointValues.insert({"R_ANKLE_R", 0.13});
	jointValues.insert({"L_HIP_Y", 0.0});
	jointValues.insert({"L_HIP_R", 0.02});
	jointValues.insert({"L_HIP_P", -0.95});
	jointValues.insert({"L_KNEE_P", 1.4});
	jointValues.insert({"L_ANKLE_P", 0.1});
	jointValues.insert({"L_ANKLE_R", -0.13});
	jointValues.insert({"CHEST_P", 0.5});
	jointValues.insert({"CHEST_Y", 0.0});
	jointValues.insert({"NECK_Y", 0.33});
	jointValues.insert({"NECK_P", 0.0});
	jointValues.insert({"R_SHOULDER_P", -0.052});
	jointValues.insert({"R_SHOULDER_R", -0.174444});
	jointValues.insert({"R_SHOULDER_Y", 0.0});
	jointValues.insert({"R_ELBOW_P", -1.44});
	jointValues.insert({"R_WRIST_Y", 0.0});
	jointValues.insert({"R_WRIST_P", 0.0});
	jointValues.insert({"R_WRIST_R", 0.0});
	jointValues.insert({"R_HAND_J0", 0.0});
	jointValues.insert({"R_HAND_J1", 0.0});
	jointValues.insert({"L_SHOULDER_P", -0.052});
	jointValues.insert({"L_SHOULDER_R", 0.174444});
	jointValues.insert({"L_SHOULDER_Y", 0.0});
	jointValues.insert({"L_ELBOW_P", -1.44});
	jointValues.insert({"L_WRIST_Y", 0.0});
	jointValues.insert({"L_WRIST_P", 0.0});
	jointValues.insert({"L_WRIST_R", 0.0});
	jointValues.insert({"L_HAND_J0", 0.0});
	jointValues.insert({"L_HAND_J1", 0.0});
 
	// ctl_.solver().addTask(neckTask);
  	//postureTask = ctl_.getPostureTask(ctl_.robot().name()); 
  	//postureTask->stiffness(200);
  	// neckTask = std::make_shared<mc_tasks::PostureTask>(ctl_.solver(), ctl_.robot().robotIndex(), 100, 10);
}


bool SlowDance_Shake::run(mc_control::fsm::Controller & ctl_)
{
	for (const auto& [key, value] : jointValues)
  		postureTask->target({{key, {{value}}}});

  output("OK");
  if (iter_ > 9999) {
  	return true;
  }
  iter_++;
  return false;

}


void SlowDance_Shake::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SlowDance &>(ctl_);
}


EXPORT_SINGLE_STATE("SlowDance_Shake", SlowDance_Shake)
