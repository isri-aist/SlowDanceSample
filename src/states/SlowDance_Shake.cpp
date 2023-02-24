#include "SlowDance_Shake.h"
#include "../SlowDance.h"
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_solver/ConstraintSetLoader.h>


void SlowDance_Shake::start(mc_control::fsm::Controller & ctl_)

{
	// std::map<std::string, int> jointIndexes = {};
	// std::map<std::string, double> jointValues = {};
	// std::vector<std::vector<std::vector<double>>> desired_pos_vector = {{}};
	// std::vector<std::string> refJoint = ctl_.robot().refJointOrder();
   // voir les index réel de chaque joint
	// for (int i=0; i<refJoint.size(); i++) {
		// std::cout << i << ' ' << refJoint[i] << std::endl;
		// jointIndexes.insert({ refJoint[i], ctl_.robot().jointIndexByName(refJoint[i])});
    // std::cout << i << ' ' <<  ctl_.robot().jointIndexByName(refJoint[i]) << std::endl;
	// }
 

	// ctl_.solver().addTask(neckTask);
  jointIndex = ctl_.robot().jointIndexByName(jointName);
  postureTask = ctl_.getPostureTask(ctl_.robot().name()); 
  postureTask->stiffness(200);
  	// neckTask = std::make_shared<mc_tasks::PostureTask>(ctl_.solver(), ctl_.robot().robotIndex(), 100, 10);
}


bool SlowDance_Shake::run(mc_control::fsm::Controller & ctl_)
{


 double ratio = (iter_ == 0) ? 0 : (double)(iter_%t_) / t_;
 mc_rtc::log::info("ratio={}", ratio);

 value += 0.001*sin(M_PI*ratio);
 
 mc_rtc::log::info("value={}", value);

if((value < (ctl_.robot().qu()[jointIndex][0])) && (value >(ctl_.robot().ql()[jointIndex][0])))

{
postureTask->target({{"NECK_Y", {value}}});
}

else if(value > 1.22)

{ double new_value = sin(M_PI/6*ratio);
 mc_rtc::log::info("new value={}", new_value);
postureTask->target({{"NECK_Y", {new_value}}});

}

else 
{
postureTask->target({{"NECK_Y", {-new_value}}});
}

// si postureTask->posture({{"NECK_Y", {value}}}); cela donne des erreurs 
//postureTask->target(desired_pos_vector[index_]); not working
//postureTask->target({{"NECK_Y", {{-0.33}}}});

  

  // output("OK");
  // if (iter_ > 9999) {
  	// return true;
  // }
  // iter_++;
  // return false;
  iter_++;
 return true;
}


void SlowDance_Shake::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SlowDance &>(ctl_);
}


EXPORT_SINGLE_STATE("SlowDance_Shake", SlowDance_Shake)
