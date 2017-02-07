#include <aerial_autonomy/basic_events.h>
#include <aerial_autonomy/visual_servoing_events.h>
#include <gtest/gtest.h>
#include <iostream>
#include <typeinfo>

using namespace std;

//// \brief Definitions
struct LSM {
	template<class Event>
	void process_event(Event &evt) {
		std::cout<<"Event type : "<<typeid(evt).name()<<std::endl;
	}
};
////

/// \brief TEST
/// All the tests are defined here
namespace basic_events{
	TEST(BasicEventManagerTest, InstantiateManager) {
		ASSERT_NO_THROW(new BasicEventManager<LSM>());
	}
	TEST(BasicEventManagerTest, PrintEventMap) {
		BasicEventManager<LSM> evt_manager;
		ASSERT_NO_THROW(evt_manager.printEventList());
	}
	TEST(BasicEventManagerTest, TriggerEvents) {
		BasicEventManager<LSM> evt_manager;
    LSM lsm;
	  evt_manager.triggerEvent("Land", lsm);
	  evt_manager.triggerEvent("Takeoff", lsm);
	  evt_manager.triggerEvent("Abort", lsm);
	}
}
namespace visual_servoing_events{
	TEST(VisualServoingEventManagerTest, InstantiateManager) {
		ASSERT_NO_THROW(new VisualServoingEventManager<LSM>());
	}
	TEST(VisualServoingEventManagerTest, TriggerEvents) {
		VisualServoingEventManager<LSM> evt_manager;
    LSM lsm;
	  evt_manager.triggerEvent("Land", lsm);
	  evt_manager.triggerEvent("Takeoff", lsm);
	  evt_manager.triggerEvent("Abort", lsm);
	  evt_manager.triggerEvent("FollowTrajectory", lsm);
	}
}
///

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
