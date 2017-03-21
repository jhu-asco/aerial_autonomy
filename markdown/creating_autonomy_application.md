# Creating an Autonomy Application
To create a new autonomy application, we need to create a new state machine which defines the logic flow and a robot system which coordinates the hardware on the physical (or virtual) robot.
We need to implement the following components:

| Component | Description |
|-----------|-------------|
| Robot System | Provides sensor data and accepts control commands |
| Robot System Handler | Instantiates robot system and manages its control threads |
| Actions | Commands to execute when switching between states |
| Guards | Check if the transition between states is valid or not |
| Internal Actions | Process robot state continuously and trigger actions accordingly |


## Creating a Robot System
A robot system is responsible for owning any [ControllerHardwareConnectors](markdown/class_groups.md) and interacting with any hardware that the autonomy application will be utilizing. 
Examples can be found in [include/robot_systems](https://github.com/jhu-asco/aerial_autonomy/tree/master/include/aerial_autonomy/robot_systems).
New robot systems should extend BaseRobotSystem and add any ControllerHardwareConnectors to the system in the derived class constructor using
`controller_hardware_connector_container_.setObject(my_controller_connector_)`.  Keep in mind that only one instance of each ControllerHardwareConnector
type can be stored in the container.  The new robot system should expose any additional hardware functionality that will be used in the state machine,
 e.g. `takeoff()` or `land()` for the `UAVSystem`.

The state machine actions will have access to the robot system and interact with controllers by calling the robot system `setGoal` function templated on the controller it wants to use
to navigate to the given goal.

## Creating Actions
Actions define commands to execute when transitioning between states.  Examples can be found in `include/actions_guards`.
Actions which do not need access to the event which triggered the action should derive from `EventAgnosticActionFunctor<RobotSystemT, LogicStateMachineT>`
and override the `run(RobotSystemT& LogicStateMachineT&)` function with the command to be executed. See `include/land_functors.h` for an example. 

Actions which do need access to the triggering event should derive from `ActionFunctor<EventT, RobotSystemT, LogicStateMachineT>` and override the run function
with the command to execute.  See `include/position_control_functors.h` for an example.

## Creating Internal Actions
Internal actions define a behavior that is executed while in a particular state.  They should derive from `EventAgnosticActionFunctor<RobotSystemT, LogicStateMachineT>`
and override its `run` function with the intended behavior.  See `PositionControlInternalActionFunctor_` in `include/position_control_functors.h` for an example.

## Creating Guards
Guards check if a triggered transition between states is valid or not.  For example, the `TakeoffTransitionGuardFunctor_` keeps a `UAVSystem` from taking off if the battery level percentage
is below some threshold.  Guards which do not need access to the triggering event should derive from `EventAgnosticGuardFunctor<RobotSystemT, LogicStateMachineT>` and override the `guard` function
to return true when a transition is valid and false otherwise.  

Guards which do need access to the triggering event should inherit from `GuardFunctor<EventT, RobotSystemT, LogicStateMachineT>` and override its `guard` fuction. See `PositionControlTransitionGuardFunctor_` in `include/position_control_functors.h` for an example.

## Creating a State Machine
The state machine defines the logic of the system using the defined actions, states, and guards.  A "front end" for the state machine defines the connections and transitions between states.
The front end should derive from both `msmf::state_machine_def<StateMachineFrontEnd>` and `BaseStateMachine<RobotSystemT>`.  
Its `transition_table` will define the state machine itself by specifying which action will cause which state transitions and which guards will check the validity of the transitions. 
See `include/state_machines/uav_state_machine.h` and its associated flow chart below for an example and see [here](http://www.boost.org/doc/libs/1_63_0/libs/msm/doc/HTML/ch03s02.html) for a more in depth explanation of the underlying boost mechanisms. 

![alt text](state_machine.png "Example State Machine")

The state machine that we will interact with will be of type `boost::msm::back::thread_safe_state_machine<StateMachineFrontEnd>`.

Any new state machine must include `using BaseStateMachine<UAVSystem>::no_transition` to avoid type ambiguities.

## Creating a System Handler
The system handler should have a member of type `CommonSystemHandler<LogicStateMachineT, EventManagerT, RobotSystemT>`, which will automatically take care of instantiating and managing the logic state machine it is templated on.  The system handler must call the CommonSystemHandler `startTimers` function to start the state machine.
The system handler will need to instantiate the robot system and any hardware drivers and spawn any timers for stepping controllers owned by the robot system. See `include/system_handlers/uav_system_handler.h` for an example.

## Creating a Node
Create an executable with the following structure:

    int main(int argc, char** argv) {
      ros::init(argc, argv);
      ros::NodeHandle nh;
      // Load config into config
      MySystemHandler<MyStateMachine, MyEventManager>(nh, config)
      ros::spin();
    } 

See `src/system_handler_nodes/uav_system_node.cpp` for an example.
