# Classes by Group
## Controllers
- BuiltInController
- ManualRPYTController

## ControllerHardwareConnectors
- AbstractControllerHardwareConnector
   - ControllerHardwareConnector
       - BuiltInVelocityControllerDroneConnector
       - PositionControllerDroneConnector
       - ManualRPYTControllerDroneConnector

## States
- BaseState
    - LogicStateMachineFrontEnd::Landed
    - ::TakingOff_
    - ::Hovering_
    - ::ReachingGoal_
    - ::Landing_

## Action Functors
- ActionFunctor
    - LandTransitionActionFunctor_
    - TakeoffTransitionActionFunctor_
    - PositionControlTransitionActionFunctor_
- EventAgnosticActionFunctor
    - LandInternalActionFunctor_
    - TakeoffInternalActionFunctor_
    - TakeoffAbortActionFunctor_
    - HoveringInternalActionFunctor_
    - PositionControlAbortActionFunctor_
    - PositionControlInternalActionFunctor_

## Guard Functors
- GuardFunctor
    - TakeoffTransitionGuardFunctor_
    - PositionControlTransitionGuardFunctor_
- EventAgnosticGuardFunctor

## State Machine Front End
- LogicStateMachineFrontEnd

## State Machine Back End
- ::LogicStateMachine

## Robot Systems
- UAVSystem
