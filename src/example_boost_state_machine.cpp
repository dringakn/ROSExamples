/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description: Boost start chart example.
        First create events, then states and then the state machine.
        Use CRTP (Curiously Recurring Template Pattern) and forward decleration.
        CRTP is used for compile time polymorphism and state machines.

        The message recieved, invalid, invalid-time period events are generate
        by the message callback. The message callbacke reset the timer before
        timeout occures, otherwise timer callback generates a timeout event.

        Whenver a transitiona occures, the object is destroyed and a new
        instance of the target state is created. So there is no persistance of
        data. Boost states are not meant to store persistant data during
        transistion.
    Note:
        Building:
          In order to visulize the state chart from the executable
          - sudo apt install plantuml
          - git clone https://github.com/kanje/bosce.git
          - mkdir build
          - cd build
          - cmake -DCMAKE_BUILD_TYPE=Release ../../bosce
          - make -j8
        Testing:
        - The executable must be built using -DCMAKE_BUILD_TYPE=Debug
        - cp
          ~/cmu_ws/devel/.private/ros_examples/ros_examples/lib/example_boost_state_machine
          binary
        - ./bosce binary -l (Get the list of state machines)
        - ./bosce binary -s StMachine > uml.pu
        - plantuml uml.pu
        - xdg-open uml.png
*/

#include <ros/ros.h>

#include <boost/statechart/custom_reaction.hpp>  // Custom reactions
#include <boost/statechart/event.hpp>            // Events
#include <boost/statechart/simple_state.hpp>     // State
#include <boost/statechart/state_machine.hpp>    // State machine
#include <boost/statechart/transition.hpp>       // Transistion

using namespace std;
namespace sc = boost::statechart;  // Boost statchart
namespace mpl = boost::mpl;        // Multiple event handler

/*
    Step1: Define events using CRTP
*/
struct EvMessageRecieved : sc::event<EvMessageRecieved> {
  EvMessageRecieved() { cout << "EvMessageRecieved" << endl; }
};
struct EvMessageTimeOut : sc::event<EvMessageTimeOut> {
  EvMessageTimeOut() { cout << "EvMessageTimeOut" << endl; }
};
struct EvInvalidMessage : sc::event<EvInvalidMessage> {
  EvInvalidMessage() { cout << "EvInvalidMessage" << endl; }
};
struct EvInvalidMessageTimePeriod : sc::event<EvInvalidMessageTimePeriod> {
  EvInvalidMessageTimePeriod() { cout << "EvInvalidMessageTimePeriod" << endl; }
};

/*
    Step2: Define state machine
    Because the context of a state must be a complete type (i.e. not forward
    declared), a machine must be defined from "outside to inside". That is, we
    always start with the state machine, followed by outermost states,
    followed by the direct inner states of outermost states and so on. We can
    do so in a breadth-first or depth-first way or employ a mixture of the
    two.

    Note: It's important that state_machine is defined before the states.

*/
struct StActive;   // Forward decleration
struct StStopped;  // Forward decleration
struct StRunning;  // Forward decleration
struct StIdle;     // Forward decleration
struct SmMessage : sc::state_machine<SmMessage, StActive> {};
/*
    Step3: Define states using CRTP
    The simple_state Class template accepts up to four parameters:
    - simple_stae<State, Context, Default Inner State>
    - Stopped and Running both specify Active as their Context, which makes them
      nested inside Active.
    - The third parameter specifies the inner initial state, if there is one.
      Here, only Active has inner states, which is why it needs to pass its
      inner initial state Stopped to its base.
    - The fourth parameter specifies whether and what kind of history is kept.
    - Active is the outermost state and therefore needs to pass the state
      machine Class it belongs to
*/
struct StActive : sc::simple_state<StActive, SmMessage, StStopped> {
  StActive() { cout << "StActive" << endl; }
  typedef sc::transition<EvMessageRecieved, StActive> reactions;
};
struct StStopped : sc::simple_state<StStopped, StActive> {
  StStopped() { cout << "StStopped" << endl; }
  typedef sc::transition<EvMessageTimeOut, StRunning> reactions;
};
struct StRunning : sc::simple_state<StRunning, StActive> {
  StRunning() { cout << "StRunning" << endl; }
  typedef sc::transition<EvMessageTimeOut, StIdle> reactions;
};
struct StIdle : sc::simple_state<StIdle, StActive> {
  StIdle() { cout << "StIdle" << endl; }
  typedef sc::transition<EvInvalidMessage, StStopped> reactions;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_boost_state_machine");
  ros::NodeHandle nh;
  SmMessage sm;   // Create the state machine object
  sm.initiate();  // Initialize the state machine
  sm.process_event(EvMessageRecieved());
  sm.process_event(EvMessageTimeOut());
  sm.process_event(EvInvalidMessage());
  ros::spin();
  return 0;
}