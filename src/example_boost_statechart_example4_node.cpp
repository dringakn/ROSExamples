/*
./bosce
~/test_ws/devel/lib/test_boost_statechart_example/test_boost_statechart_example4_node
-s SmStateMachine > uml.pu && plantuml uml.pu && xdg-open uml.png

    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description: Boost start chart example.
    NOTE : The context<>() can't be used inside the constructor / destructors of
           the state.
       - If the event is not handled by the inner state it is propogated to the
         outer state.
       - The state history is not maintained. That means the default inner state
         is resumed on (re-)entry.
       - If the event needs to handled at both inner and outer state use
         forward_event() inside the custom event handler.
       - exit() function gets called ONLY IF state gets terminated because state
         machine is moved to another state.
       - The event not relevant to the current scope can be queued using defer
         events.
       - Orthogoanl regions are similar to inner states of a meta state,
         however, they run concurrently.

*/

#include <ros/ros.h>

#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/deferral.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/transition.hpp>

using namespace std;
namespace sc = boost::statechart;
namespace mpl = boost::mpl;

struct EvOne : sc::event<EvOne> {
  EvOne() { cout << "EvOne" << endl; }
};

struct EvTwo : sc::event<EvTwo> {
  EvTwo() { cout << "EvTwo" << endl; }
};

struct EvThree : sc::event<EvThree> {
  EvThree() { cout << "EvThree" << endl; }
};

struct EvFour : sc::event<EvFour> {
  EvFour() { cout << "EvFour" << endl; }
};

struct StOne;
struct StTwo;
struct StThree;

struct SmStateMachine : sc::state_machine<SmStateMachine, StOne> {
  int data;
  SmStateMachine() : data(0) { cout << "SmStateMachine: " << endl; }
  ~SmStateMachine() { cout << "~SmStateMachine:" << endl; }
};

struct StOne : sc::simple_state<StOne, SmStateMachine> {
  StOne() { cout << "StOne" << endl; }
  typedef mpl::list<sc::transition<EvOne, StTwo>, sc::deferral<EvThree>,
                    sc::deferral<EvFour>>
      reactions;
};

struct StTwo : sc::simple_state<StTwo, SmStateMachine> {
  StTwo() { cout << "StTwo" << endl; }
  typedef sc::transition<EvTwo, StThree> reactions;
};

struct StThree : sc::simple_state<StThree, SmStateMachine> {
  StThree() { cout << "StThree" << endl; }
  typedef mpl::list<sc::custom_reaction<EvThree>, sc::custom_reaction<EvFour>>
      reactions;
  sc::result react(const EvThree& e) {
    cout << "EvThreeCB" << endl;
    return discard_event();
  };
  sc::result react(const EvFour& e) {
    cout << "EvFourCB" << endl;
    return discard_event();
  };
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_boost_statechart_example4_node");
  ros::NodeHandle nh;
  shared_ptr<SmStateMachine> sm = make_shared<SmStateMachine>();
  sm->initiate();
  sm->process_event(EvThree());  // Queue
  sm->process_event(EvThree());  // Queue
  sm->process_event(EvThree());  // Queue
  sm->process_event(EvFour());   // Queue
  sm->process_event(EvFour());   // Queue
  sm->process_event(EvFour());   // Queue
  sm->process_event(EvFour());   // Queue
  sm->process_event(EvOne());    // S2
  sm->process_event(EvTwo());    // S3

  return 0;
}