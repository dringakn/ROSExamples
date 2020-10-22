/*
./bosce
~/test_ws/devel/lib/test_boost_statechart_example/test_boost_statechart_example3_node
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
       - forward_event() inside the custom event handler.

*/

#include <ros/ros.h>

#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/transition.hpp>

using namespace std;
namespace sc = boost::statechart;
namespace mpl = boost::mpl;

struct EvOne : sc::event<EvOne> {
  int data;
  EvOne() : data(0) { cout << "EvOne" << endl; }
};

struct EvTwo : sc::event<EvTwo> {
  int data;
  EvTwo() : data(0) { cout << "EvTwo" << endl; }
};

struct EvThree : sc::event<EvThree> {
  EvThree() { cout << "EvThree" << endl; }
};

struct EvOneOne : sc::event<EvOneOne> {
  EvOneOne() { cout << "EvOneOne" << endl; }
};

struct EvOneTwo : sc::event<EvOneTwo> {
  EvOneTwo() { cout << "EvOneTwo" << endl; }
};

struct EvOneThree : sc::event<EvOneThree> {
  EvOneThree() { cout << "EvOneThree" << endl; }
};

struct StOne;
struct StOneOne;
struct StOneTwo;
struct StOneThree;
struct StTwo;

struct SmStateMachine : sc::state_machine<SmStateMachine, StOne> {
  SmStateMachine() { cout << "SmStateMachine: " << endl; }
  ~SmStateMachine() { cout << "~SmStateMachine:" << endl; }
};

struct StOne : sc::simple_state<StOne, SmStateMachine, StOneOne> {
  StOne() { cout << "StOne" << endl; }
  typedef mpl::list<sc::custom_reaction<EvOne>, sc::custom_reaction<EvThree>>
      reactions;
  sc::result react(const EvOne& e) { return transit<StTwo>(); }
  // Important Note:
  // There is a difference to handle the EvThree.
  // If the event is discarded, no internal state reset occures,
  // otherwise, the StOneOne (default inner state) becomes active.
  // sc::result react(const EvThree& e) { return discard_event(); }
  sc::result react(const EvThree& e) { return transit<StOne>(); }
};

struct StTwo : sc::simple_state<StTwo, SmStateMachine> {
  StTwo() { cout << "StTwo" << endl; }
  typedef mpl::list<sc::custom_reaction<EvTwo>> reactions;
  sc::result react(const EvTwo& e) { return transit<StOne>(); };
};

struct StOneOne : sc::simple_state<StOneOne, StOne> {
  StOneOne() { cout << "StOneOne" << endl; }
  typedef sc::transition<EvOneOne, StOneTwo> reactions;
};

struct StOneTwo : sc::simple_state<StOneTwo, StOne> {
  StOneTwo() { cout << "StOneTwo" << endl; }
  typedef sc::transition<EvOneTwo, StOneThree> reactions;
};

struct StOneThree : sc::simple_state<StOneThree, StOne> {
  StOneThree() { cout << "StOneThree" << endl; }
  typedef sc::transition<EvOneThree, StOneOne> reactions;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_boost_statechart_example3_node");
  ros::NodeHandle nh;
  shared_ptr<SmStateMachine> sm = make_shared<SmStateMachine>();
  sm->initiate();
  sm->process_event(EvOne());
  sm->process_event(EvTwo());
  sm->process_event(EvThree());
  sm->process_event(EvOneOne());
  sm->process_event(EvOneTwo());
  sm->process_event(EvOneThree());
  sm->process_event(EvThree());
  sm->process_event(EvOne());
  sm->process_event(EvOneOne());
  sm->process_event(EvOneTwo());
  sm->process_event(EvOneThree());
  sm->process_event(EvTwo());
  sm->process_event(EvOneOne());
  sm->process_event(EvThree());
  sm->process_event(EvTwo());
  sm->process_event(EvOne());
  sm->process_event(EvTwo());
  sm->process_event(EvOneOne());
  sm->process_event(EvOne());
  return 0;
}