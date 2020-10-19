/*
./bosce
~/test_ws/devel/lib/test_boost_statechart_example/test_boost_statechart_example2_node
-s SmStateMachine > uml.pu && plantuml uml.pu && xdg-open uml.png

NOTE : The context<>() can't be used inside the constructor / destructors of the
       state.

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
  EvOne() { cout << "EvOne" << endl; }
};

struct EvTwo : sc::event<EvTwo> {
  EvTwo() { cout << "EvTwo" << endl; }
};

struct EvThree : sc::event<EvThree> {
  EvThree() { cout << "EvThree" << endl; }
};

struct StOne;
struct StTwo;

struct SmStateMachine : sc::state_machine<SmStateMachine, StOne> {
  int var;
  SmStateMachine() : var(0) { cout << "SmStateMachine: " << var << endl; }
  ~SmStateMachine() { cout << "~SmStateMachine:" << var << endl; }
};

struct StOne : sc::simple_state<StOne, SmStateMachine> {
  StOne() { cout << "StOne" << endl; }
  typedef mpl::list<sc::custom_reaction<EvOne>, sc::custom_reaction<EvThree>>
      reactions;
  // Note: The context<>() can't be used inside the constructor / destructors of
  // the state.
  sc::result react(const EvOne& e) {
    context<SmStateMachine>().var++;
    int temp = context<SmStateMachine>().var;
    cout << "VAR: " << temp << endl;
    return transit<StTwo>();
  }
  sc::result react(const EvThree& e) {
    context<SmStateMachine>().var = 0;
    return discard_event();
  }
};

struct StTwo : sc::simple_state<StTwo, SmStateMachine> {
  StTwo() { cout << "StTwo" << endl; }
  typedef mpl::list<sc::custom_reaction<EvTwo>, sc::custom_reaction<EvThree>>
      reactions;
  sc::result react(const EvTwo& e) {
    return (context<SmStateMachine>().var < 3) ? transit<StOne>()
                                               : discard_event();
  };
  sc::result react(const EvThree& e) {
    context<SmStateMachine>().var = 0;
    return transit<StOne>();
  };
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_boost_statechart_example2_node");
  ros::NodeHandle nh;
  shared_ptr<SmStateMachine> sm = make_shared<SmStateMachine>();
  sm->initiate();
  sm->process_event(EvOne());
  sm->process_event(EvTwo());
  sm->process_event(EvOne());
  sm->process_event(EvTwo());
  sm->process_event(EvOne());
  sm->process_event(EvTwo());
  sm->process_event(EvOne());
  sm->process_event(EvTwo());
  sm->process_event(EvOne());
  sm->process_event(EvTwo());
  sm->process_event(EvOne());
  sm->process_event(EvTwo());
  sm->process_event(EvThree());
  return 0;
}