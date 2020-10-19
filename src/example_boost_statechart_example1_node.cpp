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

struct EvFour : sc::event<EvFour> {
  EvFour() { cout << "EvFour" << endl; }
};

struct EvFive : sc::event<EvFive> {
  EvFive() { cout << "EvFive" << endl; }
};

struct EvSix : sc::event<EvSix> {
  EvSix() { cout << "EvSix" << endl; }
};

struct StOne;
struct StTwo;
struct StThree;
struct StFour;

struct SmStateMachine : sc::state_machine<SmStateMachine, StOne> {
  SmStateMachine() { cout << "SmStateMachine" << endl; }
  ~SmStateMachine() { cout << "~SmStateMachine" << endl; }
};

struct StOne : sc::simple_state<StOne, SmStateMachine> {
  StOne() { cout << "StOne" << endl; }
  // typedef sc::transition<EvOne, StTwo> reactions; // Single event
  typedef mpl::list<sc::transition<EvTwo, StTwo>,
                    sc::transition<EvThree, StThree>,
                    sc::custom_reaction<EvFour>, sc::custom_reaction<EvFive>,
                    sc::custom_reaction<EvSix> >
      reactions;  // Multiple events
  sc::result react(const EvFour& e) { return transit<StFour>(); }
  sc::result react(const EvFive& e) { return discard_event(); }
  sc::result react(const EvSix& e) { return discard_event(); }
};

struct StTwo : sc::simple_state<StTwo, SmStateMachine> {
  StTwo() { cout << "StTwo" << endl; }
  typedef sc::transition<EvOne, StOne> reactions;
};

struct StThree : sc::simple_state<StThree, SmStateMachine> {
  StThree() { cout << "StThree" << endl; }
  typedef sc::transition<EvOne, StOne> reactions;
};

struct StFour : sc::simple_state<StFour, SmStateMachine> {
  StFour() { cout << "StFour" << endl; }
  typedef sc::transition<EvOne, StOne> reactions;
};

/*
./bosce
~/test_ws/devel/lib/test_boost_statechart_example/test_boost_statechart_example1_node
-s SmStateMachine > uml.pu && plantuml uml.pu && xdg-open uml.png
*/

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_boost_statechart_example1_node");
  ros::NodeHandle nh;
  shared_ptr<SmStateMachine> sm = make_shared<SmStateMachine>();
  sm->initiate();
  sm->process_event(EvOne());
  sm->process_event(EvTwo());
  sm->process_event(EvThree());
  sm->process_event(EvThree());
  sm->process_event(EvOne());
  sm->process_event(EvFour());
  sm->process_event(EvFive());
  sm->process_event(EvSix());
  sm->process_event(EvOne());
  return 0;
}