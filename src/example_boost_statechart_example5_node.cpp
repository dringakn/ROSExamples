/*
./bosce
~/test_ws/devel/lib/test_boost_statechart_example/test_boost_statechart_example5_node
-s SmStateMachine > uml.pu && plantuml uml.pu && xdg-open uml.png

    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description: Boost start chart example.
    NOTE : The context<>() can't be used inside the constructor / destructors of
           the state.
       - Release mode doesn't shows the correct diagram using bosce.
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
        - History works only on one region.
        - bosce cann't display correctly the histroy state.

*/

#include <ros/ros.h>

#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/deep_history.hpp>
#include <boost/statechart/deferral.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/shallow_history.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/transition.hpp>

using namespace std;
namespace sc = boost::statechart;
namespace mpl = boost::mpl;

struct EvOne : sc::event<EvOne> {
  EvOne() {}
};

struct EvTwo : sc::event<EvTwo> {
  EvTwo() {}
};

struct EvThree : sc::event<EvThree> {
  EvThree() {}
};

struct EvFour : sc::event<EvFour> {
  EvFour() {}
};

struct EvFive : sc::event<EvFive> {
  EvFive() {}
};

struct EvSix : sc::event<EvSix> {
  EvSix() {}
};

struct EvSeven : sc::event<EvSeven> {
  EvSeven() {}
};

struct EvEight : sc::event<EvEight> {
  EvEight() {}
};

struct StOne;
struct StOneOne;
struct StOneTwo;
struct StOneThree;
struct StOneFour;
struct StTwo;
struct StTwoOne;
struct StTwoTwo;

struct SmStateMachine : sc::state_machine<SmStateMachine, StOne> {
  int data;
  SmStateMachine() : data(0) { cout << "SmStateMachine: " << endl; }
  ~SmStateMachine() { cout << "~SmStateMachine:" << endl; }
};

struct StOne
    : sc::simple_state<StOne, SmStateMachine, mpl::list<StOneOne, StOneThree>,
                       sc::has_deep_history> {
  StOne() {}
  typedef sc::transition<EvOne, sc::deep_history<StTwoOne>> reactions;
};

struct StOneOne : sc::simple_state<StOneOne, StOne::orthogonal<0>> {
  StOneOne() {}
  typedef mpl::list<sc::transition<EvThree, StOneTwo>> reactions;
};

struct StOneTwo : sc::simple_state<StOneTwo, StOne::orthogonal<0>> {
  StOneTwo() {}
  typedef mpl::list<sc::transition<EvFour, StOneOne>> reactions;
};

struct StOneThree : sc::simple_state<StOneThree, StOne::orthogonal<1>> {
  StOneThree() {}
  typedef mpl::list<sc::transition<EvFive, StOneFour>> reactions;
};

struct StOneFour : sc::simple_state<StOneFour, StOne::orthogonal<1>> {
  StOneFour() {}
  typedef mpl::list<sc::transition<EvSix, StOneThree>> reactions;
};

struct StTwo : sc::simple_state<StTwo, SmStateMachine, mpl::list<StTwoOne>,
                                sc::has_deep_history> {
  StTwo() {}
  typedef sc::transition<EvTwo, sc::deep_history<StOneOne>> reactions;
};

struct StTwoOne : sc::simple_state<StTwoOne, StTwo::orthogonal<0>> {
  StTwoOne() {}
  typedef mpl::list<sc::transition<EvSeven, StTwoTwo>> reactions;
};

struct StTwoTwo : sc::simple_state<StTwoTwo, StTwo::orthogonal<0>> {
  StTwoTwo() {}
  typedef mpl::list<sc::transition<EvEight, StTwoOne>> reactions;
};

void getKeyboardEvents(shared_ptr<SmStateMachine>& sm) {
  int x = 0;
  do {
    switch (x) {
      case 1:
        sm->process_event(EvOne());
        break;
      case 2:
        sm->process_event(EvTwo());
        break;
      case 3:
        sm->process_event(EvThree());
        break;
      case 4:
        sm->process_event(EvFour());
        break;
      case 5:
        sm->process_event(EvFive());
        break;
      case 6:
        sm->process_event(EvSix());
        break;
      case 7:
        sm->process_event(EvSeven());
        break;
      case 8:
        sm->process_event(EvEight());
        break;
      default:
        cout << "Ctrl+z to exit." << endl;
        // sm->state_begin()
        break;
    }
    for (auto st = sm->state_begin(); st != sm->state_end(); ++st)
      // If state::custom_static_type_ptr("") is not initialized
      // cout << "Current State: " << typeid(*st).name() << endl;
      // Otherwise;
      cout << "Current State: " << st->custom_dynamic_type_ptr<char>() << endl;
    cout << "Enter Event(1:One, 2:Two, 3:Three, 4:Four, 5:Five, 6:Six, "
            "7:Seven, 8:Eight):";
  } while (cin >> x);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example_boost_statechart_example5_node");
  ros::NodeHandle nh;
  shared_ptr<SmStateMachine> sm = make_shared<SmStateMachine>();
  StOne::custom_static_type_ptr("One");
  StTwo::custom_static_type_ptr("Two");
  StOneOne::custom_static_type_ptr("OneOne");
  StOneTwo::custom_static_type_ptr("OneTwo");
  StOneThree::custom_static_type_ptr("OneThree");
  StOneFour::custom_static_type_ptr("OneFour");
  StTwoOne::custom_static_type_ptr("TwoOne");
  StTwoTwo::custom_static_type_ptr("TwoTwo");

  sm->initiate();
  getKeyboardEvents(sm);
  return 0;
}