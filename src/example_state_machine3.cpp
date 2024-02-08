#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/transition.hpp>
#include <iostream>

namespace sc = boost::statechart;

// Define states
struct State1;
struct State2;

// Define events
struct Event1 : sc::event<Event1>
{
};
struct Event2 : sc::event<Event2>
{
};

// Define state machine
struct StateMachine : sc::state_machine<StateMachine, State1>
{
};

// Define State1
struct State1 : sc::simple_state<State1, StateMachine>
{
  State1()
  {
    std::cout << "Entering State1\n";
  }
  ~State1()
  {
    std::cout << "Leaving State1\n";
  }

  typedef sc::transition<Event1, State2> reactions;
};

// Define State2
struct State2 : sc::simple_state<State2, StateMachine>
{
  State2()
  {
    std::cout << "Entering State2\n";
  }
  ~State2()
  {
    std::cout << "Leaving State2\n";
  }

  typedef sc::transition<Event2, State1> reactions;
};

int main()
{
  StateMachine stateMachine;
  stateMachine.initiate();

  stateMachine.process_event(Event1());
  stateMachine.process_event(Event2());

  return 0;
}
