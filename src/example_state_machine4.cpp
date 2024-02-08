#include <iostream>
#include <string>

// Forward declaration of states
class State;

// Context class that maintains the current state
class Context
{
public:
  Context(State* initialState);

  void setState(State* newState);

  void processEvent(const std::string& event);

private:
  State* currentState;
};

// Abstract base class for states
class State
{
public:
  virtual void handleEvent(Context& context, const std::string& event) = 0;
};

// Concrete state classes
class StateA : public State
{
public:
  void handleEvent(Context& context, const std::string& event) override;
};

class StateB : public State
{
public:
  void handleEvent(Context& context, const std::string& event) override;
};

// Implementations
Context::Context(State* initialState) : currentState(initialState)
{
}

void Context::setState(State* newState)
{
  currentState = newState;
}

void Context::processEvent(const std::string& event)
{
  currentState->handleEvent(*this, event);
}

void StateA::handleEvent(Context& context, const std::string& event)
{
  if (event == "toB")
  {
    std::cout << "Transitioning from StateA to StateB" << std::endl;
    context.setState(new StateB());
    delete this;  // Release the current state object
  }
  else
  {
    std::cout << "StateA handling event: " << event << std::endl;
  }
}

void StateB::handleEvent(Context& context, const std::string& event)
{
  if (event == "toA")
  {
    std::cout << "Transitioning from StateB to StateA" << std::endl;
    context.setState(new StateA());
    delete this;  // Release the current state object
  }
  else
  {
    std::cout << "StateB handling event: " << event << std::endl;
  }
}

// Example usage
int main()
{
  State* initialState = new StateA();
  Context context(initialState);

  context.processEvent("toB");
  context.processEvent("toA");

  return 0;
}
