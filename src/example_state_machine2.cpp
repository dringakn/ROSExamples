#include <iostream>
#include <memory>

class State
{
public:
  virtual void handleInput() = 0;
  virtual void update() = 0;
  virtual void enter() = 0;
  virtual void exit() = 0;
  virtual ~State()
  {
  }
};

class IdleState : public State
{
public:
  void handleInput() override
  {
    std::cout << "IdleState: Handling Input\n";
  }

  void update() override
  {
    std::cout << "IdleState: Updating\n";
  }

  void enter() override
  {
    std::cout << "IdleState: Entering\n";
  }

  void exit() override
  {
    std::cout << "IdleState: Exiting\n";
  }
};

class RunningState : public State
{
public:
  void handleInput() override
  {
    std::cout << "RunningState: Handling Input\n";
  }

  void update() override
  {
    std::cout << "RunningState: Updating\n";
  }

  void enter() override
  {
    std::cout << "RunningState: Entering\n";
  }

  void exit() override
  {
    std::cout << "RunningState: Exiting\n";
  }
};

class StateMachine
{
public:
  void setState(std::unique_ptr<State> newState)
  {
    if (currentState)
      currentState->exit();

    currentState = std::move(newState);
    currentState->enter();
  }

  void handleInput()
  {
    if (currentState)
      currentState->handleInput();
  }

  void update()
  {
    if (currentState)
      currentState->update();
  }

private:
  std::unique_ptr<State> currentState;
};

int main(int argc, char** argv)
{
  StateMachine stateMachine;

  stateMachine.setState(std::make_unique<IdleState>());
  stateMachine.handleInput();
  stateMachine.update();

  stateMachine.setState(std::make_unique<RunningState>());
  stateMachine.handleInput();
  stateMachine.update();

  return 0;
}
