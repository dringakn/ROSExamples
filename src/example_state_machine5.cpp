#include <iostream>

// Define the states of the coffee machine
enum class CoffeeMachineState
{
  Idle,
  Brewing,
  AddingIngredients
};

// Define the events that trigger state transitions
enum class CoffeeMachineEvent
{
  StartBrewing,
  AddMilk,
  AddSugar,
  FinishBrewing
};

// Define the CoffeeMachine class
class CoffeeMachine
{
public:
  // Constructor with initial state
  CoffeeMachine() : currentState(CoffeeMachineState::Idle)
  {
  }

  // Function to handle events and transition to the new state
  void handleEvent(CoffeeMachineEvent event)
  {
    switch (currentState)
    {
      case CoffeeMachineState::Idle:
        if (event == CoffeeMachineEvent::StartBrewing)
        {
          currentState = CoffeeMachineState::Brewing;
          std::cout << "Coffee machine is now Brewing.\n";
        }
        break;

      case CoffeeMachineState::Brewing:
        if (event == CoffeeMachineEvent::AddMilk)
        {
          currentState = CoffeeMachineState::AddingIngredients;
          std::cout << "Adding Milk to the coffee.\n";
        }
        else if (event == CoffeeMachineEvent::FinishBrewing)
        {
          currentState = CoffeeMachineState::Idle;
          std::cout << "Coffee brewing is finished. Coffee machine is now Idle.\n";
        }
        break;

      case CoffeeMachineState::AddingIngredients:
        if (event == CoffeeMachineEvent::AddSugar)
        {
          currentState = CoffeeMachineState::Brewing;
          std::cout << "Adding Sugar to the coffee.\n";
        }
        break;
    }
  }

private:
  // Current state of the coffee machine
  CoffeeMachineState currentState;
};

int main()
{
  // Create a coffee machine object
  CoffeeMachine coffeeMachine;

  // Simulate some transitions
  coffeeMachine.handleEvent(CoffeeMachineEvent::StartBrewing);
  coffeeMachine.handleEvent(CoffeeMachineEvent::AddMilk);
  coffeeMachine.handleEvent(CoffeeMachineEvent::AddSugar);
  coffeeMachine.handleEvent(CoffeeMachineEvent::FinishBrewing);

  return 0;
}
