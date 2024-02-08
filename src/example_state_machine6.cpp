#include <iostream>
#include <thread>
#include <chrono>

// Define the states of the coffee machine
enum class CoffeeMachineState
{
  Idle,
  Brewing,
  Error
};

// Define the events that trigger state transitions
enum class CoffeeMachineEvent
{
  StartBrew,
  FinishBrew,
  ErrorDetected,
  Timeout
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
        if (event == CoffeeMachineEvent::StartBrew)
        {
          currentState = CoffeeMachineState::Brewing;
          startBrewTimer();
          std::cout << "Coffee machine is now brewing.\n";
        }
        else if (event == CoffeeMachineEvent::ErrorDetected)
        {
          currentState = CoffeeMachineState::Error;
          std::cout << "Error detected. Coffee machine is in an error state.\n";
        }
        break;

      case CoffeeMachineState::Brewing:
        if (event == CoffeeMachineEvent::FinishBrew)
        {
          stopBrewTimer();
          currentState = CoffeeMachineState::Idle;
          std::cout << "Brewing complete. Coffee machine is now idle.\n";
        }
        else if (event == CoffeeMachineEvent::Timeout)
        {
          currentState = CoffeeMachineState::Error;
          std::cout << "Timeout! Brewing took too long. Coffee machine is in an error state.\n";
        }
        else if (event == CoffeeMachineEvent::ErrorDetected)
        {
          stopBrewTimer();
          currentState = CoffeeMachineState::Error;
          std::cout << "Error detected. Coffee machine is in an error state.\n";
        }
        break;

      case CoffeeMachineState::Error:
        // Handle any recovery logic or user intervention here
        std::cout << "Error state. Please perform necessary actions for recovery.\n";
        break;
    }
  }

private:
  // Current state of the coffee machine
  CoffeeMachineState currentState;

  // Timer variables
  std::chrono::time_point<std::chrono::system_clock> startTime;
  bool timerRunning = false;

  // Function to start the brew timer
  void startBrewTimer()
  {
    startTime = std::chrono::system_clock::now();
    timerRunning = true;

    // Simulate a 10-second brewing time
    std::thread([this]() {
      std::this_thread::sleep_for(std::chrono::seconds(10));
      if (timerRunning)
      {
        handleEvent(CoffeeMachineEvent::Timeout);
      }
    }).detach();
  }

  // Function to stop the brew timer
  void stopBrewTimer()
  {
    timerRunning = false;
  }
};

int main()
{
  // Create a coffee machine object
  CoffeeMachine coffeeMachine;

  // Simulate some transitions
  coffeeMachine.handleEvent(CoffeeMachineEvent::StartBrew);
  std::this_thread::sleep_for(std::chrono::seconds(15));
  coffeeMachine.handleEvent(CoffeeMachineEvent::FinishBrew);

  return 0;
}
