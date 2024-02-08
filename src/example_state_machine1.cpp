#include <iostream>
#include <chrono>
#include <thread>

// Define the states of the traffic light
enum class TrafficLightState
{
  Red,
  Yellow,
  Green
};

// Define the events that trigger state transitions
enum class TrafficLightEvent
{
  SwitchToGreen,
  SwitchToYellow,
  SwitchToRed,
  Timeout
};

// Define the TrafficLight class
class TrafficLight
{
public:
  // Constructor with initial state
  TrafficLight() : currentState(TrafficLightState::Red)
  {
  }

  // Function to handle events and transition to the new state
  void handleEvent(TrafficLightEvent event)
  {
    switch (currentState)
    {
      case TrafficLightState::Red:
        if (event == TrafficLightEvent::SwitchToGreen)
        {
          currentState = TrafficLightState::Green;
          std::cout << "Traffic Light is now Green.\n";
          startTimer(5000);  // Transition to the next state after 5 seconds (5000 milliseconds)
        }
        else if (event == TrafficLightEvent::Timeout)
        {
          std::cout << "Timeout event.\n";
        }
        break;

      case TrafficLightState::Yellow:
        if (event == TrafficLightEvent::SwitchToRed)
        {
          currentState = TrafficLightState::Red;
          std::cout << "Traffic Light is now Red.\n";
          startTimer(3000);  // Transition to the next state after 3 seconds (3000 milliseconds)
        }
        else if (event == TrafficLightEvent::Timeout)
        {
          std::cout << "Timeout event.\n";
        }

        break;

      case TrafficLightState::Green:
        if (event == TrafficLightEvent::SwitchToYellow)
        {
          currentState = TrafficLightState::Yellow;
          std::cout << "Traffic Light is now Yellow.\n";
          startTimer(2000);  // Transition to the next state after 2 seconds (2000 milliseconds)
        }
        else if (event == TrafficLightEvent::Timeout)
        {
          std::cout << "Timeout event.\n";
        }
        break;

      default:
        if (event == TrafficLightEvent::Timeout)
        {
          std::cout << "Unknown state.\n";
        }
        break;
    }
  }

  // Function to simulate a timer
  void startTimer(int milliseconds)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
    handleEvent(TrafficLightEvent::Timeout);
  }

private:
  // Current state of the traffic light
  TrafficLightState currentState;
};

int main()
{
  // Create a traffic light object
  TrafficLight trafficLight;

  // Simulate some transitions with timeouts
  trafficLight.handleEvent(TrafficLightEvent::SwitchToGreen);
  trafficLight.handleEvent(TrafficLightEvent::SwitchToYellow);
  trafficLight.handleEvent(TrafficLightEvent::SwitchToRed);

  return 0;
}
