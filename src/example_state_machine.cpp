#include <iostream>

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
  SwitchToRed
};

// Define the TrafficLight class representing the state machine
class TrafficLight
{
public:
  // Constructor with the initial state set to Red
  TrafficLight() : currentState(TrafficLightState::Red)
  {
  }

  // Function to trigger state transitions based on events
  void handleEvent(TrafficLightEvent event)
  {
    switch (currentState)
    {
      case TrafficLightState::Red:
        if (event == TrafficLightEvent::SwitchToGreen)
        {
          currentState = TrafficLightState::Green;
          std::cout << "Switching to Green\n";
        }
        break;

      case TrafficLightState::Green:
        if (event == TrafficLightEvent::SwitchToYellow)
        {
          currentState = TrafficLightState::Yellow;
          std::cout << "Switching to Yellow\n";
        }
        break;

      case TrafficLightState::Yellow:
        if (event == TrafficLightEvent::SwitchToRed)
        {
          currentState = TrafficLightState::Red;
          std::cout << "Switching to Red\n";
        }
        break;
    }
  }

private:
  TrafficLightState currentState;
};

int main(int argc, char** argv)
{
  // Create an instance of TrafficLight
  TrafficLight trafficLight;

  // Simulate traffic light transitions
  trafficLight.handleEvent(TrafficLightEvent::SwitchToGreen);
  trafficLight.handleEvent(TrafficLightEvent::SwitchToYellow);
  trafficLight.handleEvent(TrafficLightEvent::SwitchToRed);

  return 0;
}
