#include <iostream>
#include <thread>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <arpa/inet.h>

/*
CommandLine Testing:
Terminal1: nc -l -p 12345 < myfifo | /bin/bash > myfifo
Terminal2: echo "pwd" | nc localhost 12345
Terminal3: nc localhost 12345

Compaliation: g++ -std=c++17  src/example_udp_remote_bash.cpp -lboost_system -lpthread
Terminal1: sudo ./a.out
Terminal2: echo "ls -l" | nc -u localhost 12345
Terminal2: nc -u localhost 12345
*/

// Define a class for the server
class UDPServer
{
private:
  int serverSocket;
  struct sockaddr_in serverAddr, clientAddr;
  socklen_t clientAddrLen;

public:
  UDPServer(int port)
  {
    // Create socket
    serverSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (serverSocket == -1)
    {
      std::cerr << "Error creating socket" << std::endl;
      exit(EXIT_FAILURE);
    }

    // Set up server address
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(port);

    // Bind the socket
    if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1)
    {
      std::cerr << "Error binding socket" << std::endl;
      close(serverSocket);
      exit(EXIT_FAILURE);
    }

    std::cout << "Server listening on port " << port << std::endl;

    // Set client address length
    clientAddrLen = sizeof(clientAddr);
  }

  void receiveAndReply()
  {
    char buffer[65535];
    while (true)
    {
      // Receive data from client: Flags: MSG_DONTWAIT
      ssize_t recvLen =
          recvfrom(serverSocket, buffer, sizeof(buffer), 0, (struct sockaddr*)&clientAddr, &clientAddrLen);
      if (recvLen == -1)
      {
        perror("Error:");
        // usleep(1000);
        continue;
      }
      else
      {
        std::cout << "Recieved data: " << recvLen << std::endl;
      }
      // Null-terminate the received data
      buffer[recvLen] = '\0';

      // Execute the received command and get the output
      std::string commandOutput = executeCommand(buffer);

      // Send the command output back to the client
      ssize_t sendLen = sendto(serverSocket, commandOutput.c_str(), commandOutput.length(), 0,
                               (struct sockaddr*)&clientAddr, clientAddrLen);
      if (sendLen == -1)
      {
        std::cerr << "Error sending data" << std::endl;
      }
    }
  }

private:
  std::string executeCommand(const char* command)
  {
    std::string result;

    // Source the bashrc to ensure that the environment is set up correctly
    std::string sourceBashrc = ". ~/.bashrc";
    FILE* sourcePipe = popen(sourceBashrc.c_str(), "r");
    pclose(sourcePipe);

    // Execute the command within the terminal
    std::string terminalCommand = "/bin/bash -c '";
    terminalCommand += command;
    terminalCommand += "'";
    FILE* pipe = popen(terminalCommand.c_str(), "r");

    if (!pipe)
    {
      std::cerr << "Error executing command" << std::endl;
      return "Error executing command";
    }

    char buffer[128];
    while (fgets(buffer, sizeof(buffer), pipe) != NULL)
    {
      result += buffer;
    }

    pclose(pipe);

    return result;
  }
};

int main()
{
  const int PORT = 12345;

  // Create and run the server in a separate thread
  UDPServer server(PORT);
  std::thread serverThread(&UDPServer::receiveAndReply, &server);

  // Wait for the user to press enter to terminate the program
  std::cout << "Press enter to exit." << std::endl;
  std::cin.get();

  // Join the server thread and exit
  serverThread.join();

  return 0;
}
