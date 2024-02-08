#include <iostream>
#include <thread>
#include <cstring>
#include <cstdlib>
#include <arpa/inet.h>
#include <unistd.h>

class TCPServer
{
public:
  TCPServer(int port) : port(port)
  {
  }

  void startServer()
  {
    // Create socket
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1)
    {
      perror("Error creating socket");
      exit(EXIT_FAILURE);
    }

    // Initialize server address structure
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = INADDR_ANY;
    serverAddress.sin_port = htons(port);

    // Bind the socket
    if (bind(serverSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == -1)
    {
      perror("Error binding socket");
      exit(EXIT_FAILURE);
    }

    // Listen for incoming connections
    if (listen(serverSocket, 5) == -1)
    {
      perror("Error listening for connections");
      exit(EXIT_FAILURE);
    }

    std::cout << "Server listening on port " << port << std::endl;

    while (true)
    {
      // Accept a connection
      int clientSocket = accept(serverSocket, nullptr, nullptr);
      if (clientSocket == -1)
      {
        perror("Error accepting connection");
        continue;
      }

      // Start a new thread to handle the client
      std::thread clientThread(&TCPServer::handleClient, this, clientSocket);
      clientThread.detach();  // Detach the thread to let it run independently
    }
  }

private:
  int port;
  int serverSocket;
  struct sockaddr_in serverAddress;

  void handleClient(int clientSocket)
  {
    char buffer[1024];
    ssize_t bytesRead;

    // Receive data from the client
    while ((bytesRead = recv(clientSocket, buffer, sizeof(buffer), 0)) > 0)
    {
      // Process the received data (here, just print it)
      buffer[bytesRead] = '\0';
      std::cout << "Received from client: " << buffer << std::endl;

      // Echo the data back to the client
      send(clientSocket, buffer, bytesRead, 0);
    }

    // Close the client socket when done
    close(clientSocket);
  }
};

class TCPClient
{
public:
  TCPClient(const std::string& serverIP, int serverPort) : serverIP(serverIP), serverPort(serverPort)
  {
  }

  void startClient()
  {
    // Create socket
    clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1)
    {
      perror("Error creating socket");
      exit(EXIT_FAILURE);
    }

    // Initialize server address structure
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(serverPort);

    if (inet_pton(AF_INET, serverIP.c_str(), &serverAddress.sin_addr) <= 0)
    {
      perror("Invalid server address");
      exit(EXIT_FAILURE);
    }

    // Connect to the server
    if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == -1)
    {
      perror("Error connecting to server");
      exit(EXIT_FAILURE);
    }

    // Send data to the server
    const char* message = "Hello, server!";
    send(clientSocket, message, strlen(message), 0);

    // Receive and print the echoed data
    char buffer[1024];
    ssize_t bytesRead = recv(clientSocket, buffer, sizeof(buffer), 0);
    buffer[bytesRead] = '\0';
    std::cout << "Received from server: " << buffer << std::endl;

    // Close the socket
    close(clientSocket);
  }

private:
  std::string serverIP;
  int serverPort;
  int clientSocket;
  struct sockaddr_in serverAddress;
};

int main()
{
  const int port = 1234;

  // Start server in a separate thread
  TCPServer server(port);
  std::thread serverThread(&TCPServer::startServer, &server);

  usleep(1 * 1e6);

  // Start client
  TCPClient client("127.0.0.1", port);
  client.startClient();

  TCPClient client1("127.0.0.1", port);
  client1.startClient();

  // Wait for the server thread to finish
  serverThread.join();

  return 0;
}
