#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>

class UDPServer
{
public:
  UDPServer(int port) : port(port)
  {
    // Create socket
    serverSocket = socket(AF_INET, SOCK_DGRAM, 0);

    // Initialize server address structure
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(port);

    // Bind socket
    bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
  }

  void receive()
  {
    char buffer[1024];
    socklen_t addrLen = sizeof(clientAddr);

    // Receive data
    ssize_t bytesRead = recvfrom(serverSocket, buffer, sizeof(buffer), 0, (struct sockaddr*)&clientAddr, &addrLen);

    // Null-terminate the received data
    buffer[bytesRead] = '\0';

    std::cout << "Received from client: " << buffer << std::endl;
  }

  ~UDPServer()
  {
    close(serverSocket);
  }

private:
  int serverSocket;
  int port;
  struct sockaddr_in serverAddr, clientAddr;
};

class UDPClient
{
public:
  UDPClient(int port) : port(port)
  {
    // Create socket
    clientSocket = socket(AF_INET, SOCK_DGRAM, 0);

    // Initialize server address structure
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    inet_pton(AF_INET, "127.0.0.1", &(serverAddr.sin_addr));
  }

  void send(const char* message)
  {
    // Send data
    sendto(clientSocket, message, strlen(message), 0, (struct sockaddr*)&serverAddr, sizeof(serverAddr));
    std::cout << "Sent to server: " << message << std::endl;
  }

  ~UDPClient()
  {
    close(clientSocket);
  }

private:
  int clientSocket;
  int port;
  struct sockaddr_in serverAddr;
};

int main()
{
  const int port = 8888;

  // Start the server
  UDPServer server(port);

  // Start the client
  UDPClient client(port);

  // Send a message from client to server
  client.send("Hello, server!");

  // Receive the message on the server
  server.receive();

  return 0;
}
