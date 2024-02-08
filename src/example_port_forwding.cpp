#include <iostream>
#include <thread>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>

class PortForwarder
{
public:
  PortForwarder(int sourcePort, int destinationPort) : sourcePort(sourcePort), destinationPort(destinationPort)
  {
    // Initialize server and client sockets
    serverSocket = createSocket();
    clientSocket = createSocket();

    // Bind the server socket to the source port
    bindSocket(serverSocket, sourcePort);

    // Start listening for incoming connections
    listenForConnections(serverSocket);

    // Accept a client connection
    acceptConnection(serverSocket, clientSocket);

    // Connect to the destination port
    connectToDestination(destinationPort);

    // Create threads for bidirectional communication
    std::thread thread1(&PortForwarder::forward, this, clientSocket, destinationSocket);
    std::thread thread2(&PortForwarder::forward, this, destinationSocket, clientSocket);

    // Wait for both threads to finish
    thread1.join();
    thread2.join();

    // Close sockets
    close(serverSocket);
    close(clientSocket);
    close(destinationSocket);
  }

private:
  int sourcePort;
  int destinationPort;
  int serverSocket;
  int clientSocket;
  int destinationSocket;

  int createSocket()
  {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1)
    {
      perror("Error creating socket");
      exit(EXIT_FAILURE);
    }
    return sockfd;
  }

  void bindSocket(int sockfd, int port)
  {
    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(port);

    if (bind(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1)
    {
      perror("Error binding socket");
      exit(EXIT_FAILURE);
    }
  }

  void listenForConnections(int sockfd)
  {
    if (listen(sockfd, 1) == -1)
    {
      perror("Error listening for connections");
      exit(EXIT_FAILURE);
    }
  }

  void acceptConnection(int serverSock, int& clientSock)
  {
    sockaddr_in clientAddr;
    socklen_t addrLen = sizeof(clientAddr);
    clientSock = accept(serverSock, (struct sockaddr*)&clientAddr, &addrLen);
    if (clientSock == -1)
    {
      perror("Error accepting connection");
      exit(EXIT_FAILURE);
    }
  }

  void connectToDestination(int destinationPort)
  {
    destinationSocket = createSocket();

    sockaddr_in destAddr;
    destAddr.sin_family = AF_INET;
    destAddr.sin_addr.s_addr = INADDR_ANY;
    destAddr.sin_port = htons(destinationPort);

    if (connect(destinationSocket, (struct sockaddr*)&destAddr, sizeof(destAddr)) == -1)
    {
      perror("Error connecting to destination");
      exit(EXIT_FAILURE);
    }
  }

  void forward(int sourceSocket, int destinationSocket)
  {
    char buffer[1024];
    ssize_t bytesRead;

    while ((bytesRead = recv(sourceSocket, buffer, sizeof(buffer), 0)) > 0)
    {
      send(destinationSocket, buffer, bytesRead, 0);
    }

    if (bytesRead == -1)
    {
      perror("Error receiving data");
    }
  }
};

int main()
{
  int sourcePort = 5555;     // The source port number
  int destinationPort = 22;  // The destination port number

  PortForwarder forwarder(sourcePort, destinationPort);

  return 0;
}
