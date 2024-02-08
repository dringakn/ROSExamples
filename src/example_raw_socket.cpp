#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <pthread.h>

class RawSocketServer
{
public:
  RawSocketServer(int port) : port(port)
  {
  }

  void startServer()
  {
    // Create a socket
    sockfd = socket(AF_INET, SOCK_RAW, IPPROTO_TCP);
    if (sockfd < 0)
    {
      perror("Error creating socket");
      return;
    }

    // Set up the server address struct
    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    // Bind the socket to the specified port
    if (bind(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0)
    {
      perror("Error binding socket");
      close(sockfd);
      return;
    }

    // Create a thread to handle incoming messages
    pthread_create(&listenerThread, nullptr, &RawSocketServer::listenThread, this);

    // Join the listener thread
    pthread_join(listenerThread, nullptr);
  }

  static void* listenThread(void* obj)
  {
    RawSocketServer* server = static_cast<RawSocketServer*>(obj);
    server->listen();
    return nullptr;
  }

  void listen()
  {
    sockaddr_in clientAddr{};
    socklen_t clientLen = sizeof(clientAddr);

    while (true)
    {
      char buffer[1024];
      memset(buffer, 0, sizeof(buffer));

      // Receive data from the client
      ssize_t bytesReceived = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&clientAddr, &clientLen);

      if (bytesReceived < 0)
      {
        perror("Error receiving data");
        continue;
      }

      // Process the received data
      processMessage(buffer, bytesReceived);
    }
  }

  void processMessage(const char* buffer, ssize_t size)
  {
    // Add your message processing logic here
    // This is just a placeholder, you can modify it based on your requirements
    std::cout << "Received message: " << buffer << std::endl;
  }

  ~RawSocketServer()
  {
    // Cleanup
    close(sockfd);
  }

private:
  int port;
  int sockfd;
  pthread_t listenerThread;
};

int main()
{
  // Create a RawSocketServer instance and start the server
  RawSocketServer server(12345);  // Use your desired port number
  server.startServer();

  return 0;
}
