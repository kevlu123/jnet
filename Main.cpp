#include <iostream>
#include "jnet.h"

using namespace jnet;

struct Server : public JServer {

	Server(uint16_t port) : JServer(port) {}

	bool OnClientConnect(ExternalJClient client) noexcept override {
		std::cout << client << " connected" << std::endl;
	}

	void OnClientDisconnect(ExternalJClient client) noexcept override {
		std::cout << client << " disconnected" << std::endl;
	}

	void OnReceive(ExternalJClient client, Json& j) noexcept override {
		std::cout << client << " message" << std::endl;
		std::cout << j.dump(1) << std::endl;
	}

};

struct Client : public LocalJClient {

	Client(const std::string& host, uint16_t port) : LocalJClient(host, port) {}

	void OnReceive(Json& j) noexcept override {
		std::cout << "message" << std::endl;
		std::cout << j.dump(1) << std::endl;
	}

};


int main() {



}
