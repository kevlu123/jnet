#include <iostream>
#include <thread>
#include <chrono>
#include "jnet.h"

using namespace jnet;
using namespace std::chrono_literals;

struct Server : public JServer {

	using JServer::JServer;

	bool OnClientConnect(ExternalJClient client) noexcept override {
		std::cout << client << " connected" << std::endl;
		return true;
	}

	void OnClientDisconnect(ExternalJClient client) noexcept override {
		std::cout << client << " disconnected" << std::endl;
	}

	void OnReceive(ExternalJClient client, Json& j) noexcept override {
		std::cout << client << " message: " << j.dump(1) << std::endl;
		Send(client, "hello client");
	}

};

struct Client : public LocalJClient {

	using LocalJClient::LocalJClient;

	void OnReceive(Json& j) noexcept override {
		received = true;
		std::cout << "message" << std::endl;
		std::cout << j.dump(1) << std::endl;
	}

	bool received = false;
};


void PrintUsage() {
	std::cout << "Usage: [server|client]" << std::endl;
}

int main(int argc, char** argv) {

	if (argc < 2) {
		PrintUsage();
		return -1;
	}

	if (std::strcmp(argv[1], "server") == 0) {

		try {
			Server server(42942);
			while (true) {
				server.SendAll("hello all");
				server.Update();
				std::this_thread::sleep_for(100ms);
			}
		} catch (jnet::Exception& ex) {
			std::cout << "Error: " << ex.what() << std::endl;
		}

	} else if (std::strcmp(argv[1], "client") == 0) {

		try {
			Client client("localhost", 42942);
			client.Send("hello server");
			while (!client.received) {
				client.Update();
				std::this_thread::sleep_for(100ms);
			}
		} catch (jnet::Exception& ex) {
			std::cout << "Error: " << ex.what() << std::endl;
		}

	} else {
		PrintUsage();
		return -2;
	}

	return 0;
}
