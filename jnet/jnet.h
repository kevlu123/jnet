#pragma once
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <algorithm>
#include <cstdint>
#include <stdexcept>

#include "jnet/json.h"

#ifdef _WIN32
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0A00
#endif
#endif

#define ASIO_STANDALONE
#include "jnet/asio.hpp"
#include "jnet/asio/ts/buffer.hpp"
#include "jnet/asio/ts/internet.hpp"


namespace jnet {

	using Json = nlohmann::json;
	using ExternalJClient = uint32_t;
	using Bytes = std::vector<uint8_t>;

	struct Exception : public std::runtime_error {
		Exception(const std::string& msg = "") : runtime_error(msg) {}
		Exception(const asio::error_code& ec) : runtime_error(ec.message()) {}
	};

	struct JServer {

		JServer(uint16_t port);
		virtual ~JServer();

		void Update();
		void Send(ExternalJClient client, const Json& j);
		void SendAll(const Json& j);
		void DisconnectClient(ExternalJClient client);
		uint16_t GetPort() const;

		virtual bool OnClientConnect(ExternalJClient client) { return true; }
		virtual void OnClientDisconnect(ExternalJClient client) {}
		virtual void OnReceive(ExternalJClient client, Json& j) {}

		std::atomic_uint32_t maxClients = 0xFFFFFFFF;

	private:

		struct Client {
			Client(asio::ip::tcp::socket&& socket) : socket(std::move(socket)) {}
			asio::ip::tcp::socket socket;
			Bytes dataRecv;
			Bytes dataSend;
		};

		asio::io_context context;
		asio::ip::tcp::acceptor acceptor;
		std::thread acceptorThrd;
		std::thread msgThrd;
		std::mutex mtx;
		std::atomic_bool disconnectPending = false;

		std::vector<ExternalJClient> newConnections;
		std::vector<ExternalJClient> disconnections;
		std::unordered_map<ExternalJClient, Client> clients;
		ExternalJClient nextID = 1;
		uint16_t port = 0;
	};

	struct LocalJClient {

		LocalJClient(const std::string& host, uint16_t port);
		virtual ~LocalJClient();

		void Update();
		void Send(const Json& j);
		void Disconnect();

		virtual void OnDisconnect() {}
		virtual void OnReceive(Json& j) {}

	private:

		asio::io_context context;
		asio::ip::tcp::socket socket;
		std::thread thrd;
		std::mutex mtx;
		std::atomic_bool disconnectPending = false;

		bool connected = false;
		Bytes dataRecv;
		Bytes dataSend;
	};



	// Implementation

	namespace internal {

		// Message length is little-endian

		inline bool BytesToJson(const Bytes& bytes, Json& j, size_t& msgLen) {
			msgLen = 0;
			if (bytes.size() < 4)
				return false;

			// Bytes excluding message length
			uint32_t bytesExpected =
				(bytes[0] <<  0) |
				(bytes[1] <<  8) |
				(bytes[2] << 16) |
				(bytes[3] << 24);

			// Bytes including message length
			size_t totalLen = bytesExpected + sizeof(uint32_t);

			if (bytes.size() < totalLen)
				return false;

			msgLen = totalLen;
			auto msgStart = bytes.begin() + sizeof(uint32_t);
			j = Json::parse(msgStart, msgStart + bytesExpected, nullptr, false);
			return !j.is_discarded();
		}

		inline Bytes JsonToBytes(const Json& j) {
			std::string s = j.dump(-1, 32, true);

			// Encode message length
			uint32_t len = (uint32_t)s.size();
			Bytes msg = {
				static_cast<uint8_t>((len >>  0) & 0xFF),
				static_cast<uint8_t>((len >>  8) & 0xFF),
				static_cast<uint8_t>((len >> 16) & 0xFF),
				static_cast<uint8_t>((len >> 24) & 0xFF),
			};

			// Append message
			msg.insert(msg.end(), s.begin(), s.end());
			return msg;
		}

		inline void CheckError(const asio::error_code& ec) {
			if (ec)
				throw Exception(ec);
		}

		template <class Callback>
		void ProcessDataReceived(Bytes& dataRecv, Callback cb) {
			std::vector<Json> messages;
			Json j;
			size_t msgLen = 0;
			while (BytesToJson(dataRecv, j, msgLen)) {
				messages.push_back(std::move(j));
				j.clear();
				dataRecv.erase(dataRecv.begin(), dataRecv.begin() + msgLen);
			}
			dataRecv.erase(dataRecv.begin(), dataRecv.begin() + msgLen);

			for (auto& msg : messages)
				cb(msg);
		}

		inline void ReadSocket(asio::ip::tcp::socket& socket, Bytes& buf) {
			asio::error_code ec{};
			size_t bytesAvailable = socket.available(ec);
			CheckError(ec);
			if (bytesAvailable) {
				Bytes tmp(bytesAvailable);
				size_t bytesRead = socket.read_some(asio::buffer(tmp.data(), tmp.size()), ec);
				CheckError(ec);
				buf.insert(buf.end(), tmp.data(), tmp.data() + bytesRead);
			}
		}

		inline void WriteSocket(asio::ip::tcp::socket& socket, Bytes& buf) {
			asio::error_code ec{};
			size_t bytesSent = socket.write_some(asio::buffer(buf.data(), buf.size()), ec);
			buf.erase(buf.begin(), buf.begin() + bytesSent);
			CheckError(ec);
		}
	}


	inline JServer::JServer(uint16_t port) :
		acceptor(context)
	{
		using namespace internal;

		// Start acceptor
		asio::error_code ec{};
		asio::ip::tcp protocol = asio::ip::tcp::v4();
		acceptor.open(protocol, ec);
		CheckError(ec);
		acceptor.bind(asio::ip::tcp::endpoint(protocol, port), ec);
		CheckError(ec);
		acceptor.listen(acceptor.max_listen_connections, ec);
		CheckError(ec);

		this->port = (uint16_t)acceptor.local_endpoint().port();

		// Accept connections from a separate thread
		acceptorThrd = std::thread([this] {
			while (!disconnectPending) {
				asio::error_code ec{};
				asio::ip::tcp::socket socket = acceptor.accept(ec); // Ignore error

				if (socket.is_open()) {
					std::scoped_lock lock(mtx);
					if (clients.size() < maxClients) {
						clients.insert({ nextID, Client(std::move(socket)) });
						newConnections.push_back(nextID);
						nextID++;
					}
				}
			}
			});

		// Read and write socket from a separate thread
		msgThrd = std::thread([this] {
			asio::error_code ec{};

			while (!disconnectPending) {
				std::scoped_lock lock(mtx);

				std::vector<ExternalJClient> dcs;

				for (auto& [id, client] : clients) {
					try {
						ReadSocket(client.socket, client.dataRecv);
						WriteSocket(client.socket, client.dataSend);
					} catch (Exception&) {
						dcs.push_back(id);
						disconnections.push_back(id);
					}
				}

				// Remove disconnected clients here so iterators stay valid
				for (ExternalJClient c : dcs)
					clients.erase(c);
			}
			});
	}

	inline JServer::~JServer() {
		disconnectPending = true;
		if (acceptorThrd.joinable())
			acceptorThrd.join();
		if (msgThrd.joinable())
			msgThrd.join();
	}

	inline void JServer::Update() {
		using namespace internal;
		std::scoped_lock lock(mtx);

		// Disconnections
		for (ExternalJClient id : disconnections)
			OnClientDisconnect(id);
		disconnections.clear();

		// New connections
		for (auto& id : newConnections) {
			if (!OnClientConnect(id)) {
				asio::error_code ec{};
				clients.at(id).socket.close(ec); // Ignore error
				clients.erase(id);
			}
		}
		newConnections.clear();

		// Process messages
		for (auto& [id, client] : clients) {
			ProcessDataReceived(
				client.dataRecv,
				[this, id](Json& msg) { OnReceive(id, msg); }
			);
		}
	}

	inline void JServer::Send(ExternalJClient client, const Json& j) {
		std::scoped_lock lock(mtx);
		auto c = clients.find(client);
		if (c != clients.end()) {
			auto& sendBuf = c->second.dataSend;
			Bytes b = internal::JsonToBytes(j);
			sendBuf.insert(sendBuf.end(), b.begin(), b.end());
		}
	}

	inline void JServer::SendAll(const Json& j) {
		std::scoped_lock lock(mtx);
		for (auto& [id, _] : clients) {
			auto c = clients.find(id);
			if (c != clients.end()) {
				auto& sendBuf = c->second.dataSend;
				Bytes b = internal::JsonToBytes(j);
				sendBuf.insert(sendBuf.end(), b.begin(), b.end());
			}
		}
	}

	inline void JServer::DisconnectClient(ExternalJClient client) {
		std::scoped_lock lock(mtx);
		if (clients.contains(client))
			clients.at(client).socket.close();
	}

	inline uint16_t JServer::GetPort() const {
		return port;
	}



	inline LocalJClient::LocalJClient(const std::string& host, uint16_t port) :
		socket(context)
	{
		using namespace internal;
		asio::error_code ec{};

		// Get endpoint from hostname
		asio::ip::tcp::resolver resolver(context);
		auto results = resolver.resolve(host, std::to_string(port));

		// Connect to endpoint
		asio::connect(socket, results.begin(), results.end(), ec);
		CheckError(ec);

		connected = true;

		// Read and write socket from a separate thread
		thrd = std::thread([this] {

			try {
				while (!disconnectPending) {
					std::scoped_lock lock(mtx);
					ReadSocket(socket, dataRecv);
					WriteSocket(socket, dataSend);
				}
			} catch (Exception&) {}

			disconnectPending = true;

			});
	}

	inline LocalJClient::~LocalJClient() {
		Disconnect(); // Set flag to disconnect
		Update(); // Disconnect here
	}

	inline void LocalJClient::Update() {
		using namespace internal;

		if (!connected)
			return;

		// Check if disconnected
		if (disconnectPending) {
			if (thrd.joinable())
				thrd.join();
			disconnectPending = false;
			asio::error_code ec{};
			socket.close(ec); // Ignore error
			connected = false;
			OnDisconnect();
			return;
		}

		// Process messages
		ProcessDataReceived(
			dataRecv,
			[this](Json& msg) { OnReceive(msg); }
		);
	}

	inline void LocalJClient::Disconnect() {
		if (connected)
			disconnectPending = true;
	}

	inline void LocalJClient::Send(const Json& j) {
		if (connected) {
			std::scoped_lock lock(mtx);
			Bytes b = internal::JsonToBytes(j);
			dataSend.insert(dataSend.end(), b.begin(), b.end());
		}
	}
}
