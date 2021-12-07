# JNet

JNet is a simple header only networking library built over asio TCP sockets. To simplify serialization and deserialization, data is sent and received as JSON objects. This library uses an internal protocol and is only intended to connect to other applications using this library.<br>
Asio https://think-async.com/Asio/Download.html<br>
JSON https://github.com/nlohmann/json.<br>
C++20 is required. 

# Creating a Server

Inherit from `jnet::JServer` and optionally override its `OnClientConnect()`, `OnClientDisconnect()`, and `OnReceive()` callback methods. Creating an instance of a JServer object will immediately start the server and destroying it will stop the server. It is therefore recommended to use it in combination with a smart pointer to control its lifetime. If the server cannot be created, a `jnet::Exception` is thrown containing the error message. Call `Update()` in an update loop to receive the callbacks. 

# Connecting to a Server

Inherit from `jnet::LocalJClient` and optionally override its `OnDisconnect()` and `OnReceive()` methods. Creating an instance of a LocalJClient object will immediately connect to the server and destroying it will close the connection. If the connection cannot be made, a `jnet::Exception` is thrown containing the error message. Call `Update()` in an update loop to receive the callbacks. 
