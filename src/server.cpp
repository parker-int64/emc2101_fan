#include "emc2101_server.h"


EMC2101Server::EMC2101Server(std::string page, std::string address, int port) 
    : page_(page),
      address_(address), 
      port_(port)
{
    // Define routes and handlers here
    app_.Get("/status", [](const crow::Request& req, crow::Response& res) {
        res.set_content("EMC2101 Server is running", "text/plain");
    });


    app_.Get("/get_pwm_speed", [](const crow::Request& req, crow::Response& res) {
        res.set_content("Pwm Speed: 100", "text/plain");
    });

    // app_.Get("/status", [](const crow::Request& req, crow::Response& res) {
    //     res.set_content("EMC2101 Server is running", "text/plain");
    // });
}



void EMC2101Server::start() {
    server_.listen(address_, port_);
}

void EMC2101Server::stop() {
    server_.stop();
}


EMC2101Server::~EMC2101Server()
{
    stop();
}
