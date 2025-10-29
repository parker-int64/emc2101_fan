#include "crow.h"
#include <string>
#include <thread>


class EMC2101Server {

public:
    explicit EMC2101Server(std::string page = "index.html",
                           std::string address = "0.0.0.0", 
                           int port = 8080);
    ~EMC2101Server();

    void start();
    void stop();

    crow::SimpleApp& app() { return app_; }

private:
    std::string address_;
    std::string page_;
    int port_;
    crow::SimpleApp app_;


};