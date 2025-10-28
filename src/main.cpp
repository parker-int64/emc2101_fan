// #include "emc2101.h"
#include "emc2101_server.h"
// #include "st7789.h"
// #include "display.h"



int main(int argc, char** argv) {

    EMC2101Server server("localhost", 8080);
    server.start();

    return 0;
}