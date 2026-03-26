#include "controller.hpp"
#include <thread>

int main() {
    std::string configFile = "../config/control.yaml";
    ThreadJobs threadJob(configFile);

    std::thread trajThread(&ThreadJobs::subTraj, &threadJob);
    std::thread stateThread(&ThreadJobs::recvStateAndPub, &threadJob);

    trajThread.join();
    stateThread.join();

    return 0;
}