#include <cstdlib>
#include <memory>
#include "DeliverySimulator.h"

int main() {
    system("chcp 65001 > nul");   // Windows console UTF-8 fix

    // Choose the algorithm here. Swapping it later needs no other change.
    DeliverySimulator sim(std::make_unique<DijkstraPlanner>());

    for (int dest : {1, 4, 7, 9, 12}) sim.addOrder(dest);

    sim.run();
    return 0;
}
