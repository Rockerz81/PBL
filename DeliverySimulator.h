#pragma once
#include <iostream>
#include <memory>
#include <vector>
#include "RoadNetwork.h"
#include "DeliveryOrder.h"
#include "Vehicle.h"
#include "RoutePlanner.h"

// Controller class: owns every object and runs the simulation.
// It is built from other objects (composition / "has-a" relationships).
class DeliverySimulator {
private:
    RoadNetwork network;
    Vehicle vehicle;
    std::vector<DeliveryOrder> orders;
    std::unique_ptr<RoutePlanner> planner;   // can hold ANY RoutePlanner (polymorphism)
    int warehouse;

    void buildCityMap() {
        const char* names[] = {
            "Clement Town", "Clock Tower", "Prem Nagar", "Rajpur Road", "Patel Nagar",
            "Ballupur Chowk", "ISBT Dehradun", "Raipur", "Sahastradhara", "Vasant Vihar",
            "Dalanwala", "Hathibarkala", "Rajpur", "Kaulagarh", "GMS Road",
            "Niranjanpur", "Turner Road", "Shimla Bypass", "Chakrata Road", "Jakhan"
        };
        for (const char* n : names) network.addLocation(n);

        // Cross-town bridges
        network.addRoad(1, 4, 4);   network.addRoad(6, 7, 6);
        // Regular roads
        network.addRoad(0, 6, 5);   network.addRoad(0, 16, 3);  network.addRoad(0, 4, 6);
        network.addRoad(1, 3, 2);   network.addRoad(1, 10, 3);  network.addRoad(1, 11, 4);
        network.addRoad(2, 18, 4);  network.addRoad(2, 5, 6);   network.addRoad(2, 17, 7);
        network.addRoad(3, 12, 6);  network.addRoad(3, 19, 4);
        network.addRoad(4, 15, 2);  network.addRoad(4, 14, 3);
        network.addRoad(5, 13, 3);  network.addRoad(5, 9, 3);
        network.addRoad(6, 14, 4);  network.addRoad(6, 17, 5);  network.addRoad(6, 16, 4);
        network.addRoad(7, 8, 7);   network.addRoad(7, 10, 5);
        network.addRoad(8, 19, 5);  network.addRoad(9, 13, 2);
        network.addRoad(10, 11, 2); network.addRoad(11, 12, 5);
        network.addRoad(12, 19, 3); network.addRoad(15, 18, 5);
    }

public:
    // Constructor builds the map; the vehicle starts at the warehouse (index 0)
    DeliverySimulator(std::unique_ptr<RoutePlanner> routePlanner)
        : vehicle("Van-1", 0), planner(std::move(routePlanner)), warehouse(0) {
        buildCityMap();
    }

    void addOrder(int destination) {
        orders.emplace_back((int)orders.size() + 1, destination);
    }

    void run() {
        std::cout << "Starting Delivery Route from Warehouse: "
                  << network.getName(warehouse) << "\n";
        std::cout << "Vehicle: " << vehicle.getName()
                  << " | Algorithm: " << planner->getAlgorithmName() << "\n";
        std::cout << "==================================================\n\n";

        for (auto& order : orders) {
            int from = vehicle.getCurrentLocation();
            int to = order.getDestination();

            std::cout << "Leg " << order.getOrderId() << ": Traveling from "
                      << network.getName(from) << " to " << network.getName(to) << "\n";

            planner->planLeg(from, to, network);

            vehicle.moveTo(to);
            vehicle.recordDelivery();
            order.markDelivered();
        }
        std::cout << "All deliveries completed successfully! ("
                  << vehicle.getDeliveriesCompleted() << " orders)\n";
    }
};
