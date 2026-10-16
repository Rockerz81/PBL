#pragma once
#include <string>

// The delivery vehicle. It remembers where it is and what it has done.
class Vehicle {
private:
    std::string name;
    int currentLocation;
    int deliveriesCompleted;

public:
    Vehicle(const std::string& vehicleName, int startLocation)
        : name(vehicleName), currentLocation(startLocation), deliveriesCompleted(0) {}

    const std::string& getName() const { return name; }
    int getCurrentLocation() const { return currentLocation; }
    int getDeliveriesCompleted() const { return deliveriesCompleted; }

    void moveTo(int newLocation) { currentLocation = newLocation; }
    void recordDelivery() { ++deliveriesCompleted; }
};
