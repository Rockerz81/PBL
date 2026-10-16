#pragma once
#include <string>

enum class OrderStatus { PENDING, DELIVERED };

// One delivery request: which customer location it must reach.
class DeliveryOrder {
private:
    int orderId;
    int destination;       // location index in RoadNetwork
    OrderStatus status;

public:
    DeliveryOrder(int id, int dest)
        : orderId(id), destination(dest), status(OrderStatus::PENDING) {}

    int getOrderId() const { return orderId; }
    int getDestination() const { return destination; }
    OrderStatus getStatus() const { return status; }
    std::string getStatusText() const {
        return status == OrderStatus::DELIVERED ? "Delivered" : "Pending";
    }

    void markDelivered() { status = OrderStatus::DELIVERED; }
};
