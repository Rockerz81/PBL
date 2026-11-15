// MapRenderer.h - Raylib view of the simulation. READ-ONLY with respect to the
// model: it never changes the RoadNetwork or DeliveryManager, it only draws them.
#pragma once
#include <raylib.h>

#include <string>
#include <utility>
#include <vector>

#include "DeliveryManager.h"
#include "OverpassClient.h"
#include "RoadNetwork.h"

namespace logi {

class MapRenderer {
public:
    explicit MapRenderer(const RoadNetwork& network);

    void fitToWindow(int width, int height);       // reset zoom/pan so the whole map is visible
    void zoomAt(Vector2 screenPoint, float wheel); // zoom around the cursor
    void pan(Vector2 deltaPixels);
    void setNamedPlaces(std::vector<NamedPlace> places) { places_ = std::move(places); }
    void addNamedPlace(const std::string& name, const GeoPoint& position) { places_.push_back({name, position}); }

    Vector2 toScreen(const GeoPoint& p) const;     // GPS -> pixels
    GeoPoint toGeo(Vector2 screen) const;          // pixels -> GPS (for mouse clicks)

    void draw(const DeliveryManager& manager);     // roads, route, markers, truck
    void drawHud(const DeliveryManager& manager, const std::string& status) const;

private:
    Vector2 toMeters(const GeoPoint& p) const;     // local equirectangular projection
    void refreshScreenCache();
    void drawTruck(const DeliveryManager::TruckPose& pose);

    const RoadNetwork& network_;
    GeoPoint origin_{};          // lat/lon that maps to the metre-space origin
    double cosOriginLat_{1.0};   // shrinks longitude so 1 deg lon ~ real ground distance
    float scale_{1.0f};          // pixels per metre
    Vector2 offset_{0.f, 0.f};   // pixel position of the metre-space origin
    std::vector<Vector2> screen_;  // cached pixel position of each node
    std::vector<NamedPlace> places_;
    bool cacheDirty_{true};
    float lastHeadingRad_{0.f};  // keeps truck orientation when it is stationary
};

}  // namespace logi
