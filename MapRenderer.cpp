#include "MapRenderer.h"

#include <algorithm>
#include <cmath>

#include "Geo.h"

namespace logi {

namespace {
const Color kBackground{24, 28, 36, 255};
const Color kRoad{62, 70, 88, 255};          // dimmer than before so labels/route stand out
const Color kNode{110, 120, 138, 255};
const Color kRoute{255, 196, 61, 255};
const Color kPoiDot{110, 210, 220, 255};
const Color kPoiText{190, 210, 222, 255};

// Lower number = drawn first = survives the collision pass.
// Roads/junction names help navigation, so they beat shops and clinics.
int labelPriority(const std::string& name) {
    for (const char* key : {"Road", "Marg", "Chowk", "Lane"}) {
        if (name.find(key) != std::string::npos) return 0;
    }
    return 1;
}

bool overlapsAny(const Rectangle& r, const std::vector<Rectangle>& taken) {
    for (const auto& t : taken) {
        if (CheckCollisionRecs(r, t)) return true;
    }
    return false;
}
}  // namespace

MapRenderer::MapRenderer(const RoadNetwork& network) : network_(network) {
    const BoundingBox b = network_.bounds();  // throws if the network is empty
    origin_ = {(b.south + b.north) / 2.0, (b.west + b.east) / 2.0};
    cosOriginLat_ = std::cos(toRadians(origin_.lat));
}

void MapRenderer::setNamedPlaces(std::vector<NamedPlace> places) {
    places_ = std::move(places);
    std::stable_sort(places_.begin(), places_.end(), [](const NamedPlace& a, const NamedPlace& b) {
        return labelPriority(a.name) < labelPriority(b.name);
    });
}

// Equirectangular projection around origin_: accurate to well under 1% for
// city-sized areas, and trivially invertible for mouse picking.
//   x = R * dLon * cos(lat0)      y = -R * dLat   (screen y grows downward)
Vector2 MapRenderer::toMeters(const GeoPoint& p) const {
    const double x = kEarthRadiusMeters * toRadians(p.lon - origin_.lon) * cosOriginLat_;
    const double y = -kEarthRadiusMeters * toRadians(p.lat - origin_.lat);
    return {static_cast<float>(x), static_cast<float>(y)};
}

Vector2 MapRenderer::toScreen(const GeoPoint& p) const {
    const Vector2 m = toMeters(p);
    return {m.x * scale_ + offset_.x, m.y * scale_ + offset_.y};
}

GeoPoint MapRenderer::toGeo(Vector2 s) const {
    const double mx = (s.x - offset_.x) / scale_;
    const double my = (s.y - offset_.y) / scale_;
    return {origin_.lat + toDegrees(-my / kEarthRadiusMeters),
            origin_.lon + toDegrees(mx / (kEarthRadiusMeters * cosOriginLat_))};
}

void MapRenderer::fitToWindow(int width, int height) {
    const BoundingBox b = network_.bounds();
    const Vector2 a = toMeters({b.south, b.west});
    const Vector2 c = toMeters({b.north, b.east});
    const float spanX = std::max(1.0f, std::fabs(c.x - a.x));
    const float spanY = std::max(1.0f, std::fabs(c.y - a.y));
    const float margin = 40.0f;
    scale_ = std::min((width - 2 * margin) / spanX, (height - 2 * margin) / spanY);
    fitScale_ = scale_;  // remember the "zoom = 1.0" reference
    offset_ = {width / 2.0f, height / 2.0f};  // origin_ is the centre of the box
    cacheDirty_ = true;
}



void MapRenderer::zoomAt(Vector2 p, float wheel) {
    const float factor = std::pow(1.15f, wheel);
    scale_ *= factor;
    // Keep the point under the cursor fixed while scaling.
    offset_ = {p.x - (p.x - offset_.x) * factor, p.y - (p.y - offset_.y) * factor};
    cacheDirty_ = true;
}

void MapRenderer::pan(Vector2 d) {
    offset_ = {offset_.x + d.x, offset_.y + d.y};
    cacheDirty_ = true;
}

void MapRenderer::refreshScreenCache() {
    if (!cacheDirty_) return;
    screen_.resize(network_.nodeCount());
    for (RoadNetwork::NodeId i = 0; i < network_.nodeCount(); ++i) screen_[i] = toScreen(network_.position(i));
    cacheDirty_ = false;
}

void MapRenderer::draw(const DeliveryManager& dm) {
    refreshScreenCache();
    ClearBackground(kBackground);

    const float zoom = scale_ / fitScale_;  // 1.0 = whole map fits the window
    const int sw = GetScreenWidth();
    const int sh = GetScreenHeight();

    // 1. Roads = lines. Each undirected road is stored twice, so draw only u < v.
    for (RoadNetwork::NodeId u = 0; u < network_.nodeCount(); ++u) {
        for (const auto& e : network_.neighbors(u)) {
            if (u < e.first) DrawLineV(screen_[u], screen_[e.first], kRoad);
        }
    }

    // 2. Junction dots: only when zoomed in, otherwise they just add noise.
    if (zoom > 3.0f) {
        for (RoadNetwork::NodeId u = 0; u < network_.nodeCount(); ++u) {
            if (network_.neighbors(u).size() != 2) DrawCircleV(screen_[u], 1.5f, kNode);
        }
    }

    // 3. Planned A* route, with a dark outline so it stays readable over roads.
    const auto& route = dm.routeNodes();
    for (std::size_t i = 1; i < route.size(); ++i) {
        DrawLineEx(screen_[route[i - 1]], screen_[route[i]], 6.0f, Fade(BLACK, 0.55f));
    }
    for (std::size_t i = 1; i < route.size(); ++i) {
        DrawLineEx(screen_[route[i - 1]], screen_[route[i]], 3.0f, kRoute);
    }

    // 4. Label collision bookkeeping. Anything important is reserved FIRST so
    //    low-priority labels can never cover it.
    std::vector<Rectangle> taken;
    taken.push_back({8.0f, 8.0f, 450.0f, showHelp_ ? 150.0f : 120.0f});  // HUD panel

    const Vector2 depot = screen_[dm.depot()];
    taken.push_back({depot.x - 10, depot.y - 10, 70.0f, 20.0f});
    const auto& customers = dm.customers();
    for (const auto customer : customers) {
        const Vector2 p = screen_[customer];
        taken.push_back({p.x - 10, p.y - 10, 32.0f, 20.0f});
    }

    // 4a. Pinned names (user typed them with N): always drawn, bright.
    for (const auto& place : pinned_) {
        const Vector2 p = toScreen(place.position);
        const int w = MeasureText(place.name.c_str(), 14);
        const Rectangle r{p.x + 12.0f, p.y - 22.0f, static_cast<float>(w + 6), 18.0f};
        DrawRectangleRec(r, Fade(kBackground, 0.85f));
        DrawText(place.name.c_str(), static_cast<int>(r.x) + 3, static_cast<int>(r.y) + 2, 14, RAYWHITE);
        taken.push_back(r);
    }

    // 4b. Everything else: only when zoomed in, only on-screen, only if there's room.
    if (zoom > 1.8f) {
        const int fontSize = zoom > 4.0f ? 12 : 11;
        for (const auto& place : places_) {
            const Vector2 p = toScreen(place.position);
            if (p.x < 0 || p.y < 0 || p.x > sw || p.y > sh) continue;  // off-screen

            const int w = MeasureText(place.name.c_str(), fontSize);
            const Rectangle r{p.x + 5.0f, p.y - 13.0f, static_cast<float>(w + 4), static_cast<float>(fontSize + 2)};
            if (overlapsAny(r, taken)) continue;  // no room: skip instead of piling up
            taken.push_back(r);

            DrawCircleV(p, 2.0f, Fade(kPoiDot, 0.8f));
            DrawRectangleRec(r, Fade(kBackground, 0.65f));
            DrawText(place.name.c_str(), static_cast<int>(r.x) + 2, static_cast<int>(r.y) + 1, fontSize,
                     Fade(kPoiText, 0.85f));
        }
    }

    // 5. Depot (blue square) and customers (red = pending, green = delivered, numbered in order).
    DrawRectangleV({depot.x - 8, depot.y - 8}, {16, 16}, Color{64, 140, 255, 255});
    DrawText("DEPOT", static_cast<int>(depot.x) + 12, static_cast<int>(depot.y) - 8, 14, RAYWHITE);
    for (std::size_t i = 0; i < customers.size(); ++i) {
        const Vector2 p = screen_[customers[i]];
        const bool done = i < dm.deliveredCount();
        DrawCircleV(p, 8.0f, done ? Color{80, 200, 120, 255} : Color{235, 70, 70, 255});
        DrawCircleLines(static_cast<int>(p.x), static_cast<int>(p.y), 8.0f, RAYWHITE);
        DrawText(TextFormat("%d", static_cast<int>(i + 1)), static_cast<int>(p.x) + 11,
                 static_cast<int>(p.y) - 8, 14, RAYWHITE);
    }

    // 6. Truck.
    drawTruck(dm.truckPose());
}

// The truck is built from primitives: body + cab rectangles and 4 wheels, all
// rotated to the heading of the road segment it is currently driving along.
void MapRenderer::drawTruck(const DeliveryManager::TruckPose& pose) {
    const Vector2 p = toScreen(pose.position);
    const Vector2 a = toScreen(pose.segmentFrom);
    const Vector2 b = toScreen(pose.segmentTo);
    if (std::hypot(b.x - a.x, b.y - a.y) > 1e-3f) lastHeadingRad_ = std::atan2(b.y - a.y, b.x - a.x);

    const float h = lastHeadingRad_;
    const Vector2 dir{std::cos(h), std::sin(h)};
    const Vector2 perp{-dir.y, dir.x};
    const float deg = h * 180.0f / static_cast<float>(kPi);

    DrawRectanglePro({p.x, p.y, 22.0f, 11.0f}, {11.0f, 5.5f}, deg, Color{245, 245, 245, 255});  // cargo box
    DrawRectanglePro({p.x + dir.x * 14.5f, p.y + dir.y * 14.5f, 8.0f, 9.0f}, {4.0f, 4.5f}, deg,
                     Color{230, 90, 40, 255});  // cab
    for (float along : {-6.0f, 6.0f}) {
        for (float side : {-6.0f, 6.0f}) {
            DrawCircleV({p.x + dir.x * along + perp.x * side, p.y + dir.y * along + perp.y * side}, 2.2f, BLACK);
        }
    }
}

void MapRenderer::drawHud(const DeliveryManager& dm, const std::string& status) const {
    DrawRectangle(8, 8, 450, showHelp_ ? 150 : 120, Fade(BLACK, 0.65f));
    DrawText("Logistics Router  (A* + OpenStreetMap)", 18, 14, 18, RAYWHITE);
    DrawText(TextFormat("State: %s%s", toString(dm.state()), dm.paused() ? " [paused]" : ""), 18, 40, 16, RAYWHITE);
    DrawText(TextFormat("Delivered: %d / %d   Route: %.2f km", static_cast<int>(dm.deliveredCount()),
                        static_cast<int>(dm.customers().size()), dm.totalPlannedMeters() / 1000.0),
             18, 60, 16, RAYWHITE);
    DrawText(TextFormat("Graph: %d nodes, %d roads   Speed x%.0f   Labels: %d",
                        static_cast<int>(network_.nodeCount()), static_cast<int>(network_.edgeCount()),
                        dm.timeScale(), static_cast<int>(places_.size())),
             18, 80, 16, RAYWHITE);
    if (showHelp_) {
        DrawText("SPACE start/pause  R reset  O nearest-first  C clear  S save  N named destination", 18, 104, 13, LIGHTGRAY);
        DrawText("Left-click: add  Right-drag: pan  Wheel: zoom (labels appear)  Up/Down: speed  H: hide", 18, 122, 13, LIGHTGRAY);
    } else {
        DrawText("H: show controls", 18, 102, 13, GRAY);
    }
    if (!status.empty()) DrawText(status.c_str(), 18, GetScreenHeight() - 28, 16, Color{255, 210, 120, 255});
    DrawFPS(GetScreenWidth() - 90, 10);
}

}  // namespace logi
