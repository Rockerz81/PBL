// main.cpp - wires the layers together:
//   Persistence/OverpassClient -> RoadNetwork -> DeliveryManager(AStarRouter) -> MapRenderer
#include <raylib.h>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <utility>

#include "DeliveryManager.h"
#include "Exceptions.h"
#include "MapRenderer.h"
#include "OverpassClient.h"
#include "Persistence.h"
#include "RoadNetwork.h"

using namespace logi;

namespace {

struct Options {
    bool offline{false};  // never touch the network
    bool refresh{false};  // ignore the cache and re-download
    std::string configPath{"config/config.json"};
};

Options parseArgs(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--offline") o.offline = true;
        else if (a == "--refresh") o.refresh = true;
        else if (a == "--config" && i + 1 < argc) o.configPath = argv[++i];
        else std::cerr << "Unknown argument ignored: " << a << '\n';
    }
    return o;
}

// Synthetic street grid so the demo still runs with no internet and no cache.
RoadNetwork buildDemoGrid(const logi::BoundingBox& box) {
    constexpr int kN = 22;
    RoadNetwork net;
    for (int r = 0; r < kN; ++r)
        for (int c = 0; c < kN; ++c)
            net.addNode({box.south + (box.north - box.south) * r / (kN - 1),
                         box.west + (box.east - box.west) * c / (kN - 1)});
    auto id = [](int r, int c) { return static_cast<RoadNetwork::NodeId>(r * kN + c); };
    for (int r = 0; r < kN; ++r)
        for (int c = 0; c < kN; ++c) {
            const bool blocked = ((r * 31 + c * 17) % 7 == 0);  // deterministic "missing" streets
            if (c + 1 < kN && !blocked) net.addRoad(id(r, c), id(r, c + 1));
            if (r + 1 < kN && !blocked) net.addRoad(id(r, c), id(r + 1, c));
        }
    return net.largestComponent();
}

// Cache -> (Overpass API -> save cache) -> demo grid. Every failure is reported
// in `status` and degrades gracefully instead of crashing.
RoadNetwork obtainNetwork(const AppConfig& cfg, const Options& opt, std::string& status) {
    namespace fs = std::filesystem;
    if (!opt.refresh && fs::exists(cfg.networkCachePath)) {
        try {
            RoadNetwork net = Persistence::loadNetwork(cfg.networkCachePath);
            status = "Loaded road network from cache.";
            return net;
        } catch (const LogisticsError& e) {
            std::cerr << "Cache unusable (" << e.what() << "), will re-download.\n";
        }
    }
    if (!opt.offline) {
        try {
            std::cout << "Downloading road network from Overpass API...\n";
            RoadNetwork net = OverpassClient(cfg.overpassUrl).fetchNetwork(cfg.bbox);
            try {
                Persistence::saveNetwork(net, cfg.networkCachePath);
            } catch (const LogisticsError& e) {
                std::cerr << "Warning: could not save cache: " << e.what() << '\n';
            }
            status = "Downloaded live OpenStreetMap data.";
            return net;
        } catch (const LogisticsError& e) {
            std::cerr << "Download failed: " << e.what() << '\n';
        }
    }
    status = "OFFLINE DEMO: synthetic grid (no cache / API unavailable).";
    return buildDemoGrid(cfg.bbox);
}

AppConfig loadConfigOrDefault(const std::string& path) {
    try {
        return Persistence::loadConfig(path);
    } catch (const LogisticsError& e) {
        std::cerr << "Config not loaded (" << e.what() << "); using defaults.\n";
        return AppConfig{};
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options opt = parseArgs(argc, argv);
        AppConfig cfg = loadConfigOrDefault(opt.configPath);
        std::string status;

        const RoadNetwork network = obtainNetwork(cfg, opt, status);  // const: simulation never edits it
        std::cout << "Network: " << network.nodeCount() << " nodes, " << network.edgeCount() << " roads\n";

        // Snap GPS points from the config to the nearest real road node.
        const RoadNetwork::NodeId depot = network.nearestNode(cfg.depot);
        DeliveryManager manager(network, depot, cfg.truckSpeedKmh, cfg.timeScale);
        for (const auto& c : cfg.customers) manager.addCustomer(network.nearestNode(c));
        try {
            manager.reorderCustomersNearestFirst();
            manager.planRoute();
        } catch (const RoutingError& e) {
            status = e.what();
        }

        SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
        InitWindow(1280, 800, "Logistics Delivery Router - A* + OpenStreetMap");
        SetTargetFPS(60);

        MapRenderer renderer(network);
        renderer.fitToWindow(GetScreenWidth(), GetScreenHeight());
        try {
            auto namedPlaces = OverpassClient(cfg.overpassUrl).fetchNamedPlaces(cfg.bbox);
            std::cout << "Loaded " << namedPlaces.size() << " named map places.\n";
            renderer.setNamedPlaces(std::move(namedPlaces));
            status = "Loaded named map labels. Press N to enter a destination name.";
        } catch (const LogisticsError& e) {
            status = std::string("Place labels unavailable: ") + e.what();
            std::cerr << status << '\n';
        }

        while (!WindowShouldClose()) {
            // ---------- input ----------
            try {
                if (IsWindowResized()) renderer.fitToWindow(GetScreenWidth(), GetScreenHeight());
                const float wheel = GetMouseWheelMove();
                if (wheel != 0.0f) renderer.zoomAt(GetMousePosition(), wheel);
                if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) renderer.pan(GetMouseDelta());

                if (IsKeyPressed(KEY_N)) {
                    std::string placeName;
                    std::cout << "\nEnter delivery location name (e.g. Clock Tower): ";
                    std::getline(std::cin >> std::ws, placeName);
                    if (!placeName.empty()) {
                        const GeoPoint place = OverpassClient(cfg.overpassUrl).geocodePlace(placeName, cfg.bbox);
                        const auto destinationNode = network.nearestNode(place);
                        manager.addCustomer(destinationNode);
                        manager.planRoute();
                        // Keep the entered name visible next to the snapped road node.
                        renderer.addNamedPlace(placeName, network.position(destinationNode));
                        status = "Destination added: " + placeName;
                    }
                }
                if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                    manager.addCustomer(network.nearestNode(renderer.toGeo(GetMousePosition())));
                    manager.planRoute();
                    status = "Customer added, route re-planned.";
                }
                if (IsKeyPressed(KEY_SPACE)) {
                    if (manager.state() == AgentState::Idle || manager.state() == AgentState::Finished) manager.start();
                    else manager.togglePause();
                }
                if (IsKeyPressed(KEY_R)) manager.reset();
                if (IsKeyPressed(KEY_O)) {
                    manager.reorderCustomersNearestFirst();
                    manager.planRoute();
                    status = "Customers re-ordered nearest-first.";
                }
                if (IsKeyPressed(KEY_C)) {
                    manager.clearCustomers();
                    status = "Customers cleared. Click the map to add some.";
                }
                if (IsKeyPressed(KEY_UP)) manager.setTimeScale(manager.timeScale() * 1.5);
                if (IsKeyPressed(KEY_DOWN)) manager.setTimeScale(std::max(1.0, manager.timeScale() / 1.5));
                if (IsKeyPressed(KEY_S)) {
                    cfg.depot = network.position(manager.depot());
                    cfg.customers.clear();
                    for (auto n : manager.customers()) cfg.customers.push_back(network.position(n));
                    cfg.timeScale = manager.timeScale();
                    Persistence::saveConfig(cfg, opt.configPath);
                    status = "Configuration saved to " + opt.configPath;
                }
            } catch (const LogisticsError& e) {  // e.g. RoutingError, FileError: show, don't crash
                status = e.what();
            } catch (const std::exception& e) {
                status = std::string("Error: ") + e.what();
            }

            // ---------- simulate + draw ----------
            manager.update(GetFrameTime());
            BeginDrawing();
            renderer.draw(manager);
            renderer.drawHud(manager, status);
            EndDrawing();
        }
        CloseWindow();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << '\n';
        return 1;
    }
}
