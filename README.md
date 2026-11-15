# Logistics Delivery Router (C++17)

OOP delivery-routing simulator: real OpenStreetMap roads (Overpass API) -> sparse graph -> A* routing -> animated truck in Raylib.

## Project structure
```
logistics-router/
├── CMakeLists.txt
├── config/config.json          # bbox, depot, customers, speed (read/written via <fstream>)
├── data/                       # network_cache.txt is created here on first download
├── include/
│   ├── Types.h                 # GeoPoint, BoundingBox
│   ├── Geo.h                   # Haversine distance
│   ├── Exceptions.h            # LogisticsError, NetworkError, ParseError, FileError, RoutingError
│   ├── RoadNetwork.h           # sparse adjacency-list graph (encapsulated)
│   ├── AStarRouter.h           # A* with std::priority_queue (header-only)
│   ├── DeliveryManager.h       # truck state machine + simulation
│   ├── OverpassClient.h        # libcurl + nlohmann/json
│   ├── Persistence.h           # config (JSON) + network cache (text)
│   └── MapRenderer.h           # Raylib drawing
├── src/  RoadNetwork.cpp DeliveryManager.cpp OverpassClient.cpp Persistence.cpp MapRenderer.cpp main.cpp
└── tests/core_test.cpp         # A* vs Dijkstra, exceptions, full simulation (no external libs)
```
Layering (each layer only depends on the ones above it in this list):
`logi_core` (no external libs) -> `logi_io` (curl, json) -> `logistics_router` (Raylib).

## Build

### Linux (Debian/Ubuntu)
```bash
sudo apt install build-essential cmake libcurl4-openssl-dev nlohmann-json3-dev \
     libasound2-dev libx11-dev libxrandr-dev libxi-dev libgl1-mesa-dev libglu1-mesa-dev libxcursor-dev libxinerama-dev
# Raylib: either `sudo apt install libraylib-dev` (if your distro has >= 4.0)
# or do nothing - CMake downloads and builds Raylib 5.0 automatically.
cmake -S . -B build && cmake --build build -j
./build/logistics_router
```
### macOS
```bash
brew install cmake curl nlohmann-json raylib
cmake -S . -B build && cmake --build build -j && ./build/logistics_router
```
### Windows (vcpkg, Visual Studio)
```powershell
vcpkg install curl nlohmann-json raylib
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release
```
### Without CMake (Linux, everything installed system-wide)
```bash
g++ -std=c++17 -O2 -Iinclude src/*.cpp -o logistics_router -lcurl -lraylib -lGL -lm -lpthread -ldl -lrt -lX11
```
(Link order matters: libraries come AFTER the sources that use them.)

### Tests (needs only a compiler)
```bash
g++ -std=c++17 -Iinclude tests/core_test.cpp src/RoadNetwork.cpp src/DeliveryManager.cpp -o core_test && ./core_test
```

## Run
```
./logistics_router              # cache -> else Overpass API -> else synthetic grid
./logistics_router --refresh    # force re-download
./logistics_router --offline    # never use the network
./logistics_router --config my.json
```
Controls: SPACE start/pause · R reset · O nearest-first order · C clear · S save config ·
left-click add customer · right-drag pan · wheel zoom · Up/Down simulation speed.

To change the region, edit `bbox` in `config/config.json` (south, west, north, east) and run with `--refresh`.
Keep the box small (a few km); a whole city returns tens of MB and Overpass may rate-limit you (HTTP 429).

## Design notes for evaluation defense
* **Encapsulation / const-correctness** - `RoadNetwork` data members are private; all reads are `const` methods returning `const&`. `main` builds the network as `const RoadNetwork`, so the simulation provably cannot mutate it. `DeliveryManager` and `AStarRouter` hold `const RoadNetwork&`.
* **Why an adjacency list** - road graphs are sparse (average degree ~2-3). A list costs O(V+E) memory and iterating neighbours is O(degree); a V×V matrix would need ~10^9 cells for 30k nodes.
* **A\*** - `f = g + h`, h = Haversine distance to the goal. Admissible (a straight line is never longer than a road) and consistent, so (1) the first pop of the goal is optimal -> **early stop**, and (2) each node is expanded once. `RoadNetwork::addRoad` clamps weights to >= straight-line distance so this guarantee cannot be broken by bad data. `std::priority_queue` is a max-heap, so the comparator `a.f > b.f` makes it a min-heap. Instead of decrease-key we push duplicates and skip stale entries (lazy deletion): O((V+E) log V).
* **Verified** - `tests/core_test.cpp` checks A* distance equals Dijkstra on 200 random pairs.
* **Exception safety** - typed exceptions (`NetworkError`, `ParseError`, `FileError`, `RoutingError`). `planRoute()` has the *strong guarantee* (builds into locals, commits only if every leg succeeded). curl handles and buffers are RAII (`unique_ptr` with custom deleters), so nothing leaks when an exception is thrown. The app degrades gracefully: cache -> API -> synthetic grid.
* **State machine** - `Idle -> EnRoute -> Servicing -> EnRoute ... -> Finished`. Truck position = distance travelled along a leg, mapped to a polyline segment with `std::upper_bound` on cumulative distances (O(log n)) and linearly interpolated.
* **Separation of concerns** - the simulation has no graphics dependency; `MapRenderer` only reads state.

## Known limitations (honest answers if asked)
* Roads are treated as two-way (one-way tags are ignored); travel time ignores speed limits and traffic.
* Customer ordering is greedy nearest-neighbour, not an optimal TSP/VRP solution; there is one truck.
* `nearestNode` is an O(V) scan (fine for clicks); a k-d tree / grid index would be the next step.
* Roads are re-drawn each frame; for 100k+ edges render them once into a `RenderTexture2D`.
* The Raylib / libcurl / JSON layers were written carefully but must be compiled on your machine; only the core layer was executed in my sandbox.

## Named map labels and destination search

- On startup, the app attempts to fetch named OpenStreetMap places inside `bbox` and draw their labels.
- Press **N** while the map window is focused, then type a place name in the terminal (for example, `Clock Tower`) and press Enter. The app uses OpenStreetMap Nominatim to resolve it inside the configured map bounds, snaps it to the nearest road node, and replans the route.
- The geocoder is bounded to `config/config.json`'s `bbox`. If a destination lies outside it, expand the bbox and refresh the road cache (`./build/logistics_router.exe --refresh`).
- These online lookups require internet. Place labels can fail independently without stopping the map. Nominatim usage should be kept to occasional user searches, not bulk/rapid requests.


## Named destinations and map labels

- Named OpenStreetMap features are requested for the configured bounding box and drawn as labels on the map. The HUD shows the number of loaded labels.
- Press `N` while the map window is focused, type a place name in the terminal, and press Enter. If the place is found inside the configured map area, it is snapped to the nearest road node and its entered name is shown next to that destination marker.
- These labels and name lookups require an internet connection and available OpenStreetMap/Nominatim services. If the HUD reports that labels are unavailable, check the terminal message and try again later.
