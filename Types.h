// Types.h - plain value types shared by every module.
#pragma once

namespace logi {

// A WGS-84 GPS coordinate in decimal degrees.
struct GeoPoint {
    double lat{0.0};
    double lon{0.0};
};

// A rectangular region of the map, in decimal degrees (Overpass order: S,W,N,E).
struct BoundingBox {
    double south{0.0};
    double west{0.0};
    double north{0.0};
    double east{0.0};

    bool isValid() const {
        return south < north && west < east && south >= -90.0 && north <= 90.0 &&
               west >= -180.0 && east <= 180.0;
    }
};

}  // namespace logi
