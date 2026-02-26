#ifndef APP_LOOKUP_TABLE_H
#define APP_LOOKUP_TABLE_H

struct Point {
    float x;
    float y;
};

class LookupTable {
private:
    const Point* curve_data;
    int curve_size;
    float x_increment;

public:
    // Constructor
    LookupTable(const Point* curve, int size);

    // Generic interpolation function (O(1) lookup)
    float interpolate(float x_val);
};

#endif // APP_LOOKUP_TABLE_H
