// Host test for getNormalizedHeading(). Build and run: make -C extras/test
#include <cstdio>
#include <cstring>
#include <Arduino.h>
#include "goBILDA_Pinpoint.h"

TwoWire Wire;

static float normalized(goBILDA::Pinpoint& pp, float radians)
{
    uint8_t b[4];
    std::memcpy(b, &radians, 4);
    Wire.rx.assign(b, b + 4);
    return pp.getNormalizedHeading();
}

int main()
{
    goBILDA::Pinpoint pp;
    const float cases[][2] = {  // heading in degrees, expected normalized degrees
        {0, 0}, {10, 10}, {-10, -10}, {170, 170}, {190, -170}, {-190, 170}, {370, 10}, {-370, -10}};
    int fails = 0;
    for (auto& c : cases) {
        float got = normalized(pp, c[0] / RAD_TO_DEG);
        if (std::fabs(got - c[1]) > 1e-3f) {
            std::printf("FAIL heading %g deg: got %g, expected %g\n", c[0], got, c[1]);
            fails++;
        }
    }
    std::printf(fails ? "FAILED (%d)\n" : "PASSED\n", fails);
    return fails ? 1 : 0;
}
