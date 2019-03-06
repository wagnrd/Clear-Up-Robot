#pragma once

#include <math.h>
#include <Aria.h>
#include <memory>

/*
 * AUTHOR: Denis Wagner 
 */

namespace AriaMath
{
const long long PI = acos(-1);

template <typename T>
inline T sqr(T value)
{
    return value * value;
}

template <typename T>
inline T radToDeg(T rad)
{
    return rad * 180 / PI;
}

template <typename T>
inline T degToRad(T rad)
{
    return rad / 180 * PI;
}

inline double getLength(const ArPose &pose)
{
    return sqrt(sqr(pose.getX()) + sqr(pose.getY()));
}

inline ArPose normalize(const ArPose &pose)
{
    double length = getLength(pose);
    double x = pose.getX();
    double y = pose.getY();
    x /= length;
    y /= length;

    return ArPose(x, y, pose.getTh());
}

inline double getRotAngleDeg(const ArPose &pose)
{
    ArPose normalizedPose = normalize(pose);
    double newThXRad = acos(normalizedPose.getX());
    double newThYRad = asin(normalizedPose.getY());
    double newTh = 0;

    if (newThYRad < 0)
    {
        newTh = 0 - newThXRad;
    }
    else
    {
        newTh = newThXRad;
    }

    return radToDeg(newTh);
}
} // namespace AriaMath
