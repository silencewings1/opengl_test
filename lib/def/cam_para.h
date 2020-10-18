#pragma once

namespace CameraPara
{

#ifdef USE_STEREO

constexpr double cx = 1004.6;
constexpr double cy = 253.5;
constexpr double fx = 1096.6;
constexpr double fy = 1101.2;
constexpr double baseline = 4.0496;

#elif USE_IPHONE7P

constexpr double cx = 2016.0;
constexpr double cy = 1512.0;
constexpr double fx = 3289.914;
constexpr double fy = 3289.914;

#endif

} // namespace CameraPara