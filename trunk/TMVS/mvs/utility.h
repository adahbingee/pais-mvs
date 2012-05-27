#ifndef PAIS_MVS_UTILITY_H
#define PAIS_MVS_UTILITY_H
#define _USE_MATH_DEFINES

#include <opencv2\opencv.hpp>

using namespace cv;

namespace PAIS {

    class Utility {
    public:
        // normal to spherical coordinate theta, phi
        inline static void normal2Spherical(const Vec3d &in, Vec2d &out) {
            // 0 <= theta <= pi
            out[0] = acos(in[2]);
            // -pi <= phi <= pi
            out[1] = atan2(in[1], in[0]);
        }
        
        // spherical coordinate to normal
        inline static void spherical2Normal(const Vec2d &in, Vec3d &out) {
            out[0] = sin(in[0])*cos(in[1]);
            out[1] = sin(in[0])*sin(in[1]);
            out[2] = cos(in[0]);
        }
	};
};


#endif