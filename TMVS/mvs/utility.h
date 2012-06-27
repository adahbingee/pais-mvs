#ifndef PAIS_MVS_UTILITY_H
#define PAIS_MVS_UTILITY_H
#define _USE_MATH_DEFINES

#include <opencv2\opencv.hpp>

//#include "camera.h"

//using namespace PAIS;
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

		/*
		// get fundamental matrix xT'*F*xF = 0
		inline static Mat getFundamental(const Camera &camFrom, const Camera &camTo) {
			// F = eTx * pT * pF^-1;
			// eT = pT * C;
			const Mat &pF   = camFrom.getP();
			const Mat &pT   = camTo.getP();
			const Vec3d &cF = camFrom.getCenter();
			double cFData [] = {cF[0], cF[1], cF[2], 1.0};
		    const Mat cFM(4, 1, CV_64FC1, cFData);
			const Mat eT = pT * cFM;
			double exTData [] = {0, -eT.at<double>(2, 0), eT.at<double>(1, 0),
				                 eT.at<double>(2, 0), 0, -eT.at<double>(0, 0),
								 -eT.at<double>(1, 0), eT.at<double>(0, 0), 0};
			const Mat exT(3, 3, CV_64FC1, exTData);
			const Mat pFinv = pF.inv(DECOMP_SVD);
			return exT*pT*pFinv;
		}
		*/
	};
};


#endif