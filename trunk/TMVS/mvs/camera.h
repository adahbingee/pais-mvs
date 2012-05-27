#ifndef __PAIS_CAMERA_H__
#define __PAIS_CAMERA_H__

#define MAX_FILE_NAME_LENGTH 256
#include <math.h>
#include <vector>

#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

namespace PAIS {
	class Camera {
	private:
		// flag for camera is avaliable
		bool _isAvaliable;

		// max level of detail in image pyramid
		int maxLOD;

		// full image path
		char fileName[MAX_FILE_NAME_LENGTH];

		// RGB image in original size
		Mat_<Vec3b> imgRGB;

		// forground mask in original size
		Mat_<bool> imgMask;

		// gray level image pyramid from 0 = original size to vector size = 1 pixel size
		vector<Mat_<uchar> > imgPyramid;

		// camera focal length, K = [f, 0, cx; 0 f cy; 0 0 1]
		double focal;

		// radial distortion parameter
		double radialDistortion;

		// principle point in original size image
		Vec2d principlePoint;

		// camera intrisic matrix
		Mat_<double> intrinsic;

		// camera rotation matrix
		Mat_<double> rotation;

		// camera rotation in quaternion
		Vec4d quaternion;

		// camera translation matrix
		Mat_<double> translation;

		// projection matrix
		Mat_<double> KR;
		Mat_<double> KT;

		// camera center in world coordinate
		Vec3d center;

		// camera optical normal
		Vec3d opticalNormal;

		// convert quaternion to rotation matrix 
		static Mat_<double> Camera::quaternionToRotationMat(const Vec4d &q);

	public:
		Camera(void);
		Camera(const char *fileName, const double focal, const Vec4d &quaternion, const Vec3d &center, const double radialDistortion);
		~Camera(void);

		// get image information
		const char* getFileName()                       const { return fileName;         }
		const Mat_<Vec3b>& getRgbImage()                const { return imgRGB;           }
		const Mat_<bool>& getMaskImage()                const { return imgMask;          }
		const vector<Mat_<uchar> >& getPyramidImage()   const { return imgPyramid;       }
		const Mat_<uchar>& getPyramidImage(const int i) const { return imgPyramid[i];    }

		// get intrinsic information
		double getFocalLength()                         const { return focal;            }
		double getRadialDistortion()                    const { return radialDistortion; }
		const Vec2d& getPrinciplePoint()                const { return principlePoint;   }
		const Vec3d& getCenter()                        const { return center;           }
		const Mat_<double>& getIntrinsic()              const { return intrinsic;        }
		int getImageWidth()                             const { return imgRGB.cols;      }
		int getImageHeight()                            const { return imgRGB.rows;      }

		// get extrinsic information
		const Vec4d& getQuaternion()                    const { return quaternion;       }
		const Mat_<double>& getRotation()               const { return rotation;         }
		const Mat_<double>& getTranslation()            const { return translation;      }
		const Vec3d& getOpticalNormal()                 const { return opticalNormal;    }

		// get projection matrix
		const Mat_<double> getKR()                      const { return KR;               }
		const Mat_<double> getKT()                      const { return KT;               }

		// check camera status is avaliable
		bool isAvaliable()                              const { return _isAvaliable;     }

		// project a 3D point to image using a specified level of detail image (0 for original size)
		// and return is in image or not
		bool project(const Vec3d &in3D, Vec2d &out2D, const int LOD = 0) const;
		
		// get 2d point is in image or not using a specified level of detail image (0 for original size) 
		bool inImage(const Vec2d &in2D, const int LOD) const {
			if (_isnan(in2D[0]) || _isnan(in2D[1])) {
				return false;
			}

			if ( in2D[0] < 0 || in2D[0] >= imgPyramid[LOD].cols || in2D[1] < 0 || in2D[1] >= imgPyramid[LOD].rows) {
				return false;
			} else {
				return true;
			}
		}

		bool inImage(const int x, const int y, const int LOD) const {
			if (_isnan(x) || _isnan(y)) {
				return false;
			}

			if ( x < 0 || x >= imgPyramid[LOD].cols || y < 0 || y >= imgPyramid[LOD].rows) {
				return false;
			} else {
				return true;
			}
		}
	};
};

#endif