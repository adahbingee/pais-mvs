#include "camera.h"

using namespace PAIS;

// static function
Mat_<double> Camera::quaternionToRotationMat(const Vec4d &q) {

    const double qq = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    double qw, qx, qy, qz;
    if(qq>0)
    {
        qw = q[0] / qq;
        qx = q[1] / qq;
        qy = q[2] / qq;
        qz = q[3] / qq;
    } else {
        qw = 1;
        qx = qy = qz = 0;
    }

    Mat_<double> R(3,3);

    R.at<double>(0, 0) = qw*qw + qx*qx - qz*qz - qy*qy;
    R.at<double>(0, 1) = 2*qx*qy - 2*qz*qw;
    R.at<double>(0, 2) = 2*qy*qw + 2*qz*qx;
    R.at<double>(1, 0) = 2*qx*qy + 2*qw*qz;
    R.at<double>(1, 1) = qy*qy+ qw*qw - qz*qz - qx*qx;
    R.at<double>(1, 2) = 2*qz*qy - 2*qx*qw;
    R.at<double>(2, 0) = 2*qx*qz - 2*qy*qw;
    R.at<double>(2, 1) = 2*qy*qz + 2*qw*qx;
    R.at<double>(2, 2) = qz*qz + qw*qw - qy*qy - qx*qx;

    return R;
}

string Camera::getEdgeFileName(const char *fileName) {
	string fileNameStr(fileName);
	size_t split = fileNameStr.find_last_of(".");
	string edgeFileNameStr = fileNameStr.substr(0, split);
	edgeFileNameStr.append("e.png");
	return edgeFileNameStr;
}

// object function
Camera::Camera(void) {
	_isAvaliable = false;
}

Camera::~Camera(void) {
	
}

Camera::Camera(const char *fileName, const double focal, const Vec4d &quaternion, const Vec3d &center, const double radialDistortion) {
	_isAvaliable = false;

	// get edge image file name
	// string edgeFileName = Camera::getEdgeFileName(fileName);

	// read RGB image
	imgRGB = imread(fileName);

	// can't read image file
	if (imgRGB.data == NULL) {
		printf("Can't read image file %s\n", fileName);
		return;
	}

	// copy image file name
	strcpy(this->fileName, fileName);

	// get max level of detail
	maxLOD = (int) ( log( (double) max(imgRGB.cols, imgRGB.rows) ) / log(2.0) );
	maxLOD = min(maxLOD, 5);

	// read gray level image pyramid
	imgPyramid.resize(maxLOD);
	edgePyramid.resize(maxLOD);
	imgPyramid[0] = imread(fileName, 0);

	Mat_<uchar> gradientX, gradientY;
	Sobel(imgPyramid[0], gradientX, CV_8U, 1, 0, 1);
	Sobel(imgPyramid[0], gradientY, CV_8U, 0, 1, 1);
	edgePyramid[0] = abs(gradientX + gradientY);
	
	#pragma omp parallel for
	for (int i = 1; i < maxLOD; i++) {
		double size = 1.0 / (1<<i);
		resize(imgPyramid[0], imgPyramid[i], Size(), size, size, INTER_AREA);
		Mat_<uchar> gradientX, gradientY;
		Sobel(imgPyramid[i], gradientX, CV_8U, 1, 0, 1);
		Sobel(imgPyramid[i], gradientY, CV_8U, 0, 1, 1);
		edgePyramid[i] = abs(gradientX + gradientY);
	}

	// set focal length
	this->focal = focal;

	// set radial distortion
	this->radialDistortion = radialDistortion;

	// get principle point
	principlePoint[0] = imgRGB.cols >> 1;
	principlePoint[1] = imgRGB.rows >> 1;

	// set intrisic matrix
	double intrisic_data [] = {focal,   0.0, principlePoint[0],
	                               0, focal, principlePoint[1],
	                               0,     0,                 1};
	intrinsic = Mat_<double>(3, 3, intrisic_data);
	
	// set rotation matrix
	this->quaternion = quaternion;
	this->rotation = quaternionToRotationMat(quaternion);

	// set camera center and translation
	this->center = center;
	this->translation = -rotation * Mat(center);

	// set projection matrix
	this->KR = intrinsic * rotation;
	this->KT = intrinsic * translation;
	this->P  = Mat_<double>(3, 4);
	KR.copyTo(P(Rect(0,0,3,3)));
    KT.copyTo(P(Rect(3,0,1,3)));

	// set camera optical normal
	double dir_data [] = {0.0, 0.0, 1.0};
    Mat dir(3, 1, CV_64FC1, dir_data);
    dir = rotation.t() * dir; 
    opticalNormal = Vec3d(dir);
	
	_isAvaliable = true;
}

Camera::Camera(const char *fileName, const double focalX, const double focalY, const Vec2d &principlePoint, const Vec4d &quaternion, const Vec3d &center) {
	_isAvaliable = false;

	// read RGB image
	imgRGB = imread(fileName);

	// can't read image file
	if (imgRGB.data == NULL) {
		printf("Can't read image file %s\n", fileName);
		return;
	}

	// copy image file name
	strcpy(this->fileName, fileName);

	// read gray level image pyramid
	maxLOD = (int) ( log( (double) max(imgRGB.cols, imgRGB.rows) ) / log(2.0) );
	imgPyramid.resize(maxLOD);
	imgPyramid[0] = imread(fileName, 0);
	for (int i = 1; i < maxLOD; i++) {
		resize(imgPyramid[i-1], imgPyramid[i], Size(), 0.5, 0.5, INTER_AREA);
	}

	this->focalX = focalX;
	this->focalY = focalY;

	_isAvaliable = true;
}

bool Camera::project(const Vec3d &in3D, Vec2d &out2D, const int LOD, const bool applyDistortion) const {

	if ( !applyDistortion ) {
		// without radial distortion
		Mat X2 = rotation * Mat(in3D, false) + translation;
		out2D[0] = focal * ( X2.at<double>(0,0) / X2.at<double>(2,0) ) + principlePoint[0];
		out2D[1] = focal * ( X2.at<double>(1,0) / X2.at<double>(2,0) ) + principlePoint[1];
		out2D[0] /= 1<<LOD;
		out2D[1] /= 1<<LOD;
	} else {
		// with radial distortion
		Mat X2 = rotation * Mat(in3D, false) + translation;
		out2D[0] = focal * ( X2.at<double>(0,0) / X2.at<double>(2,0) );
		out2D[1] = focal * ( X2.at<double>(1,0) / X2.at<double>(2,0) );
		double r = radialDistortion * (out2D[0]*out2D[0] + out2D[1]*out2D[1]);
		out2D    = (1.0+r) * out2D + principlePoint;
		out2D[0] /= 1<<LOD;
		out2D[1] /= 1<<LOD;
	}

	return inImage(out2D, LOD);
}