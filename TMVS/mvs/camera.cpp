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

// object function
Camera::Camera(void) {
	_isAvaliable = false;
}

Camera::~Camera(void) {
	
}

Camera::Camera(const char *fileName, const double focal, const Vec4d &quaternion, const Vec3d &center, const double radialDistortion) {
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

	// set camera optical normal
	double dir_data [] = {0.0, 0.0, 1.0};
    Mat dir(3, 1, CV_64FC1, dir_data);
    dir = rotation.t() * dir; 
    opticalNormal = Vec3d(dir);
	
	_isAvaliable = true;
}

bool Camera::project(const Vec3d &in3D, Vec2d &out2D, const int LOD) const {
	Mat X2 = rotation * Mat(in3D, false) + translation;
	out2D[0] = focal * ( X2.at<double>(0,0) / X2.at<double>(2,0) ) + principlePoint[0];
    out2D[1] = focal * ( X2.at<double>(1,0) / X2.at<double>(2,0) ) + principlePoint[1];
	out2D[0] /= 1<<LOD;
	out2D[1] /= 1<<LOD;

	return inImage(out2D, LOD);
}