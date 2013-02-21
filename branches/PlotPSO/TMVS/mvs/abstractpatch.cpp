#include "abstractpatch.h"

using namespace PAIS;

int AbstractPatch::globalId = 0;

AbstractPatch::AbstractPatch(const int id) {
	if (id < 0) {
		#pragma omp critical
		{
			this->id = globalId++;
		}
	} else {
		this->id = id;
	}

	init();
}

AbstractPatch::~AbstractPatch(void) {

}

void AbstractPatch::init() {
	center      = Vec3d(0.0, 0.0, 0.0);
	camIdx.clear();
	refCamIdx   = -1;
	normalS     = Vec2d(0.0, 0.0);
	normal      = Vec3d(0.0, 0.0, 0.0);
	ray         = Vec3d(0.0, 0.0, 0.0);
	depth       = 0;
	depthRange  = Vec2d(0.0, 0.0);
	LOD         = -1;
	color       = Vec3b(0, 0, 0);
	imgPoint.clear();
	corrTable   = Mat_<double>(0, 0);
	fitness     = DBL_MAX;
	priority    = DBL_MAX;
	correlation = 0;
	expanded    = false;
}

void AbstractPatch::setNormal(const Vec3d &n) {
	normal = n;
	Utility::normal2Spherical(normal, normalS);
}

void AbstractPatch::setNormal(const Vec2d &n) {
	normalS = n;
	Utility::spherical2Normal(normalS, normal);
}