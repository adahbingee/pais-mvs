#include "patch.h"

using namespace PAIS;

int Patch::globalId = 0;

Patch::Patch(const MVS *mvs, const Vec3d &center, const Vec3b &color, const vector<int> &camIdx, const vector<Vec2d> &imgPoint, const int id) {
	this->mvs       = mvs;
	this->center    = center;
	this->color     = color;
	this->camIdx    = camIdx;
	this->imgPoint  = imgPoint;
	this->normalS   = Vec2d(0.0, 0.0);
	this->normal    = Vec3d(0.0, 0.0, 0.0);
	this->depth     = -1;
	this->refCamIdx = -1;
	this->fitness   = DBL_MAX;
	this->LOD = -1;
	
	if (id < 0) {
		#pragma omp critical
		{
			this->id = getNextId();
		}
	} else {
		this->id = id;
	}
}

Patch::~Patch(void) {

}

void Patch::refineSeed() {
	bool pass = true;
	pass &= setEstimatedNormal();
	pass &= setReferenceCameraIndex();
	pass &= setDepth();
	pass &= setLOD();

	if ( !pass ) {
		printf("fail during setting\n");
		return;
	}

	// PSO parameter range (theta, phi, depth)
	double rangeL [] = {0.0 , normalS[1] - M_PI/2.0, depth-0.01};
	double rangeU [] = {M_PI, normalS[1] + M_PI/2.0, depth+0.01};

	// initial guess particle
	double init   [] = {normalS[0], normalS[1], depth};

	PsoSolver solver(3, rangeL, rangeU, getFitness, this, 1000, 30);
	solver.setParticle(init);
	solver.run();
	
	// set fitness
	fitness = solver.getGbestFitness();

	// set refined patch information
	const double *gBest = solver.getGbest();
	setNormal(Vec2d(gBest[0], gBest[1]));
	center = ray * gBest[2] + mvs->getCameras()[refCamIdx].getCenter();

	printf("LOD: %d\t%f\n", LOD, fitness);
}

/* setters */

void Patch::setNormal(const Vec3d &n) {
	normal = n;
	Utility::normal2Spherical(normal, normalS);
}

void Patch::setNormal(const Vec2d &n) {
	normalS = n;
	Utility::spherical2Normal(normalS, normal);
}

bool Patch::setDepth() {
	if (refCamIdx < 0)       return false;
	
	ray = center - mvs->getCameras()[refCamIdx].getCenter();
	depth = norm(ray);
	ray = ray * (1.0 / depth);
	
	return true;
}

bool Patch::setLOD() {
	if (refCamIdx < 0) {
		printf("Reference camera index not set\n");
		return false;
	}

	// patch size
	int patchRadius = mvs->getPatchRadius();
	int size        = mvs->getPatchSize();

	// reference camera
	const Camera &refCam = mvs->getCameras()[refCamIdx];
	// image pyramid of reference image
	const vector<Mat_<uchar> > &pyramid = refCam.getPyramidImage();

	// texture info
	double mean = 0;
	double variance = 0;
	int count;

	// textures in window
	uchar *textures = new uchar[size*size];
	
	// projected point on image
	Vec2d pt;
	
	// initialize level of detail
	LOD = -1;

	// find LOD
	while (variance < mvs->getTextureVariation()) {
		// goto next LOD
		LOD++;

		// return if reach the max LOD
		if (LOD >= pyramid.size()) {
			delete [] textures;
			return false;
		}

		// LOD-- if out of image bound
		if ( !refCam.project(center, pt, LOD) ) {
			//printf("setLOD image point out of image bound: LOD %d, x: %f, y: %f\n", LOD, pt[0], pt[1]);
			LOD = max(LOD-1, 0);
			delete [] textures;
			return true;
		}

		// clear
		mean     = 0;
		variance = 0;
		count    = 0;

		// get mean
		for (int x = cvRound(pt[0])-patchRadius; x <= cvRound(pt[0])+patchRadius; x++) {
			for (int y = cvRound(pt[1])-patchRadius; y <= cvRound(pt[1])+patchRadius; y++) {
				if ( !refCam.inImage(x, y, LOD) ) {
					//printf("setLOD image point out of image bound: LOD %d, x: %d, y: %d\n", LOD, x, y);
					LOD = max(LOD-1, 0);
					delete [] textures;
					return true;
				}
				textures[count] = pyramid[LOD].at<uchar>(y, x);
				mean += textures[count];
				count++;
			}
		}
		mean /= count;

		// get variance
		for (int i = 0; i < count; i++) {
			variance += (textures[i]-mean)*(textures[i]-mean);
		}
		variance /= count; 
	}

	/* show pyramid */
	/*
	if (LOD > 0) {
		char title[30];
		for (int l = 0; l <= LOD; l++) {
			refCam.project(center, pt, l);
			sprintf(title, "LOD %d", l);
			Mat img = pyramid[l].clone();
			circle(img, Point(cvRound(pt[0]), cvRound(pt[1])), max(patchRadius, 5), Scalar(255,255,255), 2, CV_AA);
			imshow(title, img);
			cvMoveWindow(title, 0, 0);
		}
		waitKey();
		destroyAllWindows();
	}
	*/
	
	delete [] textures;

	printf("v %f\n", variance);
	return true;
}

bool Patch::setEstimatedNormal() {
	const int camNum = getCameraNumber();
	if (camNum == 0)         return false;

	Vec3d dir;
	normal = Vec3d(0.0, 0.0, 0.0);
	for (int i = 0; i < camNum; i++) {
		const Camera &cam = mvs->getCameras()[camIdx[i]];
		dir = cam.getCenter() - center;
		dir *= (1.0 / norm(dir));
		normal += dir;
	}
	normal *= (1.0 / norm(normal));
	
	setNormal(normal);

	return true;
}

bool Patch::setReferenceCameraIndex() {
	const int camNum = getCameraNumber();
	if (camNum == 0)         return false;

	refCamIdx = -1;
	double maxCorr = -DBL_MAX;
	double corr;
	for (int i = 0; i < camNum; i++) {
		const Camera &cam = mvs->getCameras()[camIdx[i]];
		corr = normal.ddot(-cam.getOpticalNormal());

		if (corr > maxCorr) {
			maxCorr = corr;
			refCamIdx = camIdx[i];
		}
	}
	return true;
}

double PAIS::getFitness(const Particle &p, void *obj) {
	const Patch  &patch   = *((Patch *)obj);
	const MVS    &mvs     = patch.getMVS();
	const int patchRadius = mvs.getPatchRadius();

	// camera parameters
	const Camera &refCam  = mvs.getCameras()[patch.getReferenceCameraIndex()];
	const Mat_<double> &KRF = refCam.getKR();
	const Mat_<double> &KTF = refCam.getKT();
	const int camNum = patch.getCameraNumber();

	// given patch normal
	Vec3d normal;
	Utility::spherical2Normal(Vec2d(p.pos[0], p.pos[1]), normal);

	// skip inversed normal
	if (normal.ddot(refCam.getOpticalNormal()) > 0) {
        return DBL_MAX;
    }

	// skip negative depth
	if (p.pos[2] <= 0) {
		return DBL_MAX;
	}

	// given patch center
	Vec3d center = patch.getRay() * p.pos[2] + refCam.getCenter();

	// plane equation (distance form plane to origin)
	const double d = -patch.getCenter().ddot(normal);          
	const Mat_<double> normalM(normal);

	// Homographies to visible camera
	vector<Mat_<double> > H(camNum);
	for (int i = 0; i < camNum; i++) {
		// visible camera
		const Camera &cam = mvs.getCameras()[patch.getCameraIndices()[i]];

		// get homography matrix
		const Mat_<double> &KRT = cam.getKR();        // K*R of to camera
		const Mat_<double> &KTT = cam.getKT();        // K*T of to camera
		H[i] = ( d*KRT - KTT*normalM.t() ) * ( d*KRF - KTF*normalM.t() ).inv();
	}

	// projected point on reference image
	Vec2d pt;
	refCam.project(patch.getCenter(), pt);

	// level of detail
	int LOD = patch.getLOD();

	// warping (get pixel-wised variance)
	double mean, variance;
	double w, ix, iy;
	int px[4]; // neighbor x
	int py[4]; // neighbor y
	double *c = new double [camNum]; // color
	double fitness = 0;
	
	for (double x = pt[0]-patchRadius; x <= pt[0]+patchRadius; x++) {
		for (double y = pt[1]-patchRadius; y <= pt[1]+patchRadius; y++) {
			// clear
			mean     = 0;
			variance = 0;

			for (int i = 0; i < camNum; i++) {
				const Mat_<uchar> &img = mvs.getCameras()[patch.getCameraIndices()[i]].getPyramidImage()[LOD];

				// homography projection
				w  =   H[i].at<double>(2, 0) * x + H[i].at<double>(2, 1) * y + H[i].at<double>(2, 2);
				ix = ( H[i].at<double>(0, 0) * x + H[i].at<double>(0, 1) * y + H[i].at<double>(0, 2) ) / w;
				iy = ( H[i].at<double>(1, 0) * x + H[i].at<double>(1, 1) * y + H[i].at<double>(1, 2) ) / w;
				
				// apply LOD transform
				ix /= 1<<LOD;
				iy /= 1<<LOD;

				// skip overflow cases
				if ((ix < 0) | (ix >= img.cols-1) | (iy < 0) | (iy >= img.rows-1) | (w == 0)) {
					delete [] c;
					return DBL_MAX;
				}

				// interpolation neighbor points
				px[0] = (int) ix;
				py[0] = (int) iy;
				px[1] = px[0] + 1;
				py[1] = py[0];
				px[2] = px[0];
				py[2] = py[0] + 1;
				px[3] = px[0] + 1;
				py[3] = py[0] + 1;

				c[i] = (double) img.at<uchar>(py[0], px[0])*(px[1]-ix)*(py[2]-iy) + 
					   (double) img.at<uchar>(py[1], px[1])*(ix-px[0])*(py[3]-iy) + 
					   (double) img.at<uchar>(py[2], px[2])*(iy-py[0])*(px[3]-ix) + 
					   (double) img.at<uchar>(py[3], px[3])*(ix-px[2])*(iy-py[1]);

				mean += c[i];
			} // end of camera

			mean /= camNum;

			for (int i = 0; i < camNum; i++) {
				variance += abs(c[i]-mean);
			}

			fitness += variance;
		} // end of warping y
	} // end of warping x

	delete [] c;
	return fitness;
}