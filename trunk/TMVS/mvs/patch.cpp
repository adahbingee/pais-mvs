#include "patch.h"

using namespace PAIS;

int Patch::globalId = 0;

/* constructors & descructor */

Patch::Patch(const MVS *mvs, const Vec3d &center, const Vec3b &color, const vector<int> &camIdx, const vector<Vec2d> &imgPoint, const int id) {
	this->mvs        = mvs;
	this->center     = center;
	this->color      = color;
	this->camIdx     = camIdx;
	this->imgPoint   = imgPoint;
	this->normalS    = Vec2d(0.0, 0.0);
	this->normal     = Vec3d(0.0, 0.0, 0.0);
	this->depth      = -1;
	this->refCamIdx  = -1;
	this->fitness    = DBL_MAX;
	this->LOD        = -1;
	this->depthRange = Vec2d(0.0, 0.0);
	
	if (id < 0) {
		#pragma omp critical
		{
			this->id = globalId++;
		}
	} else {
		this->id = id;
	}
}

Patch::~Patch(void) {

}

/* public functions */

void Patch::refineSeed() {
	bool pass = true;
	pass &= setEstimatedNormal();
	pass &= setReferenceCameraIndex();
	pass &= setDepth();
	pass &= setDepthRange();
	if (LOD < 0) pass &= setLOD();

	if ( !pass ) {
		printf("fail during setting\n");
		return;
	}

	// PSO parameter range (theta, phi, depth)
	double rangeL [] = {0.0 , normalS[1] - M_PI/2.0, depthRange[0]};
	double rangeU [] = {M_PI, normalS[1] + M_PI/2.0, depthRange[1]};

	// initial guess particle
	double init   [] = {normalS[0], normalS[1], depth};

	PsoSolver solver(3, rangeL, rangeU, getFitness, this, 200/(LOD+1), 15);
	solver.setParticle(init);
	solver.run(true);
	
	// set fitness
	fitness = solver.getGbestFitness();

	// set refined patch information
	const double *gBest = solver.getGbest();
	setNormal(Vec2d(gBest[0], gBest[1]));
	depth  = gBest[2];
	center = ray * depth + getReferenceCamera().getCenter();

	printf("ID: %d\tLOD: %d\tit: %d\tfit: %.2f\n", id, LOD, solver.getIteration(), fitness);
	
	// LOD down refinement
	if (LOD > 0) {
		LOD = max(LOD-1, 0);
		refineSeed();
	} else {
		//showError();
		//showRefinedResult();
		printf("\n");
	}
}

/* for debug used */

void Patch::showRefinedResult() const {
	static const vector<Camera> &cameras = mvs->getCameras();
	static const int patchRadius = mvs->getPatchRadius();

	// camera parameters
	const Camera &refCam  = getReferenceCamera();
	const Mat_<double> &KRF = refCam.getKR();
	const Mat_<double> &KTF = refCam.getKT();
	const int camNum = getCameraNumber();

	// plane equation (distance form plane to origin)
	const double d = -center.ddot(normal);          
	const Mat_<double> normalM(normal);

	// Homographies to visible camera
	vector<Mat_<double> > H(camNum);
	for (int i = 0; i < camNum; i++) {
		// visible camera
		const Camera &cam = cameras[camIdx[i]];

		// get homography matrix
		const Mat_<double> &KRT = cam.getKR();        // K*R of to camera
		const Mat_<double> &KTT = cam.getKT();        // K*T of to camera
		H[i] = ( d*KRT - KTT*normalM.t() ) * ( d*KRF - KTF*normalM.t() ).inv();
	}

	Vec2d pt;
	refCam.project(center, pt);
	
	double x[5] = {pt[0]-patchRadius, pt[0]-patchRadius, pt[0]+patchRadius, pt[0]+patchRadius, pt[0]};
	double y[5] = {pt[1]-patchRadius, pt[1]+patchRadius, pt[1]-patchRadius, pt[1]+patchRadius, pt[1]};

	double w, ix[5], iy[5];
	char title[30];
	for (int i = 0; i < camNum; i++) {
		const Camera &cam = mvs->getCameras()[camIdx[i]];
		Mat_<Vec3b> img = cam.getRgbImage().clone();

		// homography projection
		Mat F = Utility::getFundamental(refCam, cam);
		for (int c = 0; c < 5; c++) {
			w  =   H[i].at<double>(2, 0) * x[c] + H[i].at<double>(2, 1) * y[c] + H[i].at<double>(2, 2);
			ix[c] = ( H[i].at<double>(0, 0) * x[c] + H[i].at<double>(0, 1) * y[c] + H[i].at<double>(0, 2) ) / w;
			iy[c] = ( H[i].at<double>(1, 0) * x[c] + H[i].at<double>(1, 1) * y[c] + H[i].at<double>(1, 2) ) / w;
			ix[c] = cvRound(ix[c]);
			iy[c] = cvRound(iy[c]);

			// draw epipolar line
			Mat_<double> pF(3, 1);
			pF.at<double>(0,0) = x[c];
			pF.at<double>(1,0) = y[c];
			pF.at<double>(2,0) = 1;
			Mat epiLine = F*pF;
			double yStart = -epiLine.at<double>(0, 2)/epiLine.at<double>(0, 1);
			double yEnd   = (-epiLine.at<double>(0, 2)-epiLine.at<double>(0, 0)*img.cols) / epiLine.at<double>(0, 1);
			yStart = cvRound(yStart);
			yEnd   = cvRound(yEnd);
			line(img, Point(0, yStart), Point( img.cols, yEnd), Scalar(0,255,0, 0.5));
		}

		// draw patch
		line(img, Point(ix[0],iy[0]), Point(ix[1],iy[1]), Scalar(0,0,255));
		line(img, Point(ix[0],iy[0]), Point(ix[2],iy[2]), Scalar(0,0,255));
		line(img, Point(ix[3],iy[3]), Point(ix[1],iy[1]), Scalar(0,0,255));
		line(img, Point(ix[3],iy[3]), Point(ix[2],iy[2]), Scalar(0,0,255));
		circle(img, Point(ix[4], iy[4]), 1, Scalar(0,0,255));

		sprintf(title, "img %d", camIdx[i]);
		imshow(title, img);
	}
	waitKey();
	destroyAllWindows();
}

void Patch::showError() const {
	static const vector<Camera> &cameras = mvs->getCameras();
	static const int patchRadius = mvs->getPatchRadius();
	static const int patchSize   = mvs->getPatchSize();

	// camera parameters
	const Camera &refCam  = getReferenceCamera();
	const Mat_<double> &KRF = refCam.getKR();
	const Mat_<double> &KTF = refCam.getKT();
	const int camNum = getCameraNumber();

	// plane equation (distance form plane to origin)
	const double d = -center.ddot(normal);          
	const Mat_<double> normalM(normal);

	// Homographies to visible camera
	vector<Mat_<double> > H(camNum);
	for (int i = 0; i < camNum; i++) {
		// visible camera
		const Camera &cam = cameras[camIdx[i]];

		// get homography matrix
		const Mat_<double> &KRT = cam.getKR();        // K*R of to camera
		const Mat_<double> &KTT = cam.getKT();        // K*T of to camera
		H[i] = ( d*KRT - KTT*normalM.t() ) * ( d*KRF - KTF*normalM.t() ).inv();
	}

	Vec2d pt;
	refCam.project(center, pt);

	// warping (get pixel-wised variance)
	double mean, avgSad;             // pixel-wised mean, avgSad
	double w, ix, iy;                // position on target image
	int px[4];                       // neighbor x
	int py[4];                       // neighbor y
	double *c = new double [camNum]; // bilinear color
	Mat_<double> error(patchSize, patchSize);

	for (double x = pt[0]-patchRadius, ex = 0; x <= pt[0]+patchRadius; x++, ex++) {
		for (double y = pt[1]-patchRadius, ey = 0; y <= pt[1]+patchRadius; y++, ey++) {
			// clear
			mean = 0;
			avgSad = 0;

			for (int i = 0; i < camNum; i++) {
				const Mat_<uchar> &img = cameras[camIdx[i]].getPyramidImage()[LOD];

				// homography projection
				w  =   H[i].at<double>(2, 0) * x + H[i].at<double>(2, 1) * y + H[i].at<double>(2, 2);
				ix = ( H[i].at<double>(0, 0) * x + H[i].at<double>(0, 1) * y + H[i].at<double>(0, 2) ) / w;
				iy = ( H[i].at<double>(1, 0) * x + H[i].at<double>(1, 1) * y + H[i].at<double>(1, 2) ) / w;
				
				// apply LOD transform
				if (LOD > 0) {
					ix /= 1<<LOD;
					iy /= 1<<LOD;
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
				avgSad += abs(c[i]-mean);
			}
			avgSad /= camNum;

			error.at<double>(ey, ex) = avgSad;
		} // end of warping y
	} // end of warping x

	double minE ,maxE;
	minMaxLoc(error, &minE, &maxE);
	error = (error - minE) / (maxE - minE) * 255;
	imwrite("img.png", error);
}

/* getters */
const Camera& Patch::getReferenceCamera() const {
	return mvs->getCameras()[refCamIdx];
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

bool Patch::setDepthRange() {
	const int camNum = getCameraNumber();

	if (camNum <= 0 || depth < 0) return false;

	const Camera &refCam = getReferenceCamera();

	const double cellSize = mvs->getCellSize();

	// center shift
	Vec3d c2 = ray * (depth+1.0) + refCam.getCenter();
	// projected point
	Vec2d p1, p2;

	double imgDist, worldDist;
	double maxWorldDist = -DBL_MAX;
	for (int i = 0; i < camNum; i++) {

		if (camIdx[i] == refCamIdx) continue;
		
		const Camera &cam = mvs->getCameras()[camIdx[i]];

		cam.project(c2, p2);
		cam.project(center, p1);

		imgDist = norm(p1-p2);
		worldDist = max(cellSize, 5.0) / imgDist;

		if (worldDist > maxWorldDist) {
			maxWorldDist = worldDist;
		}
	}

	depthRange[0] = depth - maxWorldDist;
	depthRange[1] = depth + maxWorldDist;
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

/* fitness function */

double PAIS::getFitness(const Particle &p, void *obj) {
	// current patch
	const Patch  &patch   = *((Patch *)obj);
	// visible camera indices
	const vector<int> &cameraIdx = patch.getCameraIndices();
	// static instances
	static const MVS  &mvs               = patch.getMVS();
	static const int patchRadius         = mvs.getPatchRadius();
	static const vector<Camera> &cameras = mvs.getCameras();

	// camera parameters
	const Camera &refCam  = patch.getReferenceCamera();
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

	// given patch center
	const Vec3d center = patch.getRay() * p.pos[2] + refCam.getCenter();

	// plane equation (distance form plane to origin)
	const double d = -center.ddot(normal);          
	const Mat_<double> normalM(normal);

	// Homographies to visible camera
	vector<Mat_<double> > H(camNum);
	for (int i = 0; i < camNum; i++) {
		// visible camera
		const Camera &cam = cameras[cameraIdx[i]];

		// get homography matrix
		const Mat_<double> &KRT = cam.getKR();        // K*R of to camera
		const Mat_<double> &KTT = cam.getKT();        // K*T of to camera
		H[i] = ( d*KRT - KTT*normalM.t() ) * ( d*KRF - KTF*normalM.t() ).inv();
	}

	// projected point on reference image
	Vec2d pt;
	if ( !refCam.project(center, pt) ) {
		return DBL_MAX;
	}

	// level of detail
	int LOD = patch.getLOD();

	// warping (get pixel-wised variance)
	double mean, avgSad;           // pixel-wised mean, average sad
	double w, ix, iy;                // position on target image
	int px[4];                       // neighbor x
	int py[4];                       // neighbor y
	double *c = new double [camNum]; // bilinear color
	double fitness = 0;              // result of normalized fitness
	// distance weighting
	static const Mat_<double> &distWeight = mvs.getPatchDistanceWeighting();
	Mat_<double>::const_iterator it = distWeight.begin();
	// sum of weighting
	double sumWeight = 0;
	double weight;

	for (double x = pt[0]-patchRadius; x <= pt[0]+patchRadius; x++) {
		for (double y = pt[1]-patchRadius; y <= pt[1]+patchRadius; y++) {
			// clear
			mean   = 0;
			avgSad = 0;

			for (int i = 0; i < camNum; i++) {
				const Mat_<uchar> &img = cameras[cameraIdx[i]].getPyramidImage()[LOD];

				// homography projection (with LOD transform)
				w  = ( H[i].at<double>(2, 0) * x + H[i].at<double>(2, 1) * y + H[i].at<double>(2, 2) ) * (1<<LOD);
				ix = ( H[i].at<double>(0, 0) * x + H[i].at<double>(0, 1) * y + H[i].at<double>(0, 2) ) / w;
				iy = ( H[i].at<double>(1, 0) * x + H[i].at<double>(1, 1) * y + H[i].at<double>(1, 2) ) / w;

				// skip overflow cases
				if (ix < 0 || ix >= img.cols-1 || iy < 0 || iy >= img.rows-1 || w == 0) {
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
				
				// c[i] = img.at<uchar>(cvRound(iy), cvRound(ix));

				mean += c[i];
			} // end of camera

			mean /= camNum;

			for (int i = 0; i < camNum; i++) {
				avgSad += abs(c[i]-mean);
			}
			avgSad /= camNum;

			weight = (*it++) * exp(-(avgSad*avgSad)/16384.0);
			sumWeight += weight;
			fitness += avgSad * weight;
		} // end of warping y
	} // end of warping x

	delete [] c;
	return fitness / sumWeight;
}