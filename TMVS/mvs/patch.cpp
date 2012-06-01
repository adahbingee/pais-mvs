#include "patch.h"

using namespace PAIS;

int Patch::globalId = 0;

bool Patch::isNeighbor(const Patch& pth1, const Patch &pth2) {
	const Vec3d &c1 = pth1.getCenter();
	const Vec3d &c2 = pth2.getCenter();
	const Vec3d &n1 = pth1.getNormal();
	const Vec3d &n2 = pth2.getNormal();

	double dist = 0;
	dist += abs((c1-c2).ddot(n1));
	dist += abs((c1-c2).ddot(n2));

	if (dist < 0.1)
		return true;
	else 
		return false;
}

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
	this->priority   = DBL_MAX;
	this->expanded   = false;
	
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
	pass &= setLOD();

	if ( !pass ) {
		printf("fail during setting\n");
		return;
	}

	// PSO parameter range (theta, phi, depth)
	double rangeL [] = {0.0 , normalS[1] - M_PI/2.0, depthRange[0]};
	double rangeU [] = {M_PI, normalS[1] + M_PI/2.0, depthRange[1]};

	// initial guess particle
	double init   [] = {normalS[0], normalS[1], depth};

	PsoSolver solver(3, rangeL, rangeU, getFitness, this, 60, 15);
	solver.setParticle(init);
	solver.run(true);
	
	// set fitness
	fitness = solver.getGbestFitness();

	// set refined patch information
	const double *gBest = solver.getGbest();
	setNormal(Vec2d(gBest[0], gBest[1]));
	depth  = gBest[2];
	center = ray * depth + getReferenceCamera().getCenter();

	// set normalized homography patch correlation table
	setCorrelationTable();
	
	// remove insvisible camera
	int beforeCamNum = getCameraNumber();
	removeInvisibleCamera();
	int afterCamNum = getCameraNumber();
	if (beforeCamNum != afterCamNum && afterCamNum >= MIN_CAMERA_NUMBER) {
		refineSeed();
	}

	// set patch priority
	setPriority();

	printf("ID: %d\tLOD: %d\tit: %d\tfit: %.2f \tpri: %.2f\n", id, LOD, solver.getIteration(), fitness, priority);
}

void Patch::expand() const {
	const int camNum = getCameraNumber();
	const int cellSize = mvs->getCellSize();
	const vector<CellMap> &cellMaps = mvs->getCellMaps();

	int cx, cy;
	for (int i = 0; i < camNum; ++i) {
		// image cell map
		const CellMap &map = cellMaps[camIdx[i]];

		// position on cell map
		cx = imgPoint[i][0] / cellSize;
		cy = imgPoint[i][1] / cellSize;

		// check neighbor cells
		int nx [] = {cx, cx-1, cx-1, cx+1, cx+1};
		int ny [] = {cy, cy-1, cy+1, cy-1, cy+1};
		for (int j = 0; j < 5; ++j) {
			// skip out of map
			if ( !map.inMap(nx[j], ny[j]) ) continue;

			// skip exist neighbor patch & discontinous
			const vector<int> &cell = map.getCell(nx[j], ny[j]);
			const int pthNum = (int) cell.size();
			bool skip = false;
			for (int k = 0; k < pthNum; k++) {
				const Patch &pth = mvs->getPatch(cell[k]);
				if ( Patch::isNeighbor(*this, pth) || pth.getCorrelation() > 0.9) {
					skip = true;
					break;
				}
			}
			if (skip) continue;


		}
	}
}

/* main functions */

bool Patch::removeInvisibleCamera() {
	const int camNum = getCameraNumber();

	// sum correlation and find max correlation index
	vector<double> corrSum(camNum);
	double maxCorr = -DBL_MAX;
	int maxIdx;
	for (int i = 0; i < camNum; ++i) {
		corrSum[i] = 0;
		for (int j = 0; j < camNum; ++j) {
			corrSum[i] += corrTable.at<double>(i, j);
		}

		if (corrSum[i] > maxCorr) {
			maxIdx = i;
			maxCorr = corrSum[i];
		}
	}

	// remove invisible camera
	vector<int> removeIdx;
	for (int i = 0; i < camNum; ++i) {
		if (i == maxIdx) continue;
		if (corrTable.at<double>(maxIdx, i) < CORRELATION_THRESHOLD) {
			removeIdx.push_back(camIdx[i]);
		}
	}
	vector<int>::iterator it;
	for (int i = 0; i < (int) removeIdx.size(); i++) {
		it = find(camIdx.begin(), camIdx.end(), removeIdx[i]);
		camIdx.erase(it);
	}

	return true;
}

bool Patch::getHomographyPatch(const Vec2d &pt, const Camera &cam, const Mat_<double> &H, Mat_<double> &hp) const {
	const int patchRadius = mvs->getPatchRadius();
	const int patchSize   = mvs->getPatchSize();
	const Mat_<uchar> &img = cam.getPyramidImage()[LOD];
	hp = Mat_<double>(patchSize*patchSize, 1);

	double w, ix, iy;                // position on target image
	int px[4];                       // neighbor x
	int py[4];                       // neighbor y
	int count = 0;
	double sum = 0;
	for (double x = pt[0]-patchRadius; x <= pt[0]+patchRadius; ++x) {
		for (double y = pt[1]-patchRadius; y <= pt[1]+patchRadius; ++y) {
			// homography projection (with LOD transform)
			w  = ( H.at<double>(2, 0) * x + H.at<double>(2, 1) * y + H.at<double>(2, 2) ) * (1<<LOD);
			ix = ( H.at<double>(0, 0) * x + H.at<double>(0, 1) * y + H.at<double>(0, 2) ) / w;
			iy = ( H.at<double>(1, 0) * x + H.at<double>(1, 1) * y + H.at<double>(1, 2) ) / w;

			// interpolation neighbor points
			px[0] = (int) ix;
			py[0] = (int) iy;
			px[1] = px[0] + 1;
			py[1] = py[0];
			px[2] = px[0];
			py[2] = py[0] + 1;
			px[3] = px[0] + 1;
			py[3] = py[0] + 1;
				
			hp.at<double>(count, 0) = (double) img.at<uchar>(py[0], px[0])*(px[1]-ix)*(py[2]-iy) + 
					                  (double) img.at<uchar>(py[1], px[1])*(ix-px[0])*(py[2]-iy) + 
					                  (double) img.at<uchar>(py[2], px[2])*(px[1]-ix)*(iy-py[0]) + 
					                  (double) img.at<uchar>(py[3], px[3])*(ix-px[0])*(iy-py[0]);

			sum += hp.at<double>(count, 0)*hp.at<double>(count, 0);
			++count;
		}
	}

	hp /= sqrt(sum);
	return true;
}

/* for debug used */

void Patch::showRefinedResult() const {
	const vector<Camera> &cameras = mvs->getCameras();
	const int patchRadius = mvs->getPatchRadius();

	// camera parameters
	const int camNum        = getCameraNumber();
	const Camera &refCam    = getReferenceCamera();
	const Mat_<double> &KRF = refCam.getKR();
	const Mat_<double> &KTF = refCam.getKT();
	
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
			line(img, Point(0, cvRound(yStart)), Point( img.cols, cvRound(yEnd)), Scalar(0,255,0, 0.5));
		}

		// draw patch
		line(img, Point(ix[0],iy[0]), Point(ix[1],iy[1]), Scalar(0,0,255));
		line(img, Point(ix[0],iy[0]), Point(ix[2],iy[2]), Scalar(0,0,255));
		line(img, Point(ix[3],iy[3]), Point(ix[1],iy[1]), Scalar(0,0,255));
		line(img, Point(ix[3],iy[3]), Point(ix[2],iy[2]), Scalar(0,0,255));
		circle(img, Point(ix[4], iy[4]), 1, Scalar(0,0,255));

		sprintf(title, "img%d_%d.png", id, camIdx[i]);
		imshow(title, img);
		imwrite(title, img);
	}
	waitKey();
	destroyAllWindows();
}

void Patch::showError() const {
	const vector<Camera> &cameras = mvs->getCameras();
	const int patchRadius = mvs->getPatchRadius();
	const int patchSize   = mvs->getPatchSize();

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
	char title[30];
	sprintf(title, "error%d.png", id);
	imwrite(title, error);
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

bool Patch::setCorrelationTable() {
	const vector<Camera> &cameras = mvs->getCameras();

	// camera parameters
	const int camNum        = getCameraNumber();
	const Camera &refCam    = getReferenceCamera();
	const Mat_<double> &KRF = refCam.getKR();
	const Mat_<double> &KTF = refCam.getKT();

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

	// 2D image point on reference image
	Vec2d pt;
	refCam.project(center, pt);

	// get normalized homography patch column vector
	vector<Mat_<double> > HP(camNum);
	#pragma omp parallel for
	for (int i = 0; i < camNum; i++) {
		getHomographyPatch(pt, cameras[camIdx[i]], H[i], HP[i]);
	}

	// correlation table
	corrTable = Mat_<double>(camNum, camNum);
	for (int i = 0; i < camNum; ++i) {
		corrTable.at<double>(i, i) = 0;
		for (int j = i+1; j < camNum; ++j) {
			Mat_<double> corr = HP[i].t() * HP[j];
			corrTable.at<double>(i, j) = corr.at<double>(0, 0);
			corrTable.at<double>(j, i) = corrTable.at<double>(i, j);
		}
	}
	
	return true;
}

bool Patch::setPriority() {
	const int totalCamNum = (int) mvs->getCameras().size();
	const int camNum = getCameraNumber();

	correlation = 0;
	for (int i = 0; i < corrTable.rows; ++i) {
		for (int j = 0; j < corrTable.cols; ++j) {
			correlation += corrTable.at<double>(i,j);
		}
	}
	correlation /= (camNum*camNum);

	double camRatio = 1.0 - ((double) camNum) / ((double) totalCamNum);

	priority = fitness * exp(-correlation) * camRatio;
	return true;
}

bool Patch::setImagePoint() {
	const int camNum              = getCameraNumber();
	const vector<Camera> &cameras = mvs->getCameras();
	const Camera &refCam          = cameras[refCamIdx];
	const Mat_<Vec3b> &img        = refCam.getPyramidImage(0);

	// set image points
	imgPoint.resize(camNum);
	for (int i = 0; i < camNum; ++i) {
		const Camera &cam = cameras[camIdx[i]];
		cam.project(center, imgPoint[i]);
	}

	// set point color
	Vec2d pt;
	refCam.project(center, pt);
	center = img.at<Vec3b>(cvRound(pt[1]), cvRound(pt[0]));

	return true;
}

/* fitness function */

double PAIS::getFitness(const Particle &p, void *obj) {
	// current patch
	const Patch  &patch   = *((Patch *)obj);
	// visible camera indices
	const vector<int> &camIdx = patch.getCameraIndices();
	// static instances
	const MVS  &mvs               = patch.getMVS();
	const int patchRadius         = mvs.getPatchRadius();
	const vector<Camera> &cameras = mvs.getCameras();

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
		const Camera &cam = cameras[camIdx[i]];

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
	double mean, avgSad;             // pixel-wised mean, average sad
	double w, ix, iy;                // position on target image
	int px[4];                       // neighbor x
	int py[4];                       // neighbor y
	double *c = new double [camNum]; // bilinear color
	double fitness = 0;              // result of normalized fitness
	// distance weighting
	const Mat_<double> &distWeight = mvs.getPatchDistanceWeighting();
	Mat_<double>::const_iterator it = distWeight.begin();
	// sum of weighting
	double sumWeight = 0;
	double weight;

	for (double x = pt[0]-patchRadius; x <= pt[0]+patchRadius; ++x) {
		for (double y = pt[1]-patchRadius; y <= pt[1]+patchRadius; ++y) {
			// clear
			mean   = 0;
			avgSad = 0;

			for (int i = 0; i < camNum; i++) {
				const Mat_<uchar> &img = cameras[camIdx[i]].getPyramidImage(LOD);

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
					   (double) img.at<uchar>(py[1], px[1])*(ix-px[0])*(py[2]-iy) + 
					   (double) img.at<uchar>(py[2], px[2])*(px[1]-ix)*(iy-py[0]) + 
					   (double) img.at<uchar>(py[3], px[3])*(ix-px[0])*(iy-py[0]);
				
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