#include "patch.h"

using namespace PAIS;

/* static functions */
bool Patch::isNeighbor(const Patch &pth1, const Patch &pth2) {
	const MVS &mvs = mvs.getInstance();

	const Vec3d &c1 = pth1.getCenter();
	const Vec3d &c2 = pth2.getCenter();
	const Vec3d &n1 = pth1.getNormal();
	const Vec3d &n2 = pth2.getNormal();

	double dist = 0;
	dist += abs((c1-c2).ddot(n1));
	dist += abs((c1-c2).ddot(n2));

	if (dist <= mvs.neighborRadius) {
		return true;
	} else { 
		return false;
	}
}

/* constructor */
Patch::Patch(const Vec3d &center, const Vec3b &color, const vector<int> &camIdx, const vector<Vec2d> &imgPoint, const int id) : AbstractPatch(id) {
	this->type     = TYPE_SEED;
	this->center   = center;
    this->color    = color;
    this->camIdx   = camIdx;
    this->imgPoint = imgPoint;
	this->drop     = false;
	setEstimatedNormal();
}

Patch::Patch(const Vec3d &center, const Patch &parent, const int id) : AbstractPatch(id) {
	this->type      = TYPE_EXPAND;
    this->center    = center;
	this->camIdx    = parent.getCameraIndices();
	this->drop      = false;
	setNormal(parent.getNormal());
	expandVisibleCamera();
}

Patch::Patch(const Vec3d &center, const Vec2d &normalS, const vector<int> &camIdx, const double fitness, const double correlation, const int id) : AbstractPatch(id) {
	this->type        = TYPE_SEED;
	this->center      = center;
	this->camIdx      = camIdx;
	this->fitness     = fitness;
	this->correlation = correlation;
	this->drop        = false;
	setNormal(normalS);
	setReferenceCameraIndex();
	setDepthAndRay();
	setDepthRange();
	setLOD();
	setPriority();
	setImagePoint();
}

// added by Chaody, 2012.Sep.04
Patch::Patch(const Vec3d &center, const Vec2d &normalS, const vector<int> &camIdx, const double fitness, const double correlation, const Vec3b &color, const int id) : AbstractPatch(id) {
	this->type        = TYPE_SEED;
	this->center      = center;
	this->color    = color;
	this->camIdx      = camIdx;
	this->fitness     = fitness;
	this->correlation = correlation;
	this->drop        = false;
	setNormal(normalS);
	setReferenceCameraIndex();
	setDepthAndRay();
	setDepthRange();
	setLOD();
	setPriority();
	//setImagePoint();
}

Patch::~Patch(void) {

}

/* public functions */

void Patch::reCentering() {
	const MVS &mvs = MVS::getInstance();
	const int camNum = getCameraNumber();
	
	Mat_<double> A = Mat_<double>::zeros(3, 3);
	Mat_<double> b = Mat_<double>::zeros(3, 1);
	for (int i = 0 ; i < camNum; ++i) {
		const Vec2d  &pt        = imgPoint[i];
		const Camera &cam       = mvs.getCamera(camIdx[i]);
		const Vec3d  &camCenter = cam.getCenter();
		const Vec2d  &principle = cam.getPrinciplePoint();
		const Vec2d  &focal     = cam.getFocalLength();

		// get pixel position in world coordinate
		Mat p3d(3, 1, CV_64FC1);
		p3d.at<double>(0, 0) = (pt[0] - principle[0]) / focal[0];
		p3d.at<double>(1, 0) = (pt[1] - principle[1]) / focal[1];
		p3d.at<double>(2, 0) = 1.0;
		p3d = cam.getRotation().t() * (p3d - cam.getTranslation());

		Vec3d n(p3d.at<double>(0, 0) - camCenter[0],
			    p3d.at<double>(1, 0) - camCenter[1],
				p3d.at<double>(2, 0) - camCenter[2]);
		n = n * (1.0 / norm(n));

		A.at<double>(0, 0) += 1 - n[0]*n[0];
		A.at<double>(0, 1) += -n[0]*n[1];
		A.at<double>(0, 2) += -n[0]*n[2];
		A.at<double>(1, 0) += -n[0]*n[1];
		A.at<double>(1, 1) += 1 - n[1]*n[1];
		A.at<double>(1, 2) += -n[1]*n[2];
		A.at<double>(2, 0) += -n[0]*n[2];
		A.at<double>(2, 1) += -n[1]*n[2];
		A.at<double>(2, 2) += 1 - n[2]*n[2];
		b.at<double>(0, 0) += (1-n[0]*n[0])*camCenter[0] - n[0]*n[1]*camCenter[1] - n[0]*n[2]*camCenter[2];
		b.at<double>(1, 0) += -n[0]*n[1]*camCenter[0] + (1-n[1]*n[1])*camCenter[1] - n[1]*n[2]*camCenter[2];
		b.at<double>(2, 0) += -n[0]*n[2]*camCenter[0] - n[1]*n[2]*camCenter[1] + (1-n[2]*n[2])*camCenter[2];
	}

	Mat_<double> x = A.inv(DECOMP_SVD)*b;
	center[0] = x.at<double>(0);
	center[1] = x.at<double>(1);
	center[2] = x.at<double>(2);

	setEstimatedNormal();
}

void Patch::refine() {
	const MVS &mvs = MVS::getInstance();

	// skip few cameras
	if (getCameraNumber() < mvs.minCamNum) {
		fitness  = DBL_MAX;
		priority = DBL_MAX;
		drop = true;
		return;
	}

	setReferenceCameraIndex();
	setDepthAndRay();
	setDepthRange();
	setLOD();

	if (drop) return;

	int beforeRefCamIdx = refCamIdx;
	int afterRefCamIdx  = -1;
	int beforeCamNum    = getCameraNumber();
	int afterCamNum     = -1;
	int count           = 0; // optimization counter
	int totalCamNum     = beforeCamNum;

	// re-optimization when reference camera index or visible cameras are changed
	while ( (beforeRefCamIdx != afterRefCamIdx || beforeCamNum != afterCamNum) && count++ <= totalCamNum ) {

		if (getCameraNumber() < mvs.minCamNum) {
			fitness  = DBL_MAX;
			priority = DBL_MAX;
			drop = true;
			return;
		}

		beforeRefCamIdx = refCamIdx;
		beforeCamNum    = getCameraNumber();

		// do pso optimization (update center and normal)
		psoOptimization();

		// skip fail optimization
		if (fitness > mvs.maxFitness) {
			drop = true;
			return;
		}

		// update information
		removeInvisibleCamera();
		setReferenceCameraIndex();
		setDepthAndRay();
		setDepthRange();
		setLOD();

		if (type == TYPE_EXPAND) break;

		afterRefCamIdx = refCamIdx;
		afterCamNum    = getCameraNumber();
	}

	setPriority();
	setImagePoint();
}

/* process */

void Patch::psoOptimization() {
	const MVS &mvs = MVS::getInstance();

	// PSO parameter range (theta, phi, depth)
    double rangeL [] = {0.0 , normalS[1] - M_PI/2.0, depthRange[0]};
    double rangeU [] = {M_PI, normalS[1] + M_PI/2.0, depthRange[1]};

    // initial guess particle
    double init   [] = {normalS[0], normalS[1], depth};

	PsoSolver *solver = NULL;
	if (type == TYPE_SEED) {
		solver = new PsoSolver(3, rangeL, rangeU, PAIS::getFitness, this, mvs.maxIteration*2, mvs.particleNum*2 );
	} else {
		// reduce normal search range for expansion patch
		rangeL[0] = max(  0.0, normalS[0] - M_PI/mvs.reduceNormalRange);
		rangeU[0] = min( M_PI, normalS[0] + M_PI/mvs.reduceNormalRange);
		rangeL[1] = normalS[1] - M_PI/mvs.reduceNormalRange;
		rangeU[1] = normalS[1] + M_PI/mvs.reduceNormalRange;
		solver = new PsoSolver(3, rangeL, rangeU, PAIS::getFitness, this, mvs.maxIteration, mvs.particleNum);
	}

	clock_t start_t, end_t;
	start_t = clock();
	solver->setParticle(init);
    solver->run(true);
	end_t = clock();

	// set refined patch information
	fitness = solver->getGbestFitness();
    const double *gBest = solver->getGbest();
    setNormal(Vec2d(gBest[0], gBest[1]));
    depth  = gBest[2];
	center = ray * depth + mvs.getCamera(refCamIdx).getCenter();

	if (type != TYPE_SEED)
		LogManager::log("patch it\t%d\tsec\t%f", solver->getIteration(), (double)(end_t - start_t) / CLOCKS_PER_SEC);

	delete solver;
}

void Patch::setCorrelationTable(const vector<Mat_<double>> &H) {
	const MVS &mvs = MVS::getInstance();
	const vector<Camera> &cameras = mvs.cameras;

	// camera parameters
	const int camNum        = getCameraNumber();
	const Camera &refCam    = mvs.getCamera(refCamIdx);

	corrTable = Mat_<double>::zeros(camNum, camNum);

	// 2D image point on reference image
	Vec2d pt;
	refCam.project(center, pt, LOD);

	// get normalized homography patch column vector
	vector<Mat_<double> > HP(camNum);
	#pragma omp parallel for
	for (int i = 0; i < camNum; i++) {
		const Mat_<uchar> &img = cameras[camIdx[i]].getPyramidImage(LOD);
		getHomographyPatch(pt, img, H[i], HP[i]);
	}

	// drop patch if out of boundary
	if (drop) {
		correlation = 0;
		return;
	}

	// correlation table
	for (int i = 0; i < camNum; ++i) {
		corrTable.at<double>(i, i) = 0;
		for (int j = i+1; j < camNum; ++j) {
			Mat_<double> corr = HP[i].t() * HP[j];
			corrTable.at<double>(i, j) = corr.at<double>(0, 0);
			corrTable.at<double>(j, i) = corrTable.at<double>(i, j);
		}
	}

	// set average correlation
	correlation = 0;
	for (int i = 0; i < corrTable.rows; ++i) {
		for (int j = 0; j < corrTable.cols; ++j) {
			correlation += corrTable.at<double>(i,j);
		}
	}
	correlation /= (camNum*camNum-camNum);
}

double Patch::getHomographyRegionRatio(const Vec2d &pt, const Mat_<double> &H) const {
	const int patchRadius = MVS::getInstance().patchRadius;

	// 0 3
	// 1 2
	double x [] = {pt[0]-patchRadius, pt[0]-patchRadius, pt[0]+patchRadius, pt[0]+patchRadius, pt[0]-patchRadius, pt[0]            , pt[0]+patchRadius, pt[0]            };
	double y [] = {pt[1]-patchRadius, pt[1]+patchRadius, pt[1]+patchRadius, pt[1]-patchRadius, pt[1]            , pt[1]+patchRadius, pt[1]            , pt[1]-patchRadius};
	vector<Point2f> p(8);
	double w;
	for (int i = 0; i < 8; ++i) {
		w       =   H.at<double>(2, 0) * x[i] + H.at<double>(2, 1) * y[i] + H.at<double>(2, 2);
		p[i].x = ( H.at<double>(0, 0) * x[i] + H.at<double>(0, 1) * y[i] + H.at<double>(0, 2) ) / w;
		p[i].y = ( H.at<double>(1, 0) * x[i] + H.at<double>(1, 1) * y[i] + H.at<double>(1, 2) ) / w;
	}

	// fit ellipse
	RotatedRect box = fitEllipse(p);

	return double( min(box.size.width, box.size.height) / max(box.size.width, box.size.height) );
}

void Patch::getHomographies(const Vec3d &center, const Vec3d &normal, vector<Mat_<double>> &H) const {
	const MVS &mvs = MVS::getInstance();
	const vector<Camera> &cameras = mvs.getCameras();

	// camera parameters
	const Camera &refCam  = mvs.getCamera(refCamIdx);
	const Mat_<double> &KRF = refCam.getKR();
	const Mat_<double> &KTF = refCam.getKT();

	// plane equation (distance form plane to origin)
	const double d = -center.ddot(normal);          
	const Mat_<double> normalM(normal);

	// set container
	const int camNum = getCameraNumber();
	H.resize(camNum);

	// LOD scalar for camera intrisic matrix K
	Mat_<double> LODM = Mat_<double>::zeros(3, 3);
	LODM.at<double>(0, 0) = pow(mvs.lodRatio, LOD);
	LODM.at<double>(1, 1) = pow(mvs.lodRatio, LOD);
	LODM.at<double>(2, 2) = 1.0;

	// get homography from reference to target image
	Mat_<double> invH = ( d*LODM*KRF - LODM*KTF*normalM.t() ).inv();
	for (int i = 0; i < camNum; i++) {
		// indentity for reference camera
		if (camIdx[i] == refCamIdx) {
			H[i] = Mat_<double>::eye(3, 3);
			continue;
		}

		// visible camera
		const Camera &cam = cameras[camIdx[i]];

		// get homography matrix
		const Mat_<double> &KRT = cam.getKR();        // K*R of to camera
		const Mat_<double> &KTT = cam.getKT();        // K*T of to camera
		H[i] = ( d*LODM*KRT - LODM*KTT*normalM.t() ) * invH;
	}
}

void Patch::getHomographyPatch(const Vec2d &pt, const Mat_<uchar> &img, const Mat_<double> &H, Mat_<double> &hp) {

	if (this->drop) return;

	const MVS &mvs = MVS::getInstance();
	const int patchRadius = mvs.patchRadius;
	const int patchSize   = mvs.patchSize;

	hp = Mat_<double>(patchSize*patchSize, 1);

	double w, ix, iy;                // position on target image
	int px[4];                       // neighbor x
	int py[4];                       // neighbor y
	int count = 0;
	double sum = 0;
	for (double x = pt[0]-patchRadius; x <= pt[0]+patchRadius; ++x) {
		for (double y = pt[1]-patchRadius; y <= pt[1]+patchRadius; ++y) {
			// homography projection (with LOD transform)
			w  = ( H.at<double>(2, 0) * x + H.at<double>(2, 1) * y + H.at<double>(2, 2) );
			ix = ( H.at<double>(0, 0) * x + H.at<double>(0, 1) * y + H.at<double>(0, 2) ) / w;
			iy = ( H.at<double>(1, 0) * x + H.at<double>(1, 1) * y + H.at<double>(1, 2) ) / w;

			// skip overflow cases
			if (ix < 0 || ix >= img.cols-1 || iy < 0 || iy >= img.rows-1 || w == 0 || this->drop) {
				// printf("ID %d \t corr overflow x:%f \t y:%f LOD:%d fit:%f\n", getId(), ix, iy, LOD, fitness);
				#pragma omp critical 
				{
					this->drop = true;
				}
				return;
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

			hp.at<double>(count, 0) = (double) img.at<uchar>(py[0], px[0])*(px[1]-ix)*(py[2]-iy) + 
					                  (double) img.at<uchar>(py[1], px[1])*(ix-px[0])*(py[2]-iy) + 
					                  (double) img.at<uchar>(py[2], px[2])*(px[1]-ix)*(iy-py[0]) + 
					                  (double) img.at<uchar>(py[3], px[3])*(ix-px[0])*(iy-py[0]);

			sum += hp.at<double>(count, 0)*hp.at<double>(count, 0);
			++count;
		}
	}

	hp /= sqrt(sum);
	return;
}

/* setters */

void Patch::setEstimatedNormal() {
	if (drop) return;

	const MVS &mvs = MVS::getInstance();
    const int camNum = getCameraNumber();

	if (camNum < mvs.minCamNum) {
		drop = true;
		printf("setEstimatedNormal: Not enough visible camera\n");
		return;
	}

    Vec3d dir;
    normal = Vec3d(0.0, 0.0, 0.0);
    for (int i = 0; i < camNum; i++) {
        const Camera &cam = mvs.getCamera(camIdx[i]);
        dir = cam.getCenter() - center;
        dir *= (1.0 / norm(dir));
        normal += dir;
    }
    normal *= (1.0 / norm(normal));
        
    setNormal(normal);
}

void Patch::setReferenceCameraIndex() {
	if (drop) return;

	const MVS &mvs = MVS::getInstance();
    const int camNum = getCameraNumber();

	if (camNum < mvs.minCamNum) {
		drop = true;
		printf("setReferenceCameraIndex: Not enough visible camera\n");
		return;
	}

    refCamIdx = -1;
    double maxCorr = -DBL_MAX;
    double corr;
    for (int i = 0; i < camNum; i++) {
        const Camera &cam = mvs.getCamera(camIdx[i]);
        corr = normal.ddot(-cam.getOpticalNormal());

        if (corr > maxCorr) {
            maxCorr = corr;
            refCamIdx = camIdx[i];
        }
    }

	if (refCamIdx < 0) {
		printf("can't set reference camera camNum: %d maxCorr: %f\n", camNum, maxCorr);
		refCamIdx = camIdx[0];
		drop = true;
	}
}

void Patch::setDepthAndRay() {
	if (drop) return;

	const MVS &mvs = MVS::getInstance();

	if (refCamIdx < 0) {
		drop = true;
		printf("setDepthAndRay: Not set reference camera\n");
		return;
	}

    ray = center - mvs.getCamera(refCamIdx).getCenter();
    depth = norm(ray);
    ray = ray * (1.0 / depth);
}

void Patch::setDepthRange() {
	if (drop) return;

	const MVS &mvs   = MVS::getInstance();
    const int camNum = getCameraNumber();

	if (camNum < mvs.minCamNum) {
		drop = true;
		printf("setDepthRange: Not enough visible camera\n");
		return;
	}

    const Camera &refCam = mvs.getCamera(refCamIdx);

    // center shift
    Vec3d c2 = ray * (depth+1.0) + refCam.getCenter();
    // projected point
    Vec2d p1, p2;

    double worldDist, imgDist;
    double maxWorldDist = -DBL_MAX;
    for (int i = 0; i < camNum; i++) {
        if (camIdx[i] == refCamIdx) continue;
                
        const Camera &cam = mvs.getCamera(camIdx[i]);

        cam.project(center, p1);
		cam.project(c2, p2);

		imgDist   = norm(p1-p2);
        worldDist = 1.0 / imgDist;

		// fix for small baseline
        if (worldDist > maxWorldDist && imgDist >= 0.01) {
            maxWorldDist = worldDist;
        }
    }

	// skip small baseline view
	if (maxWorldDist == -DBL_MAX) {
		drop = true;
		return;
	}

	depthRange[0] = max(depth - maxWorldDist*mvs.depthRangeScalar, 0.0);
	depthRange[1] = depth + min(maxWorldDist*mvs.depthRangeScalar, mvs.neighborRadius * 100);
}

void Patch::setLOD() {
	if (drop) return;

	if (refCamIdx < 0) {
		drop = true;
		printf("setLOD: Not set reference camera\n");
        return;
    }

	const MVS &mvs = MVS::getInstance();

    // patch size
	int patchRadius = mvs.patchRadius;
    int size        = mvs.patchSize;

    // reference camera
	const Camera &refCam = mvs.getCamera(refCamIdx);
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
	LOD = mvs.minLOD-1;

    // find LOD
	while (variance < mvs.textureVariation) {
        // goto next LOD
        LOD++;

        // return if reach the max LOD
        if (LOD >= refCam.getMaxLOD()) {
			LOD = refCam.getMaxLOD();
            delete [] textures;
            return;
        }

        // LOD-- if out of image bound
        if ( !refCam.project(center, pt, LOD) ) {
            //printf("setLOD image point out of image bound: LOD %d, x: %f, y: %f\n", LOD, pt[0], pt[1]);
            LOD = max(LOD-1, 0);
            delete [] textures;
            return;
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
                    return;
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
			Mat img = refCam.getPyramidEdge(l).clone();
            circle(img, Point(cvRound(pt[0]), cvRound(pt[1])), max(patchRadius, 5), Scalar(255,0,0), 1, CV_AA);
            imshow(title, img);
            cvMoveWindow(title, 0, 0);
        }
        waitKey();
        destroyAllWindows();
    }
	*/
    delete [] textures;
}

void Patch::setPriority() {
	if (drop) return;
	// cross correlation weighting
	double w1 = 1.0;
	// visible camera ratio weighting
	double w2 = 1.0;

	const MVS &mvs = MVS::getInstance();
	const int totalCamNum = (int) mvs.getCameras().size();
	const int camNum = getCameraNumber();
	double camRatio = ((double) camNum) / ((double) totalCamNum);
	priority = fitness * exp( -correlation/w1 - camRatio/w2 ) * (LOD+1.0);
	//priority = fitness * (1.0/correlation) * (1.0/camRatio) * (LOD+1.0);
}

void Patch::setImagePoint() {
	if (drop) return;
	const MVS &mvs                = MVS::getInstance();
	const int camNum              = getCameraNumber();

	if (camNum == 0) {
		printf("no visible camera\n");
		return;
	}

	const vector<Camera> &cameras = mvs.getCameras();
	const Camera &refCam          = cameras[refCamIdx];
	const Mat_<Vec3b> &img        = refCam.getRgbImage();

	// set image points
	imgPoint.resize(camNum);
	for (int i = 0; i < camNum; ++i) {
		const Camera &cam = cameras[camIdx[i]];
		cam.project(center, imgPoint[i]);
	}

	// set point color
	Vec2d pt;
	if ( refCam.project(center, pt) ) {
		color = img.at<Vec3b>(cvRound(pt[1]), cvRound(pt[0]));
	}
}

void Patch::removeInvisibleCamera() {
	if (drop) return;

	const MVS &mvs = MVS::getInstance();
	const int camNum = getCameraNumber();
	const Camera &refCam = mvs.getCamera(refCamIdx);

	vector<Mat_<double> > H;
	getHomographies(center, normal, H);
	setCorrelationTable(H);

	// sum correlation and find max correlation index
	double corrSum;
	double maxCorr = -DBL_MAX;
	int maxIdx = 0;
	for (int i = 0; i < camNum; ++i) {
		corrSum = 0;
		for (int j = 0; j < camNum; ++j) {
			corrSum += corrTable.at<double>(i, j);
		}

		if (corrSum >= maxCorr) {
			maxIdx = i;
			maxCorr = corrSum;
		}
	}

	Vec2d pt;
	refCam.project(center, pt, LOD);

	// remove invisible camera
	vector<int> removeIdx;
	// mark idx
	for (int i = 0; i < camNum; ++i) {
		// filter by region ratio
		if (getHomographyRegionRatio(pt, H[i]) < mvs.minRegionRatio) {
			removeIdx.push_back(camIdx[i]);
			continue;
		}

		// filter by normal correlation
		if (normal.ddot(-mvs.getCamera(camIdx[i]).getOpticalNormal()) < 0) {
			removeIdx.push_back(camIdx[i]);
			continue;
		}

		// filter by correlation
		if (i == maxIdx) continue;
		if (corrTable.at<double>(maxIdx, i) < mvs.minCorrelation) {
			removeIdx.push_back(camIdx[i]);
			continue;
		}
	}

	// remove camera idx
	vector<int>::iterator it;
	for (int i = 0; i < (int) removeIdx.size(); i++) {
		it = find(camIdx.begin(), camIdx.end(), removeIdx[i]);
		if(it != camIdx.end()) {
			camIdx.erase(it);
		}
	}

	if (getCameraNumber() < mvs.minCamNum) {
		drop = true;
	}
}

void Patch::expandVisibleCamera() {
	if (drop) return;

	const MVS &mvs = MVS::getInstance();
	const vector<Camera> &cameras = mvs.cameras;

	vector<int> expCamIdx;

	// expand visible camera through a viewing cone
	for (int i = 0; i < cameras.size(); ++i) {
		const Camera &cam = cameras[i];
		if (normal.ddot(-cam.getOpticalNormal()) >= mvs.visibleCorrelation) {
			expCamIdx.push_back(i);
		}
	}

	// use parent visible camera when not enough visible cameras
	if (expCamIdx.size() < mvs.minCamNum) {

		for (int i = 0; i < camIdx.size(); ++i) {
			const Camera &cam = cameras[camIdx[i]];
			// get parent patch camera through a larger viewing cone
			if (normal.ddot(-cam.getOpticalNormal()) >= mvs.visibleCorrelation/2.0) {
				expCamIdx.push_back(camIdx[i]);
			}
		}
		
		// unique camera indices
		sort(expCamIdx.begin(), expCamIdx.end());
		vector<int>::iterator it = unique(expCamIdx.begin(), expCamIdx.end());
		expCamIdx.resize(it - expCamIdx.begin());
	}

	camIdx = expCamIdx;

	if (getCameraNumber() < mvs.minCamNum) {
		drop = true;
	}
}

/* misc */
void Patch::showRefinedResult() const {
	if (refCamIdx < 0) {
		printf("showRefinedResult: reference camera not set\n");
		return;
	}

	const MVS &mvs = MVS::getInstance();
	const vector<Camera> &cameras = mvs.getCameras();
	const int patchRadius = mvs.getPatchRadius();

	// camera parameters
	const int camNum        = getCameraNumber();
	const Camera &refCam    = mvs.getCamera(refCamIdx);
	
	// Homographies to visible camera
	vector<Mat_<double> > H(camNum);
	getHomographies(center, normal, H);

	Vec2d pt;
	refCam.project(center, pt, LOD);
	
	double x[5] = {pt[0]-patchRadius, pt[0]-patchRadius, pt[0]+patchRadius, pt[0]+patchRadius, pt[0]};
	double y[5] = {pt[1]-patchRadius, pt[1]+patchRadius, pt[1]-patchRadius, pt[1]+patchRadius, pt[1]};

	double w, ix[5], iy[5];
	char title[128];
	for (int i = 0; i < camNum; i++) {
		const Camera &cam = mvs.getCameras()[camIdx[i]];
		Mat_<Vec3b> img = cam.getRgbImage().clone();
		resize(img, img, Size(img.cols*pow(mvs.lodRatio, LOD), img.rows*pow(mvs.lodRatio, LOD) ) );

		// homography projection
		for (int c = 0; c < 5; c++) {
			w     =   H[i].at<double>(2, 0) * x[c] + H[i].at<double>(2, 1) * y[c] + H[i].at<double>(2, 2);
			ix[c] = ( H[i].at<double>(0, 0) * x[c] + H[i].at<double>(0, 1) * y[c] + H[i].at<double>(0, 2) ) / w;
			iy[c] = ( H[i].at<double>(1, 0) * x[c] + H[i].at<double>(1, 1) * y[c] + H[i].at<double>(1, 2) ) / w;
			ix[c] = cvRound(ix[c]);
			iy[c] = cvRound(iy[c]);
		}

		// draw patch
		line(img, Point(ix[0],iy[0]), Point(ix[1],iy[1]), Scalar(0,0,255));
		line(img, Point(ix[0],iy[0]), Point(ix[2],iy[2]), Scalar(0,0,255));
		line(img, Point(ix[3],iy[3]), Point(ix[1],iy[1]), Scalar(0,0,255));
		line(img, Point(ix[3],iy[3]), Point(ix[2],iy[2]), Scalar(0,0,255));
		circle(img, Point(ix[4], iy[4]), 1, Scalar(0,0,255));

		double regionRatio = getHomographyRegionRatio(pt, H[i]);
		
		sprintf(title, "id: %d cam: %d ratio: %f", getId(), camIdx[i], regionRatio);
		if (camIdx[i] == refCamIdx) {
			sprintf(title, "reference id: %d cam: %d ratio: %f", getId(), camIdx[i], regionRatio);
		}
		imshow(title, img);
		cvMoveWindow(title, 0, 0);
	}
}

void Patch::showError() const {
	if (refCamIdx < 0) {
		printf("showError: reference camera not set\n");
		return;
	}

	const MVS &mvs = MVS::getInstance();
	const vector<Camera> &cameras = mvs.getCameras();
	const int patchRadius = mvs.getPatchRadius();
	const int patchSize   = mvs.getPatchSize();

	// camera parameters
	const Camera &refCam  = mvs.getCamera(refCamIdx);
	const int camNum = getCameraNumber();

	// Homographies to visible camera
	vector<Mat_<double> > H(camNum);
	getHomographies(center, normal, H);

	Vec2d pt;
	refCam.project(center, pt, LOD);

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
				const Mat_<uchar> &img = cameras[camIdx[i]].getPyramidImage(LOD);

				// homography projection
				w  =   H[i].at<double>(2, 0) * x + H[i].at<double>(2, 1) * y + H[i].at<double>(2, 2);
				ix = ( H[i].at<double>(0, 0) * x + H[i].at<double>(0, 1) * y + H[i].at<double>(0, 2) ) / w;
				iy = ( H[i].at<double>(1, 0) * x + H[i].at<double>(1, 1) * y + H[i].at<double>(1, 2) ) / w;
				
				// interpolation neighbor points
				px[0] = (int) ix;
				py[0] = (int) iy;
				px[1] = px[0] + 1;
				py[1] = py[0];
				px[2] = px[0];
				py[2] = py[0] + 1;
				px[3] = px[0] + 1;
				py[3] = py[0] + 1;

				for (int j = 0; j < 4; ++j) {
					if ( !cameras[camIdx[i]].inImage(px[j], py[j], LOD) ) {
						return;
					}
				}

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
	error = (error - minE) / (maxE - minE);
	char title[30];
	sprintf(title, "error%d.png", getId());
	resize(error, error, Size(200, 200), 0, 0, CV_INTER_NN);
	imshow(title, error);
	imwrite(title, error*255);
	cvMoveWindow(title, 0, 0);
}

// show fitness error image (plus and minus weighted function), 2013.01.19 by Chaody
void Patch::showFitness() const {
	if (refCamIdx < 0) {
		printf("showError: reference camera not set\n");
		return;
	}

	const MVS &mvs = MVS::getInstance();
	const vector<Camera> &cameras = mvs.getCameras();
	const int patchRadius = mvs.getPatchRadius();
	const int patchSize   = mvs.getPatchSize();

	// camera parameters
	const Camera &refCam  = mvs.getCamera(refCamIdx);
	const int camNum = getCameraNumber();

	// Homographies to visible camera
	vector<Mat_<double> > H(camNum);
	getHomographies(center, normal, H);

	Vec2d pt;
	refCam.project(center, pt, LOD);

	// warping (get pixel-wised variance)
	double mean, avgSad;             // pixel-wised mean, avgSad
	double w, ix, iy;                // position on target image
	int px[4];                       // neighbor x
	int py[4];                       // neighbor y
	double *c = new double [camNum]; // bilinear color
	Mat_<double> patch_fitness_minus(patchSize, patchSize); // plus function fitness
	Mat_<double> patch_fitness_plus(patchSize, patchSize);	// minus function fitness
	double fitness_plus = 0;              // result of normalized fitness
	double fitness_minus = 0;              // result of normalized fitness
	double weight_plus, weight_minus;
	double sumWeight_plus = 0;
	double sumWeight_minus = 0;
	const double diffWeighting = mvs.getDifferenceWeight();

	for (double x = pt[0]-patchRadius, ex = 0; x <= pt[0]+patchRadius; x++, ex++) {
		for (double y = pt[1]-patchRadius, ey = 0; y <= pt[1]+patchRadius; y++, ey++) {
			// clear
			mean = 0;
			avgSad = 0;

			for (int i = 0; i < camNum; i++) {
				const Mat_<uchar> &img = cameras[camIdx[i]].getPyramidImage(LOD);

				// homography projection
				w  =   H[i].at<double>(2, 0) * x + H[i].at<double>(2, 1) * y + H[i].at<double>(2, 2);
				ix = ( H[i].at<double>(0, 0) * x + H[i].at<double>(0, 1) * y + H[i].at<double>(0, 2) ) / w;
				iy = ( H[i].at<double>(1, 0) * x + H[i].at<double>(1, 1) * y + H[i].at<double>(1, 2) ) / w;
				
				// interpolation neighbor points
				px[0] = (int) ix;
				py[0] = (int) iy;
				px[1] = px[0] + 1;
				py[1] = py[0];
				px[2] = px[0];
				py[2] = py[0] + 1;
				px[3] = px[0] + 1;
				py[3] = py[0] + 1;

				for (int j = 0; j < 4; ++j) {
					if ( !cameras[camIdx[i]].inImage(px[j], py[j], LOD) ) {
						return;
					}
				}

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

			
			weight_plus = 1;
			weight_minus = 1;

			if ( mvs.isAdaptiveDifferenceEnable() ) { // adaptive difference weighting
				//weight *= exp(avgSad*avgSad/diffWeighting);
				weight_minus *= exp(-avgSad/diffWeighting); // non-square minus
				weight_plus *= exp(avgSad/diffWeighting); // non-square plus
			}
						
			sumWeight_plus += weight_plus;
			sumWeight_minus += weight_minus;

			fitness_plus   += weight_plus * avgSad;
			fitness_minus   += weight_minus * avgSad;

			patch_fitness_plus.at<double>(ey, ex) = weight_plus * avgSad;
			patch_fitness_minus.at<double>(ey, ex) = weight_minus * avgSad;

		} // end of warping y
	} // end of warping x


	double minE ,maxE;
	fitness_plus = fitness_plus / sumWeight_plus;
	patch_fitness_plus = patch_fitness_plus -  fitness_plus;
	minMaxLoc(patch_fitness_plus, &minE, &maxE);
	patch_fitness_plus = (patch_fitness_plus - minE) / (maxE - minE);

	fitness_minus = fitness_minus / sumWeight_minus;
	patch_fitness_minus = patch_fitness_minus -  fitness_minus;
	minMaxLoc(patch_fitness_minus, &minE, &maxE);
	patch_fitness_minus = (patch_fitness_minus - minE) / (maxE - minE);
	
	char title[30];
	sprintf(title, "Plus_fitness%d.png", getId());
	resize(patch_fitness_plus, patch_fitness_plus, Size(200, 200), 0, 0, CV_INTER_NN);
	imshow(title, patch_fitness_plus);
	imwrite(title, patch_fitness_plus*255);
	cvMoveWindow(title, 0, 0);

	sprintf(title, "Minus_fitness%d.png", getId());
	resize(patch_fitness_minus, patch_fitness_minus, Size(200, 200), 0, 0, CV_INTER_NN);
	imshow(title, patch_fitness_minus);
	imwrite(title, patch_fitness_minus*255);
	cvMoveWindow(title, 0, 0);

}

// zc asked
bool Patch::centerDifferenceFiltering() const {
	//return true;

	if (refCamIdx < 0) {
		printf("showError: reference camera not set\n");
		return false;
	}

	const MVS &mvs = MVS::getInstance();
	const vector<Camera> &cameras = mvs.getCameras();
	const int patchRadius = mvs.getPatchRadius();
	const int patchSize   = mvs.getPatchSize();

	// camera parameters
	const Camera &refCam  = mvs.getCamera(refCamIdx);
	const int camNum = getCameraNumber();

	// Homographies to visible camera
	vector<Mat_<double> > H(camNum);
	getHomographies(center, normal, H);

	Vec2d pt;
	refCam.project(center, pt, LOD);

	// warping (get pixel-wised variance)
	double CenterSAD,TotalSAD = 0;
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
				const Mat_<uchar> &img = cameras[camIdx[i]].getPyramidImage(LOD);

				// homography projection
				w  =   H[i].at<double>(2, 0) * x + H[i].at<double>(2, 1) * y + H[i].at<double>(2, 2);
				ix = ( H[i].at<double>(0, 0) * x + H[i].at<double>(0, 1) * y + H[i].at<double>(0, 2) ) / w;
				iy = ( H[i].at<double>(1, 0) * x + H[i].at<double>(1, 1) * y + H[i].at<double>(1, 2) ) / w;
				
				// interpolation neighbor points
				px[0] = (int) ix;
				py[0] = (int) iy;
				px[1] = px[0] + 1;
				py[1] = py[0];
				px[2] = px[0];
				py[2] = py[0] + 1;
				px[3] = px[0] + 1;
				py[3] = py[0] + 1;

				for (int j = 0; j < 4; ++j) {
					if ( !cameras[camIdx[i]].inImage(px[j], py[j], LOD) ) {
						return false;
					}
				}

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
			if(ex == patchRadius && ey == patchRadius)
			{
				CenterSAD = avgSad;
			}
			TotalSAD += avgSad;
		} // end of warping y
	} // end of warping x
	TotalSAD /= (patchSize*1.0);
	if(CenterSAD > this->getFitness())
		//TotalSAD)
	{
		return false;
	}
	return true;
}

/* fitness function */

double PAIS::getFitness(const Particle &p, void *obj) {
	// MVS
	const MVS &mvs                = MVS::getInstance();
	const int patchRadius         = mvs.getPatchRadius();
	const int patchSize           = mvs.getPatchSize();
	const vector<Camera> &cameras = mvs.getCameras();

	// current patch
	const Patch  &patch   = *((Patch *)obj);
	// visible camera indices
	const vector<int> &camIdx = patch.getCameraIndices();
	// level of detail
	int LOD = patch.getLOD();

	// camera parameters
	const Camera &refCam        = mvs.getCamera(patch.getReferenceCameraIndex());
	const int camNum            = patch.getCameraNumber();
	const Mat_<double> &edgeImg = refCam.getPyramidEdge(LOD);
	const Mat_<uchar>  &refImg  = refCam.getPyramidImage(LOD);

	// given patch normal
	Vec3d normal;
	Utility::spherical2Normal(Vec2d(p.pos[0], p.pos[1]), normal);

	// skip inversed normal
	if (normal.ddot(refCam.getOpticalNormal()) > 0) {
        return DBL_MAX;
    }

	// given patch center
	const Vec3d center = patch.getRay() * p.pos[2] + refCam.getCenter();

	// Homographies to visible camera
	vector<Mat_<double> > H(camNum);
	patch.getHomographies(center, normal, H);

	// projected point on reference image with LOD transform
	Vec2d pt;
	if ( !refCam.project(center, pt, LOD) ) {
		return DBL_MAX;
	}

	// skip out of reference image bound patch
	if (pt[0]-patchRadius < 2 || 
		pt[0]+patchRadius >= edgeImg.cols-3 || 
		pt[1]-patchRadius < 2 || 
		pt[1]+patchRadius >= edgeImg.rows-3) {
		return DBL_MAX;
	}

	// warping (get pixel-wised variance)
	double mean, avgSad;             // pixel-wised mean, average sad
	double w, ix, iy;                // position on target image
	int px[4];                       // neighbor x
	int py[4];                       // neighbor y
	double *c = new double [camNum]; // bilinear color
	double fitness = 0;              // result of normalized fitness

	// distance & difference weighting weighting
	const double diffWeighting = mvs.getDifferenceWeight();
	const double gradientWeighting = mvs.getGradientWeight();
	Mat_<double>::const_iterator it = mvs.getPatchDistanceWeighting().begin();
	double weight;
	double sumWeight = 0;

	for (double x = pt[0]-patchRadius; x <= pt[0]+patchRadius; ++x) {
		for (double y = pt[1]-patchRadius; y <= pt[1]+patchRadius; ++y, ++it) {
			// clear
			mean   = 0;
			avgSad = 0;

			// skip background
			if (refImg.at<uchar>(cvRound(y), cvRound(x)) == 0) continue;
			// skip no gradient
			// if (edgeImg.at<double>(cvRound(y), cvRound(x)) == 0.0) continue;

			for (int i = 0; i < camNum; ++i) {
				const Mat_<uchar> &img = cameras[camIdx[i]].getPyramidImage(LOD);

				// homography projection
				w  = ( H[i].at<double>(2, 0) * x + H[i].at<double>(2, 1) * y + H[i].at<double>(2, 2) );
				ix = ( H[i].at<double>(0, 0) * x + H[i].at<double>(0, 1) * y + H[i].at<double>(0, 2) ) / w;
				iy = ( H[i].at<double>(1, 0) * x + H[i].at<double>(1, 1) * y + H[i].at<double>(1, 2) ) / w;

				// skip overflow cases
				if (ix < 2 || ix >= img.cols-3 || iy < 2 || iy >= img.rows-3 || w == 0) {
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

				mean += c[i];
			} // end of camera

			mean /= camNum;

			for (int i = 0; i < camNum; i++) {
				avgSad += abs(c[i]-mean);
			}
			avgSad /= camNum;

			weight = 1;
			if ( mvs.isAdaptiveDistanceEnable() ) {   // adaptive distance weighting
				weight *= (*it);
			}
			if ( mvs.isAdaptiveDifferenceEnable() ) { // adaptive difference weighting
				//weight *= exp(avgSad*avgSad/diffWeighting);
				weight *= exp(-avgSad/diffWeighting); // non-square
			}
			if ( mvs.isAdaptiveGradientEnable() ) {   // adaptive gradient maginitude weighting
				weight *= exp( -1.0 / (edgeImg.at<double>(cvRound(y), cvRound(x))*gradientWeighting) );
			}
			sumWeight += weight;
			fitness   += weight * avgSad;
		} // end of warping y
	} // end of warping x

	delete [] c;

	return fitness / sumWeight;
}