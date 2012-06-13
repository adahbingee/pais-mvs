#include "patch.h"

using namespace PAIS;

/* static functions */
bool Patch::isNeighbor(const Patch &pth1, const Patch &pth2) {
	const Vec3d &c1 = pth1.getCenter();
	const Vec3d &c2 = pth2.getCenter();
	const Vec3d &n1 = pth1.getNormal();
	const Vec3d &n2 = pth2.getNormal();

	double dist = 0;
	dist += abs((c1-c2).ddot(n1));
	dist += abs((c1-c2).ddot(n2));

	if (dist < 0.001) {
		return true;
	} else { 
		return false;
	}
}

/* constructor */
Patch::Patch(const Vec3d &center, const Vec3b &color, const vector<int> &camIdx, const vector<Vec2d> &imgPoint, const int id) : AbstractPatch(id) {
    this->center   = center;
    this->color    = color;
    this->camIdx   = camIdx;
    this->imgPoint = imgPoint;
	setEstimatedNormal();
}

Patch::Patch(const Vec3d &center, const Patch &parent, const int id) : AbstractPatch(id) {
    this->center    = center;
	this->camIdx    = parent.getCameraIndices();
	setNormal(parent.getNormal());
	expandVisibleCamera();
}

Patch::Patch(const Vec3d &center, const Vec2d &normalS, const vector<int> &camIdx, const double fitness, const int id) : AbstractPatch(id) {
	this->center = center;
	this->camIdx = camIdx;
	this->fitness = fitness;
	setNormal(normalS);
	setReferenceCameraIndex();
	setDepthAndRay();
	setDepthRange();
	setLOD();

	// set normalized homography patch correlation table
	setCorrelationTable();
	// set patch priority
	setPriority();
	// set image point
	setImagePoint();
}

Patch::~Patch(void) {

}

/* public functions */

void Patch::refine() {
	setReferenceCameraIndex();
	setDepthAndRay();
	setDepthRange();
	setLOD();

	const MVS &mvs = MVS::getInstance();

	// PSO parameter range (theta, phi, depth)
    double rangeL [] = {0.0 , normalS[1] - M_PI/2.0, depthRange[0]};
    double rangeU [] = {M_PI, normalS[1] + M_PI/2.0, depthRange[1]};

    // initial guess particle
    double init   [] = {normalS[0], normalS[1], depth};

	PsoSolver solver(3, rangeL, rangeU, PAIS::getFitness, this, mvs.maxIteration, mvs.particleNum);
    solver.setParticle(init);
    solver.run(true);

	fitness = solver.getGbestFitness();

	// return if not convergence
	if (fitness == DBL_MAX) return;

	// set refined patch information
    const double *gBest = solver.getGbest();
    setNormal(Vec2d(gBest[0], gBest[1]));
    depth  = gBest[2];
	center = ray * depth + mvs.cameras[refCamIdx].getCenter();

    int beforeCamNum = getCameraNumber();
	// set normalized homography patch correlation table
    setCorrelationTable();
	// remove insvisible camera
    removeInvisibleCamera();
    int afterCamNum = getCameraNumber();
	if (beforeCamNum != afterCamNum && afterCamNum >= mvs.minCamNum) {
		refine();
    }

	if (beforeCamNum == afterCamNum && afterCamNum >= mvs.minCamNum) {
		// set patch priority
		setPriority();
		// set image point
		setImagePoint();
	}

	printf("ID: %d\tLOD: %d\tit: %d\tfit: %.2f \tpri: %.2f\n", getId(), LOD, solver.getIteration(), fitness, priority);
}

/* process */

void Patch::getHomographyPatch(const Vec2d &pt, const Mat_<uchar> &img, const Mat_<double> &H, Mat_<double> &hp) const {
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
			w  = ( H.at<double>(2, 0) * x + H.at<double>(2, 1) * y + H.at<double>(2, 2) ) * (1<<LOD);
			ix = ( H.at<double>(0, 0) * x + H.at<double>(0, 1) * y + H.at<double>(0, 2) ) / w;
			iy = ( H.at<double>(1, 0) * x + H.at<double>(1, 1) * y + H.at<double>(1, 2) ) / w;

			// skip overflow cases
			if (ix < 0 || ix >= img.cols-1 || iy < 0 || iy >= img.rows-1 || w == 0) {
				hp = Mat_<double>::ones(patchSize*patchSize, 1) * DBL_MAX;
				printf("ID %d \t correlation overflow %f \t %f\n", getId(), ix, iy);
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
}

/* setters */

void Patch::setEstimatedNormal() {
	const MVS &mvs = MVS::getInstance();
    const int camNum = getCameraNumber();

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
	const MVS &mvs = MVS::getInstance();
    const int camNum = getCameraNumber();

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
}

void Patch::setDepthAndRay() {
    ray = center - MVS::getInstance().getCamera(refCamIdx).getCenter();
    depth = norm(ray);
    ray = ray * (1.0 / depth);
}

void Patch::setDepthRange() {
	const MVS &mvs   = MVS::getInstance();
    const int camNum = getCameraNumber();

    const Camera &refCam = mvs.getCamera(refCamIdx);

    const double cellSize = MVS::getInstance().cellSize;

    // center shift
    Vec3d c2 = ray * (depth+1.0) + refCam.getCenter();
    // projected point
    Vec2d p1, p2;

    double imgDist, worldDist;
    double maxWorldDist = -DBL_MAX;
    for (int i = 0; i < camNum; i++) {
        if (camIdx[i] == refCamIdx) continue;
                
        const Camera &cam = mvs.getCamera(camIdx[i]);

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
}

void Patch::setLOD() {
	if (refCamIdx < 0) {
        printf("Reference camera index not set\n");
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
    LOD = -1;

    // find LOD
	while (variance < mvs.textureVariation) {
        // goto next LOD
        LOD++;

        // return if reach the max LOD
        if (LOD >= pyramid.size()) {
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
}

void Patch::setCorrelationTable() {
	const MVS &mvs = MVS::getInstance();
	const vector<Camera> &cameras = mvs.cameras;

	// camera parameters
	const int camNum        = getCameraNumber();
	const Camera &refCam    = mvs.getCamera(refCamIdx);
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
		const Mat_<uchar> &img = cameras[camIdx[i]].getPyramidImage(LOD);
		getHomographyPatch(pt, img, H[i], HP[i]);
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

	// set average correlation
	correlation = 0;
	for (int i = 0; i < corrTable.rows; ++i) {
		for (int j = 0; j < corrTable.cols; ++j) {
			correlation += corrTable.at<double>(i,j);
		}
	}
	correlation /= (camNum*camNum-camNum);
}

void Patch::setPriority() {
	const MVS &mvs = MVS::getInstance();
	const int totalCamNum = (int) mvs.getCameras().size();
	const int camNum = getCameraNumber();
	double camRatio = 1.0 - ((double) camNum) / ((double) totalCamNum);
	priority = fitness * exp(-correlation) * camRatio;
}

void Patch::setImagePoint() {
	const MVS &mvs                = MVS::getInstance();
	const int camNum              = getCameraNumber();
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
	const MVS &mvs = MVS::getInstance();
	const int camNum = getCameraNumber();

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

	// remove invisible camera
	vector<int> removeIdx;
	// mark idx
	for (int i = 0; i < camNum; ++i) {
		if (i == maxIdx) continue;
		if (corrTable.at<double>(maxIdx, i) < mvs.minCorrelation) {
			removeIdx.push_back(camIdx[i]);
		}
	}
	// remove idx
	vector<int>::iterator it;
	for (int i = 0; i < (int) removeIdx.size(); i++) {
		it = find(camIdx.begin(), camIdx.end(), removeIdx[i]);
		camIdx.erase(it);
	}
}

void Patch::expandVisibleCamera() {
	const MVS &mvs = MVS::getInstance();
	const vector<Camera> &cameras = mvs.cameras;

	vector<int> expCamIdx;
	for (int i = 0; i < cameras.size(); ++i) {
		const Camera &cam = cameras[i];
		if (normal.ddot(-cam.getOpticalNormal()) >= mvs.visibleCorrelation) {
			camIdx.push_back(i);
		}
	}

	sort(camIdx.begin(), camIdx.end());
	vector<int>::iterator it = unique(camIdx.begin(), camIdx.end());
	camIdx.resize(it - camIdx.begin());
}

/* fitness function */

double PAIS::getFitness(const Particle &p, void *obj) {
	// current patch
	const Patch  &patch   = *((Patch *)obj);
	// visible camera indices
	const vector<int> &camIdx = patch.getCameraIndices();
	// MVS
	const MVS  &mvs               = MVS::getInstance();
	const int patchRadius         = mvs.getPatchRadius();
	const vector<Camera> &cameras = mvs.getCameras();

	// camera parameters
	const Camera &refCam  = mvs.getCamera(patch.getReferenceCameraIndex());
	const Mat_<double> &KRF = refCam.getKR();
	const Mat_<double> &KTF = refCam.getKT();
	const int camNum = patch.getCameraNumber();
	// level of detail
	int LOD = patch.getLOD();

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