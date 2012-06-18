#include "mvs.h"

using namespace PAIS;

MVS* MVS::instance = NULL;

/* constructor */

MVS& MVS::getInstance(const MvsConfig &config) {
	if (instance==NULL) {
		instance = new MVS(config);
	}
	return *instance;
}

MVS::MVS(const MvsConfig &config) {
	this->cellSize           = config.cellSize;
	this->patchRadius        = config.patchRadius;
	this->minCamNum          = config.minCamNum;
	this->visibleCorrelation = config.visibleCorrelation;
	this->textureVariation   = config.textureVariation;
	this->minCorrelation     = config.minCorrelation;
	this->minLOD             = config.minLOD;
	this->maxCellPatchNum    = config.maxCellPatchNum;
	this->distWeighting      = config.distWeighting;
	this->diffWeighting      = config.diffWeighting;
	this->particleNum        = config.particleNum;
	this->maxIteration       = config.maxIteration;
	
	this->patchSize        = (patchRadius<<1)+1;
	initPatchDistanceWeighting();
}

MVS::~MVS(void) {

}

/* initialize */

bool MVS::initCellMaps() {
	if (cameras.empty()) {
		printf("can't initial cell maps\n");
		return false;
	}

	cellMaps.clear();

	for (int i = 0; i < cameras.size(); i++) {
		cellMaps.push_back( CellMap(cameras[i], cellSize) );
	}

	return true;
}

void MVS::initPatchDistanceWeighting() {
	patchDistWeight = Mat_<double>(patchSize, patchSize);
	double sigma = distWeighting;
	double s2 = 1.0/(2.0*sigma*sigma);
    double s = 1.0/(2.0*M_PI*sigma*sigma);

	double e, g;
	for (int x = 0; x < patchSize; ++x) {
		for (int y = 0; y < patchSize; ++y) {
			e = -( pow((double)(x-patchRadius), 2)+ pow((double)(y-patchRadius), 2))*s2;
			g = s*exp(e);
			patchDistWeight.at<double>(x,y) = g;
		}
	}

	Scalar n = sum(patchDistWeight);
	patchDistWeight = patchDistWeight / n[0];
}

/* io */

void MVS::loadNVM(const char* fileName) {
	FileLoader::loadNVM(fileName, *this);
	initCellMaps();
}

void MVS::loadMVS(const char* fileName) {
	FileLoader::loadMVS(fileName, *this);
	initCellMaps();
}

void MVS::writeMVS(const char* fileName) {
	FileWriter::writeMVS(fileName, *this);
}

/* main functions */

void MVS::refineSeedPatches() {
	if ( patches.empty() ) {
		printf("No seed patches\n");
		return;
	}

	map<int, Patch>::iterator it;
	for (it = patches.begin(); it != patches.end(); ) {
		Patch &pth = it->second;

		// remove patch with few visible camera
		if (pth.getCameraNumber() < minCamNum) {
			it = patches.erase(it);
			continue;
		}

		pth.refine();

		if ( !patchFilter(pth) ) {
			it = patches.erase(it);
			continue;
		}

		printf("ID: %d \t fit: %.2f \t pri: %.2f\n", pth.getId(), pth.getFitness(), pth.getPriority());

		++it;
	}
	return;
}

void MVS::expansionPatches() {
	int pthId = getTopPriorityPatchId();
	int count = 0;
	while ( pthId >= 0) {
		// get top priority seed patch
		Patch &pth = getPatch(pthId);
		pth.setExpanded();
		// get next seed patch
		pthId = getTopPriorityPatchId();

		printf("parent: fit: %f \t pri: %f \t camNum: %d\n", pth.getFitness(), pth.getPriority(), pth.getCameraNumber());
		
		// skip
		if ( !patchFilter(pth) ) {
			deletePatch(pth);
			continue;
		}

		// expand patch
		expandNeighborCell(pth);
		
		if (count++ % 500 == 0) {
			writeMVS("auto_save.mvs");
		}
	}
}

void MVS::patchQuantization(const int thetaNum, const int phiNum, const int distNum) {
	// normal bound in spherical coordinate
	double minTheta =  DBL_MAX;
	double maxTheta = -DBL_MAX;
	double minPhi   =  DBL_MAX;
	double maxPhi   = -DBL_MAX;
	// plane distance bound to origin
	double minDist  =  DBL_MAX;
	double maxDist  = -DBL_MAX;

	// get bound
	double dist;
	for (map<int, Patch>::iterator it = patches.begin(); it != patches.end(); ++it) {
		const Patch &pth = it->second;
		const Vec2d &normalS = pth.getSphericalNormal();
		dist = -pth.getNormal().ddot(pth.getCenter());

		if (dist < minDist) {
			minDist = dist;
		}
		if (dist > maxDist) {
			maxDist = dist;
		}
		if (normalS[0] < minTheta) {
			minTheta = normalS[0];
		}
		if (normalS[0] > maxTheta) {
			maxTheta = normalS[0];
		}
		if (normalS[1] < minPhi) {
			minPhi = normalS[1];
		}
		if (normalS[1] > maxTheta) {
			maxPhi = normalS[1];
		}
	}

	// set container
}

/* process */

void MVS::expandNeighborCell(const Patch &pth) {
	const int camNum               = pth.getCameraNumber();
	const vector<int> &camIdx      = pth.getCameraIndices();
	const vector<Vec2d> &imgPoints = pth.getImagePoints();

	int cx, cy;
	for (int i = 0; i < camNum; ++i) {
		// camera
		const Camera &cam = cameras[camIdx[i]];
		// cell map
		const CellMap &map = cellMaps[camIdx[i]];

		// position on cell map
		cx = (int) (imgPoints[i][0] / cellSize);
		cy = (int) (imgPoints[i][1] / cellSize);

		// check neighbor cells
		int nx [] = {cx, cx-1, cx-1, cx+1, cx+1};
		int ny [] = {cy, cy-1, cy+1, cy-1, cy+1};
		for (int j = 0; j < 5; ++j) {
			// skip out of map
			if ( !map.inMap(nx[j], ny[j]) ) continue;

			// skip neighbor cell with exist neighbor patch or discontinuous
			const vector<int> &cell = map.getCell(nx[j], ny[j]);
			if ( hasNeighborPatch(cell, pth) ) continue;

			// expand neighbor cell (create expansion patch)
			expandCell(cam, pth, nx[j], ny[j]);
		} // end of neighbor cell
	} // end of cameras
}

void MVS::expandCell(const Camera &cam, const Patch &parent, const int cx, const int cy) {
	// get expansion patch center
	Vec3d center;
	getExpansionPatchCenter(cam, parent, cx, cy, center);

	// get expansion patch
	Patch expPatch(center, parent);
	expPatch.refine();

	insertPatch(expPatch);
}

void MVS::insertPatch(const Patch &pth) {
	if ( !patchFilter(pth) ) return;

	// insert into patches container
	patches.insert(pair<int, Patch>(pth.getId(), pth));
	
	// insert into cell maps
	const int camNum = pth.getCameraNumber();
	const vector<Vec2d> &imgPoints = pth.getImagePoints();
	const vector<int>   &camIdx    = pth.getCameraIndices();
	int cx, cy;
	for (int i = 0; i < camNum; ++i) {
		cx = (int) (imgPoints[i][0] / cellSize);
		cy = (int) (imgPoints[i][1] / cellSize);
		cellMaps[camIdx[i]].insert(cx, cy, pth.getId());
	}
}

void MVS::deletePatch(Patch &pth) {
	map<int, Patch>::iterator it = patches.find(pth.getId());
	if (it == patches.end()) return;
	patches.erase(it);
}

void MVS::deletePatch(const int id) {
	map<int, Patch>::iterator it = patches.find(id);
	if (it == patches.end()) return;
	patches.erase(it);
}

/* const function */

bool MVS::hasNeighborPatch(const vector<int> &cell, const Patch &refPth) const {
	const int pthNum = (int) cell.size();
	for (int k = 0; k < pthNum; k++) {
		const Patch &pth = getPatch(cell[k]);
		if ( Patch::isNeighbor(refPth, pth) || pth.getCorrelation() > 0.95 || pthNum >= maxCellPatchNum) {
			return true;
		}
	}
	return false;
}

void MVS::getExpansionPatchCenter(const Camera &cam, const Patch &parent, const int cx, const int cy, Vec3d &center) const {
	const double focal     = cam.getFocalLength();
	const Vec2d &imgCenter = cam.getPrinciplePoint();
	const Vec3d &camCenter = cam.getCenter();
	const Vec3d &parentNormal = parent.getNormal();
	const Vec3d &parentCenter = parent.getCenter();

	// get center pixel position of cell on reference image
	const double px = (cx+0.5)*cellSize;
	const double py = (cy+0.5)*cellSize;

	// get center pixel position of cell in world coordinate
	Mat p3d(3, 1, CV_64FC1);
	p3d.at<double>(0, 0) = (px - imgCenter[0]) / focal;
	p3d.at<double>(1, 0) = (py - imgCenter[1]) / focal;
	p3d.at<double>(2, 0) = 1.0;
	p3d = cam.getRotation().t() * (p3d - cam.getTranslation());

	// get intersection point on patch plane
	const Vec3d v13 = parentCenter - camCenter;
	const Vec3d v12(p3d.at<double>(0, 0) - camCenter[0], 
			        p3d.at<double>(1, 0) - camCenter[1],
			        p3d.at<double>(2, 0) - camCenter[2]);
	const double u = parentNormal.ddot(v13) / parentNormal.ddot(v12);

	// expansion patch information
	center = camCenter + u*v12;
}

int MVS::getTopPriorityPatchId() const {
	map<int, Patch>::const_iterator it;
	double topPriority = DBL_MAX;
	int topId = -1;
	int count = 0;
	for (it = patches.begin(); it != patches.end(); ++it) { 
		const Patch &pth = (*it).second;
		// skip expanded
		if ( pth.isExpanded() ) continue;
		++count;

		// update top priority
		if (pth.getPriority() < topPriority) {
			topPriority = pth.getPriority();
			topId = pth.getId();
		}
	}

	printf("queue %d\n", count);

	return topId;
}

bool MVS::patchFilter(const Patch &pth) const {
	if (pth.getCameraNumber() < minCamNum) return false;
	if (pth.getFitness() >= 10000)         return false;
	if (pth.getFitness() == 0.0)           return false;
	if (pth.getPriority() > 10000)         return false;
	if (_isnan(pth.getFitness()))          return false;
	if (_isnan(pth.getPriority()))         return false;
	if (_isnan(pth.getCorrelation()))      return false;

	// skip background
	Vec2d pt;
	for (int i = 0; i < cameras.size(); i++) {
		const Camera &cam = cameras[i];
		const Mat_<uchar> &img = cam.getPyramidImage(0);
		if ( !cam.project(pth.getCenter(), pt) ) {
			return false;
		}
		if (img.at<uchar>(cvRound(pt[1]), cvRound(pt[0])) == 0) {
			return false;
		}
	}

	/*
	// skip visible view correlation
	int count = 0;
	for (int i = 0; i < pth.getCameraNumber(); ++i) {
		const Camera &cam = getCamera(pth.getCameraIndices()[i]);
		if (pth.getNormal().ddot(-cam.getOpticalNormal()) > visibleCorrelation) {
			count++;
		}
	}
	if (count < minCamNum) return false;
	*/

	return true;
}