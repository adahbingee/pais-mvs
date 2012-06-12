#include "mvs.h"

using namespace PAIS;

MVS* MVS::instance = NULL;

/* constructor */

MVS& MVS::getInstance(const int cellSize, const int patchRadius, const int minCamNum, const double textureVariation, const double minCorrelation, const int particleNum, const int maxIteration) {
	if (instance==NULL) {
		instance = new MVS(cellSize, patchRadius, minCamNum, textureVariation, minCorrelation, particleNum, maxIteration);
	}
	return *instance;
}

MVS::MVS(const int cellSize, const int patchRadius, const int minCamNum, const double textureVariation, const double minCorrelation, const int particleNum, const int maxIteration) {
	this->cellSize         = cellSize;
	this->patchRadius      = patchRadius;
	this->minCamNum        = minCamNum;
	this->textureVariation = textureVariation;
	this->minCorrelation   = minCorrelation;
	this->patchSize        = (patchRadius<<1)+1;
	this->particleNum      = particleNum;
	this->maxIteration     = maxIteration;
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
	double sigma = patchRadius / 2;
	double s2 = 1.0/(2.0*sigma*sigma);
    double s = 1.0/(2.0*M_PI*sigma*sigma);

	double e, g;
	for (int x = 0; x < patchSize; ++x) {
		for (int y = 0; y < patchSize; ++y) {
			e = -( pow((double)(x-patchRadius), 2)+ pow((double)(y-patchRadius), 2))*s2;
			g = s*exp(e);
			patchDistWeight.at<double>(y,x) = g;
		}
	}

	// normalize [0 1]
	patchDistWeight = patchDistWeight / s;

	//imshow("", patchDistWeight);
	//waitKey();
}

/* loader */

void MVS::loadNVM(const char* fileName) {
	FileLoader::loadNVM(fileName, *this);
	initCellMaps();
}

/* main functions */

void MVS::refineSeedPatches() {
	if ( patches.empty() ) {
		printf("No seed patches\n");
		return;
	}

	map<int, Patch>::iterator it;
	for (it = patches.begin(); it != patches.end(); ) {
		Patch &pth = (*it).second;

		// remove patch with few visible camera
		if (pth.getCameraNumber() < minCamNum) {
			it = patches.erase(it);
			continue;
		}

		pth.refine();

		// remove patch with few visible camera
		if (pth.getCameraNumber() < minCamNum) {
			it = patches.erase(it);
		}

		++it;
	}

	return;
}

void MVS::expansionPatches() {
	int pthId = getTopPriorityPatchId();
	while ( pthId >= 0) {
		// get top priority seed patch
		const Patch &pth = getPatch(pthId);
		// expand patch
		expandNeighborCell(pth);
		// get next seed patch
		pthId = getTopPriorityPatchId();
	}
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
			if ( !hasNeighborPatch(cell, pth) ) continue;

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

	if (expPatch.getCameraNumber() < minCamNum) return;
	patches.insert(pair<int, Patch>(expPatch.getId(), expPatch));
}

/* const function */

bool MVS::hasNeighborPatch(const vector<int> &cell, const Patch &refPth) const {
	const int pthNum = (int) cell.size();
	for (int k = 0; k < pthNum; k++) {
		const Patch &pth = getPatch(cell[k]);
		if ( Patch::isNeighbor(refPth, pth) || pth.getCorrelation() > 0.9) {
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

	for (it = patches.begin(); it != patches.end(); ++it) { 
		const Patch &pth = (*it).second;
		// skip expanded
		if ( pth.isExpanded() ) continue;
		// update top priority
		if (pth.getPriority() < topPriority) {
			topPriority = pth.getPriority();
			topId = pth.getId();
		}
	}

	return topId;
}