#include "mvs.h"

using namespace PAIS;

MVS::MVS(const int cellSize, const int patchRadius, const double textureVariation) {
	this->cellSize         = cellSize;
	this->patchRadius      = patchRadius;
	this->textureVariation = textureVariation;
	this->patchSize        = (patchRadius<<1)+1;
	initPatchDistanceWeighting();
}

MVS::~MVS(void) {

}

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

void MVS::loadNVM(const char* fileName) {
	FileLoader::loadNVM(fileName, *this);
	initCellMaps();
}

bool MVS::refineSeedPatches() {
	if ( patches.empty() ) {
		printf("No seed patches\n");
		return false;
	}

	map<int, Patch>::iterator it;
	for (it = patches.begin(); it != patches.end(); ) {
		Patch &pth = (*it).second;

		// remove patch with few visible camera
		if (pth.getCameraNumber() < MIN_CAMERA_NUMBER) {
			it = patches.erase(it);
			continue;
		}

		pth.refineSeed();

		// remove patch with few visible camera
		if (pth.getCameraNumber() < MIN_CAMERA_NUMBER) {
			it = patches.erase(it);
		}

		++it;
	}

	return true;
}

int MVS::getTopPriorityPatchId() const {
	map<int, Patch>::const_iterator it;
	double topPriority = DBL_MAX;
	int topId;

	for (it = patches.begin(); it != patches.end(); ++it) { 
		const Patch &pth = (*it).second;
		// skip expanded
		if (pth.isExpanded()) continue;
		// update top priority
		if (pth.getPriority() < topPriority) {
			topPriority = pth.getPriority();
			topId = pth.getId();
		}
	}

	return topId;
}