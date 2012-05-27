#include "mvs.h"

using namespace PAIS;

MVS::MVS(const int cellSize, const int patchRadius, const double textureVariation) {
	this->cellSize         = cellSize;
	this->patchRadius      = patchRadius;
	this->textureVariation = textureVariation;
	this->patchSize        = (patchRadius<<1)+1;
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
	for (it = patches.begin(); it != patches.end(); it++) {
		Patch &pth = (*it).second;
		pth.refineSeed();
	}

	return true;
}