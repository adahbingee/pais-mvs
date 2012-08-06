#include "cellmap.h"

using namespace PAIS;

CellMap::CellMap(const Camera &camera, const int cellSize) {
	// get map size
	this->width  = cvCeil((double) camera.getImageWidth()  / (double) cellSize);
	this->height = cvCeil((double) camera.getImageHeight() / (double) cellSize);

	// initial map
	map = vector<vector<vector<int> > >(height, vector<vector<int> >(width, vector<int>()));
}

CellMap::~CellMap() {

}

bool CellMap::inMap(const int x, const int y) const {
	if (_isnan(x) || _isnan(y) || x < 0 || y < 0 || x >= width || y >= height) {
		return false;
	}
	return true;
}

bool CellMap::insert(const int x, const int y, const int patchId) {
	if ( !inMap(x, y) ) return false;
	map[y][x].push_back(patchId);
	return true;
}

bool CellMap::drop(const int x, const int y, const int patchId) {
	if ( !inMap(x, y) ) return false;
	vector<int> &cell = map[y][x];
	vector<int>::iterator it = find(cell.begin(), cell.end(), patchId);
	if (it==cell.end()) return false;
	cell.erase(it);
	return true;
}