#ifndef __PAIS_CELL_MAP_H__
#define __PAIS_CELL_MAP_H__ 

#include <map>
#include "camera.h"
#include "patch.h"

namespace PAIS {

	typedef vector<vector<vector<int> > > Map;

	class CellMap {
	private:
		int cellSize;
		int width;
		int height;
		const Camera *camera;
		Map map;

	public:
		CellMap(const Camera &camera, const int cellSize);
		~CellMap(void);

		bool inMap(const int x, const int y) const;
		bool insert(const int x, const int y, const int patchId);
		const vector<int>& getCell(const int x, const int y) const { return map[y][x]; }
	};
};

#endif