#ifndef __PAIS_MVS_H__
#define __PAIS_MVS_H__

#include <math.h>
#define _USE_MATH_DEFINES

#include "../io/fileloader.h"
#include "patch.h"
#include "cellmap.h"

namespace PAIS {
	class Patch;

	class MVS {
	private:
		// instance holder
		static MVS *instance;

		// constructor
		MVS(const int cellSize, const int patchRadius, const int minCamNum, const double textureVariation, const double minCorrelation, const int particleNum, const int maxIteration);
		~MVS(void);

		// image cell size (pixel*pixel)
		int cellSize;
		// patch radius in pixel
		int patchRadius;
		// patch size 2*radius+1
		int patchSize;
		// minimum visible camera number
		int minCamNum;
		// normalized intensity variation in patch
		double textureVariation;
		// minimum patch correlation when filtering patch visible camera
		double minCorrelation;
		// PSO parameter
		// particle number
		int particleNum;
		// maximum iteration number
		int maxIteration;


		// camera container
		vector<Camera>  cameras;
		// patch container
		map<int, Patch> patches;
		// cell map container
		vector<CellMap> cellMaps;
		// pixel-wised distance weighting of patch
		Mat_<double> patchDistWeight;

		// initialize cell map using given cell size
		bool initCellMaps();
		// initial pixel-wised distance weighting of patch
		void initPatchDistanceWeighting();

		// get top priority patch id to expansion
		int getTopPriorityPatchId() const;

	public:
		friend class FileLoader;
		friend class Patch;

		static MVS& getInstance() {
			if (instance==NULL) {
				instance = new MVS(2, 15, 3, 36, 0.7, 15, 200);
			}
			return *instance;
		}
		static MVS& getInstance(const int cellSize, const int patchRadius, const int minCamNum, const double textureVariation, const double minCorrelation, const int particleNum, const int maxIteration);

		

		void loadNVM(const char* fileName);

		// getter
		const vector<Camera>&  getCameras()  const { return cameras;  }
		const Camera& getCamera(const int idx) const { return cameras[idx]; }
		const map<int, Patch>& getPatches()  const { return patches;  }
		const vector<CellMap>& getCellMaps() const { return cellMaps; }
		const Mat_<double>& getPatchDistanceWeighting() const { return patchDistWeight; }
		const Patch& getPatch(const int id) const { return patches.at(id); }
		const Patch& getTopPriorityPatch() { return getPatch(getTopPriorityPatchId()); }

		int    getCellSize()         const { return cellSize;         }
		int    getPatchRadius()      const { return patchRadius;      }
		int    getPatchSize()        const { return patchSize;        }
		double getTextureVariation() const { return textureVariation; }

		// processor
		void refineSeedPatches();
	};
};

#endif