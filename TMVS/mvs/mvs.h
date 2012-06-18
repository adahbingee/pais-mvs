#ifndef __PAIS_MVS_H__
#define __PAIS_MVS_H__

#include <math.h>
#define _USE_MATH_DEFINES

#include "../io/fileloader.h"
#include "../io/filewriter.h"
#include "patch.h"
#include "cellmap.h"

namespace PAIS {
	class Patch;

	struct MvsConfig {
		// image cell size (pixel*pixel)
		int cellSize;
		// patch radius in pixel
		int patchRadius;
		// minimum visible camera number
		int minCamNum;
		// normalized intensity variation in patch
		double textureVariation;
		// expand visible camera
		double visibleCorrelation;
		// minimum patch correlation when filtering patch visible camera
		double minCorrelation;
		// minimum LOD
		int minLOD;
		// maxium cell patch number
		int maxCellPatchNum;
		// patch distance weighting
		double distWeighting;
		// patch difference weighting
		double diffWeighting;
		// neighbor radius
		double neighborRadius;
		// PSO parameter
		// particle number
		int particleNum;
		// maximum iteration number
		int maxIteration;
	};

	class MVS {
	private:
		// instance holder
		static MVS *instance;

		// constructor
		MVS(const MvsConfig &config);
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
		// expand visible camera
		double visibleCorrelation;
		// minimum patch correlation when filtering patch visible camera
		double minCorrelation;
		// minimum LOD
		int minLOD;
		// maxium cell patch number
		int maxCellPatchNum;
		// patch distance weighting
		double distWeighting;
		// patch difference weighting
		double diffWeighting;
		// neighbor radius
		double neighborRadius;

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

		void expandNeighborCell(const Patch &pth);
		void expandCell(const Camera &cam, const Patch &parent, const int cx, const int cy);
		void insertPatch(const Patch &pth);
		void deletePatch(Patch &pth);
		void deletePatch(const int id);

		// get top priority patch id to expansion
		int getTopPriorityPatchId() const;
		// check neighbor patches in cell
		bool hasNeighborPatch(const vector<int> &cell, const Patch &refPth) const;
		// get new expansion center
		void getExpansionPatchCenter(const Camera &cam, const Patch &parent, const int cx, const int cy, Vec3d &center) const;
		// patch filter (false: filter out)
		bool patchFilter(const Patch &pth) const;

	public:
		friend class FileLoader;
		friend class Patch;

		static MVS& getInstance() { return *instance; }
		static MVS& getInstance(const MvsConfig &config);

		void loadNVM(const char* fileName);
		void loadMVS(const char* fileName);
		void writeMVS(const char* fileName);

		// getter
		const vector<Camera>&  getCameras()  const { return cameras;  }
		const Camera& getCamera(const int idx) const { return cameras[idx]; }
		const map<int, Patch>& getPatches()  const { return patches;  }
		const vector<CellMap>& getCellMaps() const { return cellMaps; }
		const Mat_<double>& getPatchDistanceWeighting() const { return patchDistWeight; }
		const Patch& getPatch(const int id) const { return patches.at(id); }
		Patch& getPatch(const int id) { return patches.at(id); }

		int    getCellSize()           const { return cellSize;           } 
		int    getPatchRadius()        const { return patchRadius;        }
		int    getPatchSize()          const { return patchSize;          }
		double getTextureVariation()   const { return textureVariation;   }
		double getVisibleCorrelation() const { return visibleCorrelation; }
		double getDifferenceWeight()   const { return diffWeighting;      }
		double getDistanceWeight()     const { return distWeighting;      }
		int    getMinLOD()             const { return minLOD;             }

		void refineSeedPatches();
		void expansionPatches();
		void patchQuantization(const int thetaNum, const int phiNum, const int distNum);
	};
};

#endif