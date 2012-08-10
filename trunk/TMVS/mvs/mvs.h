#ifndef __PAIS_MVS_H__
#define __PAIS_MVS_H__

#include <math.h>
#define _USE_MATH_DEFINES

#include "../io/fileloader.h"
#include "../io/filewriter.h"
#include "cellmap.h"

// trigger viewer event
extern void addPatchView(const Patch &pth);

namespace PAIS {
	class CellMap;
	class Camera;
	class Patch;

	class MvsConfig {
	public:
		// image cell size (pixel*pixel)
		int cellSize;
		// patch radius in pixel
		int patchRadius;
		// patch size 2*radius+1
		int patchSize;
		// minimum visible camera number
		int minCamNum;
		// intensity variation in patch for LOD
		double textureVariation;
		// expand visible camera
		double visibleCorrelation;
		// minimum patch correlation when filtering patch visible camera
		double minCorrelation;
		// fitness threshold
		double maxFitness;
		// LOD ratio
		double lodRatio;
		// minimum LOD
		int minLOD;
		// maximum LOD
		int maxLOD;
		// maxium cell patch number
		int maxCellPatchNum;
		// patch distance weighting
		double distWeighting;
		// patch difference weighting
		double diffWeighting;
		// neighbor radius
		double neighborRadius;
		// minimum region ratio
		double minRegionRatio;
		// depth range scalar (pixel)
		double depthRangeScalar;
		// PSO parameter
		// particle number
		int particleNum;
		// maximum iteration number
		int maxIteration;
		// expansion strategy (best, worst, breath, depth)
		int expansionStrategy;
	};

	class MVS : private MvsConfig {
	private:
		// instance holder
		static MVS *instance;

		// constructor
		MVS(const MvsConfig &config);
		~MVS(void);

		// camera container
		vector<Camera>  cameras;
		// patch container (id, patch)
		map<int, Patch> patches;
		// cell map container
		vector<CellMap> cellMaps;
		// pixel-wised distance weighting of patch
		Mat_<double> patchDistWeight;
		// priority queue (patch id)
		mutable vector<int> queue;

		/*******************
			initialization 
		********************/
		// set initialize cell maps and project inti cells
		void setCellMaps();
		// initialize cell map using given cell size
		bool initCellMaps();
		// initialize priority queue
		void initPriorityQueue();
		// initial pixel-wised distance weighting of patch
		void initPatchDistanceWeighting();
		// re-centering patches
		void reCentering();

		/******************
			expansion
		*******************/
		// expansion one ring neighbor cells in all visible images (optional: only reference image)
		void expandNeighborCell(const Patch &pth);
		// expansion cell
		void expandCell(const Camera &cam, const Patch &parent, const int cx, const int cy);

		/*****************
			get patch id from queue
		******************/
		// get patch id from queue (entry function)
		int getPatchIdFromQueue() const;
		// get top priority patch id from queue to expansion (best first)
		int getTopPriorityPatchId() const;
		// get last priority patch id from queue to expansion (worst first)
		int getLastPriorityPatchId() const;
		// get next patch id from queue to expansion (breath first)
		int getBreathFirstPatchId() const;
		// get next patch id from queue to expansion (depth first)
		int getDepthFirstPatchId() const;

		// check neighbor patches in cell
		bool skipNeighborCell(const vector<int> &cell, const Patch &refPth) const;
		// get new expansion center
		void getExpansionPatchCenter(const Camera &cam, const Patch &parent, const int cx, const int cy, Vec3d &center) const;
		// patch filter (false: filter out)
		bool runtimeFiltering(const Patch &pth) const;

		/*****************
			misc functions
		******************/
		// insert new patch in patch pool and queue
		void insertPatch(const Patch &pth);
		// delete patch and return next patch iterator
		map<int, Patch>::iterator deletePatch(Patch &pth);
		map<int, Patch>::iterator deletePatch(const int id);

	public:
		friend class FileWriter;
		friend class FileLoader;
		friend class Patch;
		friend class Camera;
		friend class MvsViewer;

		static const int EXPANSION_BEST_FIRST   = 0x00;
		static const int EXPANSION_WORST_FIRST  = 0x01;
		static const int EXPANSION_BREATH_FIRST = 0x02;
		static const int EXPANSION_DEPTH_FIRST  = 0x03;

		/*****************
			instance getter
		******************/
		static MVS& getInstance() { return *instance; }
		static MVS& getInstance(const MvsConfig &config);

		void setConfig(const MvsConfig &config);

		void loadNVM(const char *fileName);
		void loadNVM2(const char *fileName);
		void loadMVS(const char *fileName);
		void writeMVS(const char *fileName) const;
		void writePLY(const char *fileName) const;
		void writePSR(const char *fileName) const;

		// getter
		const vector<Camera>&  getCameras()             const { return cameras;         }
		const Camera& getCamera(const int idx)          const { return cameras[idx];    }
		const map<int, Patch>& getPatches()             const { return patches;         }
		const vector<CellMap>& getCellMaps()            const { return cellMaps;        }
		const Mat_<double>& getPatchDistanceWeighting() const { return patchDistWeight; }
		const Patch* getPatch(const int id) const;
		Patch* getPatch(const int id);

		int    getCellSize()           const { return cellSize;           } 
		int    getPatchRadius()        const { return patchRadius;        }
		int    getPatchSize()          const { return patchSize;          }
		double getTextureVariation()   const { return textureVariation;   }
		double getVisibleCorrelation() const { return visibleCorrelation; }
		double getDifferenceWeight()   const { return diffWeighting;      }
		double getDistanceWeight()     const { return distWeighting;      }
		int    getMinLOD()             const { return minLOD;             }

		// refine seed patches
		void refineSeedPatches();
		// expand neighbor cell patches
		void expansionPatches();
		// patch filtering
		void cellFiltering();
		void neighborCellFiltering(const double neighborRatio);
		void visibilityFiltering();
		void neighborPatchFiltering();
		// patch quantization (optional)
		void patchQuantization(const int thetaNum, const int phiNum, const int distNum);
		// print config information
		void printConfig() const;
	};
};

#endif