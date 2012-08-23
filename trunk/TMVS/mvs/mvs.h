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
		// reduce PSO search range for expansion patch
		double reduceNormalRange;
		// enable adaptive weighting
		bool adaptiveDistanceEnable;
		bool adaptiveDifferenceEnable;
		bool adaptiveGradientEnable;
		// patch distance weighting
		double distWeighting;
		// patch difference weighting
		double diffWeighting;
		// patch gradient maginitude weighting
		double gradientWeighting;
		// neighbor radius
		double neighborRadius;
		// neighbor radius scalar (PCMVS)
		double neighborRadiusScalar;
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
		// deleted patch container
		vector<Patch> deletedPatches;
		
		/* getter */
		// get patch by id
		Patch* getPatch(const int id);

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
		// delete patch and return next patch iterator and push deleted patch into deleted patches container
		map<int, Patch>::iterator deletePatch(Patch &pth);
		map<int, Patch>::iterator deletePatch(const int id);
		// set neighbor radius from bounding volume
		void setNeighborRadius();

	public:
		friend class FileWriter;
		friend class FileLoader;
		friend class Patch;
		friend class Camera;
		friend class FeatureManager;

		// expansion strategy
		static const int EXPANSION_BEST_FIRST   = 0x00;
		static const int EXPANSION_WORST_FIRST  = 0x01;
		static const int EXPANSION_BREATH_FIRST = 0x02;
		static const int EXPANSION_DEPTH_FIRST  = 0x03;

		/*****************
			instance getter
		******************/
		// singleton getter
		static MVS& getInstance() { return *instance; }
		static MVS& getInstance(const MvsConfig &config);

		// set config and initialize
		void setConfig(const MvsConfig &config);

		// load NVM file
		void loadNVM(const char *fileName);
		// load NVM2 file
		void loadNVM2(const char *fileName);
		// load MVS file
		void loadMVS(const char *fileName);
		// write current state to MVS file (including cameras, patches, config)
		void writeMVS(const char *fileName) const;
		// write current state to PLY file (including patch vertex and normal)
		void writePLY(const char *fileName) const;
		// write current state to PSR file (including patch vertex and normal)
		void writePSR(const char *fileName) const;
		// write deleted patches to MVS file (including cameras, deleted patches, config)
		void writeDeletedPatchMVS(const char *fileName) const;
		// write deleted patches to PLY file (including deleted patch vertex and normal)
		void writeDeletedPatchPLY(const char *fileName) const;

		/* getter */
		// get system cameras
		const vector<Camera>&  getCameras()             const { return cameras;         }
		// get camera by its index
		const Camera& getCamera(const int idx)          const { return cameras[idx];    }
		// get system patches
		const map<int, Patch>& getPatches()             const { return patches;         }
		// get deleted patches
		const vector<Patch>& getDeletedPatches()        const { return deletedPatches;  }
		// get system cell maps
		const vector<CellMap>& getCellMaps()            const { return cellMaps;        }
		// get pre-computed patch distance matrix (same size of patch size)
		const Mat_<double>& getPatchDistanceWeighting() const { return patchDistWeight; }
		// get patch by id
		const Patch* getPatch(const int id) const;
		
		int    getCellSize()           const { return cellSize;           } 
		int    getPatchRadius()        const { return patchRadius;        }
		int    getPatchSize()          const { return patchSize;          }
		double getTextureVariation()   const { return textureVariation;   }
		double getVisibleCorrelation() const { return visibleCorrelation; }
		double getDifferenceWeight()   const { return diffWeighting;      }
		double getDistanceWeight()     const { return distWeighting;      }
		double getGradientWeight()     const { return gradientWeighting;  }
		int    getMinLOD()             const { return minLOD;             }
		double getReduceNormalRange()  const { return reduceNormalRange;  }
		double getBoundingVolume(Vec3d *minPtr, Vec3d *maxPtr) const;
		bool isAdaptiveDistanceEnable()   const { return adaptiveDistanceEnable;   }
		bool isAdaptiveDifferenceEnable() const { return adaptiveDifferenceEnable; }
		bool isAdaptiveGradientEnable()   const { return adaptiveGradientEnable;   }

		// print config information
		void printConfig() const;

		/* refine seed patches */
		void refineSeedPatches();
		/* expand neighbor cell patches from priority queue */
		void expansionPatches();
		/* PMVS filtering */
		void cellFiltering();
		void neighborCellFiltering(const double neighborRatio);
		void visibilityFiltering();
		/* TVCG09 PCMVS filtering*/
		void neighborPatchFiltering(const double neighborRatio);
		// clear deleted patches container
		void clearDeletedPatches();
	};
};

#endif