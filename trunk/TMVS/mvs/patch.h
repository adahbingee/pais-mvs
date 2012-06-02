#ifndef __PAIS_PATCH__H__
#define __PAIS_PATCH__H__

// homography patch correlation threshold
#define CORRELATION_THRESHOLD 0.7
// minimul camera number threshold
#define MIN_CAMERA_NUMBER 3

#define _USE_MATH_DEFINES
#include <math.h>

#include "mvs.h"
#include "utility.h"
#include "../pso/psosolver.h"

using namespace cv;

namespace PAIS {

	class Patch {
	private:
		// global patch id counter
		static int globalId;
		// check neighbor patch
		static bool isNeighbor(const Patch &pth1, const Patch &pth2);

		// patch identifier
		int id;
		// MVS
		const MVS *mvs;
		// visible camera index
		vector<int> camIdx;
		// reference camera index
		int refCamIdx;
		// patch center
		Vec3d center;
		// normal in spherical coordinate
		Vec2d normalS;
		// normal
		Vec3d normal;
		// depth from reference camera
		double depth;
		// depth range
		Vec2d depthRange;
		// color
		Vec3b color;
		// patch radius in reference image
		int LOD;
		// depth unit ray from reference camera
		Vec3d ray;
		// image point
		vector<Vec2d> imgPoint;
		// fitness
		double fitness;
		// normalized homography patch correlation table
		Mat_<double> corrTable;
		// patch priority ((1-correlation) * fitness)
		double priority;
		// patch average correlation
		double correlation;
		// is expanded
		bool expanded;

		// normal setters
		void setNormal(const Vec3d &n);
		void setNormal(const Vec2d &n);
		// depth setters
		bool setDepth();
		// set level of detail
		bool setLOD();
		// set estimated normal using sum of unit vector from point to camera
		bool setEstimatedNormal();
		// set reference camera index using normal
		bool setReferenceCameraIndex();
		// set depth range
		bool setDepthRange();
		// set normalized homography patch correlation table
		bool setCorrelationTable();
		// set priority
        bool setPriority();
		// set image point and color
		bool setImagePoint();

		// remove invisible camera using texture correlation
		bool removeInvisibleCamera();
		// get normalized homography patch column vector
		bool getHomographyPatch(const Vec2d &pt, const Camera &cam, const Mat_<double> &H, Mat_<double> &hp) const;
		// check neighbor cell to expand
		bool checkNeighborCell(const CellMap &map, const int cx, const int cy) const;
		// expand cell (create expansion patch)
		bool expandCell(const Camera &cam, const int cx, const int cy) const;

		// show refined projection
		void showRefinedResult() const;
		// show multi-view error
		void showError() const;

	public:
		// constructors
		Patch(const MVS *mvs, const Vec3d &center, const Vec3b &color, const vector<int> &camIdx, const vector<Vec2d> &imgPoint, const int id = -1);
		Patch(const Patch &parent, const Vec3d &center);

		// descructor
		~Patch(void);

		void refineSeed();
		void expand() const;
	
		int getId()                        const    { return id;                  }
		const Vec2d& getSphericalNormal()  const    { return normalS;             }
		const Vec3d& getNormal()           const    { return normal;              }
		const Vec3d& getCenter()           const    { return center;              }
		const Vec3d& getRay()              const    { return ray;                 }
		const Vec3b& getColor()            const    { return color;               }
	    const MVS&   getMVS()              const    { return *mvs;                }
		int getLOD()                       const    { return LOD;                 }
		int getReferenceCameraIndex()      const    { return refCamIdx;           }
		int getCameraNumber()              const    { return (int) camIdx.size(); }
		double getPriority()               const    { return priority;            }
		double getCorrelation()            const    { return correlation;         }
		bool isExpanded()                  const    { return expanded;            }
		const vector<int>& getCameraIndices() const { return camIdx;              }
		const Camera& getReferenceCamera() const;
	};

	double getFitness(const Particle &p, void *obj);
};

#endif