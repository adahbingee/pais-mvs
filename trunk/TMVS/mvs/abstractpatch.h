#ifndef __PAIS_ABSTRACT_PATCH_H__
#define __PAIS_ABSTRACT_PATCH_H__

#include <opencv2\opencv.hpp>
#include "utility.h"

using namespace cv;
using namespace PAIS;

namespace PAIS {
	class AbstractPatch {
	private:
		// global patch id counter
		static int globalId;
		// patch identifier
		int id;
		// initialize patch
		void init();

	protected:
		// patch center
		Vec3d center;
		// visible camera index
		vector<int> camIdx;
		// reference camera index
		int refCamIdx;
		// normal in spherical coordinate
		Vec2d normalS;
		// normal
		Vec3d normal;
		// depth unit ray from reference camera
		Vec3d ray;
		// depth from reference camera
		double depth;
		// depth range
		Vec2d depthRange;
		// level of detail in reference image
		int LOD;
		// color
		Vec3b color;
		// image point
		vector<Vec2d> imgPoint;

		// normalized homography patch correlation table
		Mat_<double> corrTable;
		// fitness
		double fitness;
		// patch priority ((1-correlation) * fitness)
		double priority;
		// patch average correlation
		double correlation;
		// is expanded
		bool expanded;

		// setters
		void setNormal(const Vec3d &n);
		void setNormal(const Vec2d &n);
		
		// set estimated normal using sum of unit vector from point to camera
		virtual void setEstimatedNormal()      = 0;
		// set reference camera index using normal
		virtual void setReferenceCameraIndex() = 0;
		// set depth and ray to reference camera
		virtual void setDepthAndRay()          = 0;
		// set depth range using projected pixel range
		virtual void setDepthRange()           = 0;
		// set patch priority
		virtual void setPriority()             = 0;
		// set center image points
		virtual void setImagePoint()           = 0;
		// remove invisible camera using patch correlation
		virtual void removeInvisibleCamera()   = 0;

	public:
		AbstractPatch(const int id = -1);
		~AbstractPatch(void);

		// getters
		int getId()                        const    { return id;                  }
		const Vec3d& getCenter()           const    { return center;              }
		const vector<int>& getCameraIndices() const { return camIdx;              }
		int getReferenceCameraIndex()      const    { return refCamIdx;           }
		const Vec2d& getSphericalNormal()  const    { return normalS;             }
		const Vec3d& getNormal()           const    { return normal;              }
		const Vec3d& getRay()              const    { return ray;                 }
		const double getDepth()            const    { return depth;               }
		const Vec2d& getDepthRange()       const    { return depthRange;          }
		int getLOD()                       const    { return LOD;                 }
		const Vec3b& getColor()            const    { return color;               }
		const vector<Vec2d>& getImagePoints() const { return imgPoint;            }
		double getFitness()                const    { return fitness;             }
		double getPriority()               const    { return priority;            }
		double getCorrelation()            const    { return correlation;         }
		bool isExpanded()                  const    { return expanded;            }
		int getCameraNumber()              const    { return (int) camIdx.size(); }
	
		// zc ask to add
		double getFitnessCenter();

		// setters
		void setExpanded() { expanded = true; }
	};
};

#endif