#ifndef __PAIS_ABSTRACT_PATCH_H__
#define __PAIS_ABSTRACT_PATCH_H__

#include "mvs.h"

namespace PAIS {
	class AbstractPatch {
	protected:
		// MVS
		const MVS *mvs;
		// patch identifier
		int id;
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
		// patch radius in reference image
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

	public:
		AbstractPatch(void);
		~AbstractPatch(void);

		const MVS&   getMVS()              const    { return *mvs;                }
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
		const Camera& getReferenceCamera() const    { return mvs->getCameras()[refCamIdx]; }
	};
};

#endif