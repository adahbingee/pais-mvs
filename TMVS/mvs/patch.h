#ifndef __PAIS_PATCH__H__
#define __PAIS_PATCH__H__

#include "../pso/psosolver.h"
#include "abstractpatch.h"
#include "mvs.h"

using namespace PAIS;
using namespace cv;

namespace PAIS {
	class MVS;

	class Patch : public AbstractPatch {
	private:
		void getHomographyPatch(const Vec2d &pt, const Camera &cam, const Mat_<double> &H, Mat_<double> &hp) const;
	
	protected:
		void setEstimatedNormal();
		void setReferenceCameraIndex();
		void setDepthAndRay();
		void setDepthRange();
		void setLOD();
		void setCorrelationTable();
		void setPriority();
		void setImagePoint();
		void removeInvisibleCamera();
	
	public:
		// seed patch constructor
		Patch(const Vec3d &center, Vec3b &color, vector<int> &camIdx, vector<Vec2d> &imgPoint, const int id = -1);
		// expansion patch constructor
		Patch(const Vec3d &center, const Patch &parent, const int id = -1);

		void refine();

		~Patch(void);
	};

	double getFitness(const Particle &p, void *obj);
};

#endif