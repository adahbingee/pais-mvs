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
		// get homography texture 1D vector
		void getHomographyPatch(const Vec2d &pt, const Mat_<uchar> &img, const Mat_<double> &H, Mat_<double> &hp) const;
		// expand visible camera using normal correlation
		void expandVisibleCamera();

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
		static bool isNeighbor(const Patch &pth1, const Patch &pth2);
		
		// seed patch constructor
		Patch(const Vec3d &center, const Vec3b &color, const vector<int> &camIdx, const vector<Vec2d> &imgPoint, const int id = -1);
		// expansion patch constructor
		Patch(const Vec3d &center, const Patch &parent, const int id = -1);
		// mvs loader constructor
		Patch(const Vec3d &center, const Vec2d &normalS, const vector<int> &camIdx, const double fitness, const double correlation, const int id = -1);

		void refine();

		~Patch(void);
	};

	double getFitness(const Particle &p, void *obj);
};

#endif