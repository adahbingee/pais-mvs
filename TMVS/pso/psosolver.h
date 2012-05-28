#ifndef PAIS_PSO_SOLVER_H
#define PAIS_PSO_SOLVER_H
// include STL
#include <vector>
#include <algorithm>
#include <random>

// include opencv
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

// include openMP
#include <omp.h>

#include "particle.h"

using namespace std;
using namespace cv;

namespace PAIS {

	// container for local best
	struct LocalParticle {
		// distance from pbest to current position 
		double dist; 
		// pbest holder
		const Particle *p;
	};
    
	class PSOSolver
    {
    private :
        // sorter for using Local Best
        inline static bool sortLocalParticle (const LocalParticle &i, const LocalParticle &j);

        // dimension of problem space
        int dim;

        // number of iteration
        int iteration;

        // max number of iteration
        int maxIteration;

        // number of particle each iteration
        int particleNum;

        // DispersionIDX and VelocityIDX convergence threshold
        double convT;

        // particle containter
        vector<Particle> particles;

        // upper and lower range
        double *rangeL;
        double *rangeU;
        double *rangeInter; // rangeU - rangeL

        // global best
        double *gBest;
        double gBestScore;
        int    gBestIteration;

        // velocity weight
        double iw; // inertia weight  (Basic-PSO)
        double pw; // pBest weight    (Basic-PSO)
        double gw; // gBest weight    (Basic-PSO)
		double lw; // lBest weight    (GLN-PSO)
		double nw; // nBest weight    (GLN-PSO)

		// local best K nearest neighbor (GLN-PSO)
		int localK;

		// flag for using GLN-PSO
		bool enableGLNPSO;

		// GLN-PSO
		const double *lBest;
		double *nBest;
		// local best container
		mutable vector<LocalParticle> localContainer;

        // fitness function
        double (*getScore)(const Particle &p, void *obj);
		// bundled object for fitness function
		void *obj;

		// random seed
		uint32_t seed;
		// set random seed to current time and thread
		void setRandomSeed();
        // return uniform random number [0, 1]
        inline double random();

		// PSO convergence index
		inline double getDispersionIDX() const;
		inline double getVelocityIDX()   const;

        // initialize particles
        void init();

        // update particles pbest
        inline void updatePbest();

		// move particles
		inline void moveParticle();

        // update global gbest
        inline void updateGbest();

		// get local best (GLN-PSO)
		inline const double* getLocalBest(const int pidx) const;
		// get near neighbor best (GLN-PSO)
		inline void getNearBest(const int pidx) const;

    public:
        PSOSolver(int dim, double *rangeL, double *rangeU, 
            double (*getScore)(const Particle &p, void *obj), void *obj = NULL, 
            int maxIteration = 1000, int particleNum = 30, 
            double iw = 0.8, double pw = 1.2, double gw = 1.5, double lw = 1.0, double nw = 1.0, 
			int localK = 5);

        PSOSolver(const PSOSolver &p);
        PSOSolver& operator=(const PSOSolver &p);
        ~PSOSolver(void);

        void run();
		void runFullSearch(const int stepNum = 100);

        inline int           getDimension()      const { return dim; }
        inline int           getparticleNum()    const { return particleNum; }
        inline int           getMaxIteration()   const { return maxIteration; }
        inline double        getInertiaWeight()  const { return iw; }
        inline double        getPbestWeight()    const { return pw; }
        inline double        getGbestWeight()    const { return gw; }
        inline const double* getGbest()          const { return gBest; }
        inline const double* getRangeL()         const { return rangeL; }
        inline const double* getRangeU()         const { return rangeU; }
        inline double        getGbestScore()     const { return gBestScore; }
        inline int           getGbestIteration() const { return gBestIteration; }
		inline int           getIteration()      const { return iteration; }

        // setter
		void                 setGLNPSO(const bool flag); // enable/disable GLN-PSO
        void                 setConvergenceThreshold(const double threshold);
		void                 setParticle(const int pidx, const double *param);
    };
}

#endif