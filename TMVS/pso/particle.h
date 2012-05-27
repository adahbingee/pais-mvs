#ifndef __PAIS_PARTICLE_H__
#define __PAIS_PARTICLE_H__

namespace PAIS {
	class Particle {
	public:
		// parameter dimension
        int dim;
        // best parameter
        double *pBest;
		// near neighbor best
		double *nBest;
		// local best
		const double *lBest;
        // current parameter
        double *pos;
        // current velocity
        double *vec;
        // current fitness
        double fitness;
        // personal best fitness
        double pBestFitness;

        Particle(int dim);
        Particle(const Particle &p);
        Particle& operator=(const Particle &p);
        ~Particle(void);
	};
};

#endif