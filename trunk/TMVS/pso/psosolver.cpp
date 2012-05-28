#include "psosolver.h"

bool PsoSolver::sortLocalParticle (const LocalParticle &i, const LocalParticle &j) {
    return (i.dist < j.dist); 
}

PsoSolver::PsoSolver(const int dim, 
			      const double *rangeL, const double *rangeU, 
				  double (*getFitness)(const Particle &p, void *obj), 
				  void *obj,
				  int maxIteration, int particleNum,
				  double convergenceThreshold, 
				  double iw, double pw, double gw, double lw, double nw,
				  int localK) {
	this->dim            = dim;
	this->maxIteration   = maxIteration;
	this->getFitness     = getFitness;
	this->obj            = obj;
	this->particleNum    = particleNum;
	this->convergenceThreshold = convergenceThreshold;
	this->iw = iw;
	this->pw = pw;
	this->gw = gw;
	this->lw = lw;
	this->nw = nw;
	this->localK = min(particleNum, localK);

	this->rangeL         = new double[dim];
	this->rangeU         = new double[dim];
	this->rangeInter     = new double[dim];
	this->gBest          = NULL;
	this->gBestFitness   = DBL_MAX;
	this->gBestIteration = -1;

	for (int i = 0; i < dim; i++) {
		this->rangeL[i]     = rangeL[i];
		this->rangeU[i]     = rangeU[i];
		this->rangeInter[i] = rangeU[i] - rangeL[i];
	}

	setRandomSeed();
	initParticles();
}

PsoSolver::~PsoSolver() {
	if (rangeL) {
		delete [] rangeL;
		rangeL = NULL;
	}
	if (rangeU) {
		delete [] rangeU;
		rangeU = NULL;
	}
	if (rangeInter) {
		delete [] rangeInter;
		rangeInter = NULL;
	}
}

void PsoSolver::setRandomSeed() const {
	// set random seed
	unsigned int seed = (unsigned int) time(NULL) + omp_get_thread_num();
	srand(seed);
}

inline double PsoSolver::random() {
	return ((double) rand()) / ((double) RAND_MAX);
}

double PsoSolver::getDispersionIDX() const {
	double index = 0;
	for (int i = 0; i < particleNum; i++) {
		const Particle &p = particles[i];
		for (int j = 0; j < dim; j++) {
			index += abs(p.pos[j] - gBest[j]);
		}
	}
	index /= (dim*particleNum);
	return index;
}

double PsoSolver::getVelocityIDX()   const {
	double index = 0;
	for (int i = 0; i < particleNum; i++) {
		const Particle &p = particles[i];
		for (int j = 0; j < dim; j++) {
			index += abs(p.vec[j]);
		}
	}
	index /= (dim*particleNum);
	return index;
}

void PsoSolver::initParticles() {
	// allocate particle container
	particles.clear();
	particles.resize(particleNum, Particle(dim));

	// uniform random parameter between range
	for (int d = 0; d < dim; d++) {
		for (int i = 0; i < particleNum; i++) {
			// random position parameter (L~U)
            particles[i].pos[d] = (rangeInter[d] * random()) + rangeL[d];
            // random velocity parameter (-|U-L| ~ |U-L|), known as velocity inertia
            particles[i].vec[d] = (2.0 * rangeInter[d] * random()) - rangeInter[d];
            // set pBest as initial position
            particles[i].pBest[d] = particles[i].pos[d];
		}
	}
}

void PsoSolver::initFitness() {
	#pragma omp parallel for
	for (int i = 0; i < particleNum; i++) {
		Particle &p    = particles[i];
		p.fitness      = getFitness(p, obj);
		p.pBestFitness = p.fitness;
	}
}

void PsoSolver::updateFitness() {
	#pragma omp parallel for
	for (int i = 0; i < particleNum; i++) {
		Particle &p    = particles[i];
		p.fitness      = getFitness(p, obj);

		// update pBest
		if (p.fitness < p.pBestFitness) {
			p.pBestFitness = p.fitness;
			for (int d = 0; d < p.dim; d++) {
				p.pBest[d] = p.pos[d];
			}
		}
	} // end of update particles
}

void PsoSolver::updateGbest() {
	for (int j = 0; j < particleNum; j++) {
        // current particle
        const Particle &p = particles[j];

		if (p.pBestFitness <= gBestFitness) {
            gBestFitness   = p.pBestFitness;
			gBest          = p.pBest;
			gBestIteration = iteration;
			// printf("update: %f\n", gBestFitness);
        }
    }
}

const double* PsoSolver::getLocalBest(const int idx) const {
	// local particle container
	vector<LocalParticle> container(particleNum);

	// current pBest position
	const double *pos = particles[idx].pBest;

	// get Euclidean distence from pBest to current pBest position
	for (int i = 0; i < particleNum; i++) {
		const Particle &p = particles[i];
		LocalParticle &localP = container[i];
		localP.dist = 0;
		localP.p = &p;

		if (i == idx) {
			localP.dist = DBL_MAX;
			continue;
		}

		for (int d = 0; d < dim; d++) {
			localP.dist += (pos[d] - p.pBest[d])*(pos[d] - p.pBest[d]);
		}
	}

	// sort by distance
	sort(container.begin(), container.end(), sortLocalParticle);

	// find the minimum fitness pbest as lbest from localK nearest neighbors
	double minFitness = DBL_MAX;
	const double *lBest = pos;
	for (int k = 0; k < localK; k++) {
		const LocalParticle &localP = container[k];
		const Particle &p = *localP.p;
		if (p.pBestFitness < minFitness) {
			minFitness = p.pBestFitness;
			lBest = p.pBest;
		}
	}

	return lBest;
}

void PsoSolver::setNearNeighborBest(const int idx) {
	// current fitness
	const double fitness = particles[idx].fitness;
	// current position
	const double *pos    = particles[idx].pos;
	// near neighbor best
	double *nBest = particles[idx].nBest;

	double FDR;
	double maxFDR;

	for (int d = 0; d < dim; d++) { // loop dimension
		maxFDR = -DBL_MAX;
		for (int i = 0; i < particleNum; i++) { // loop particle
			if (i == idx) continue; // skip current particle
			const Particle &p = particles[i];

			FDR = (fitness - p.pBestFitness) / abs(pos[d] - p.pBest[d]);

			if (FDR > maxFDR) {
				maxFDR = FDR;
				nBest[d] = p.pBest[d];
			}
		}
	}
}

void PsoSolver::moveParticles() {
	
	#pragma omp parallel for
	for (int i = 0; i < particleNum; i++) {
		// velocity weighting
		double pVecW, gVecW, lVecW, nVecW;

		// current particle
		Particle &p = particles[i];

		// get random weighting w * [0 ~ 1]
		// pBest, gBest, lBest, nBest weighting noise
		pVecW = pw * random();
        gVecW = gw * random();

		if (enableGLNPSO) {
			lVecW = lw * random();
			nVecW = nw * random();
			p.lBest = getLocalBest(i);
			setNearNeighborBest(i);
		}
		
		for (int d = 0; d < dim; d++) {
			// update velocity
			if (enableGLNPSO) {
				p.vec[d] = iw*p.vec[d]                 + 
					       pVecW*(p.pBest[d]-p.pos[d]) +
						   gVecW*(gBest[d]-p.pos[d])   + 
						   lVecW*(p.lBest[d]-p.pos[d]) +
						   nVecW*(p.nBest[d]-p.pos[d]);
			} else {
				p.vec[d] = iw*p.vec[d]                 + 
					       pVecW*(p.pBest[d]-p.pos[d]) +
						   gVecW*(gBest[d]-p.pos[d]);
			}

			// update position
			p.pos[d] += p.vec[d];

			// parameter bound check
            if (p.pos[d] > rangeU[d]) p.pos[d] = rangeU[d];
            if (p.pos[d] < rangeL[d]) p.pos[d] = rangeL[d];
		} // end of velocity dimension

	} // end of move particles
}

bool PsoSolver::setParticle(const double *pos, const double *vec, const int idx) {
	if (pos == NULL) {
		printf("set particle fail\n");
		return false;
	}

	for (int d = 0; d < dim; d++) {
		particles[idx].pos[d]   = pos[d];
		particles[idx].pBest[d] = particles[idx].pos[d];
		if ( vec != NULL) {
			particles[idx].vec[d] = vec[d];
		} else {
			particles[idx].vec[d] = (2.0 * rangeInter[d] * random()) - rangeInter[d];
		}
	}

	return true;
}

void PsoSolver::run(const bool enableGLNPSO, const double minIw) {
	this->enableGLNPSO = enableGLNPSO;
	initFitness();
	updateGbest();

	for (iteration = 0; iteration < maxIteration; iteration++) {

		if (getDispersionIDX() < convergenceThreshold && getVelocityIDX() < convergenceThreshold) {
			break;
		}

		moveParticles();
		updateFitness();
		updateGbest();

		// linear interia weighting adjustment
		iw = max(iw - 1.0/maxIteration, minIw);
	} // end of iteration
}