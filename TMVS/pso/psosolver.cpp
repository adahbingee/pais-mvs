#include "psosolver.h"

using namespace PAIS;

inline bool PSOSolver::sortLocalParticle (const LocalParticle &i, const LocalParticle &j) {
    return (i.dist < j.dist); 
}

// object functions

void PSOSolver::setRandomSeed() {
	// set random seed
	unsigned int seed = (unsigned int) time(NULL) + omp_get_thread_num();
	srand(seed);
}

inline double PSOSolver::random() {
	return ((double) rand()) / ((double) RAND_MAX);
}

PSOSolver::PSOSolver(int dim, double *rangeL, double *rangeU, 
            double (*getScore)(const Particle &p, void *obj), void *obj,
            int maxIteration, int particleNum, 
            double iw, double pw, double gw, double lw, double nw, 
			int localK)
{
	// set seed for uniform random
	setRandomSeed();

    this->dim = dim;
    this->maxIteration   = maxIteration;
    this->iteration      = 0;
    this->particleNum    = particleNum;
    this->gBestScore     = 1.7976931348623158e+308;
    this->iw             = iw;
    this->pw             = pw;
    this->gw             = gw;
	this->lw             = lw;
	this->nw             = nw;
    this->getScore       = getScore;
    this->gBestIteration = 0;
	this->localK         = min(localK, particleNum);
	this->enableGLNPSO   = false;
    this->convT          = 0.01;
	this->obj            = obj;
    this->rangeL         = new double [dim];
    this->rangeU         = new double [dim];
    this->rangeInter     = new double [dim];
    this->gBest          = new double [dim];
	this->nBest          = new double [dim];
    
    for (int i = 0; i < dim; i++) {
        this->rangeL[i]     = rangeL[i];
        this->rangeU[i]     = rangeU[i];
        this->rangeInter[i] = rangeU[i] - rangeL[i];
        this->gBest[i]      = 0;
		this->nBest[i]      = 0;
    }

	// allocate localContainer
	localContainer.resize(particleNum);

	init();
}

PSOSolver::PSOSolver(const PSOSolver &p) {
    this->dim            = p.dim;
    this->maxIteration   = p.maxIteration;
    this->iteration      = p.iteration;
    this->particleNum    = p.particleNum;
    this->particles      = p.particles;
    this->gBestScore     = p.gBestScore;
    this->iw             = p.iw;
    this->pw             = p.pw;
    this->gw             = p.gw;
	this->lw             = p.lw;
	this->nw             = p.nw;
	this->localK         = p.localK;
	this->enableGLNPSO   = p.enableGLNPSO;
    this->convT          = p.convT;
    this->getScore       = p.getScore;
    this->gBestIteration = p.gBestIteration;
	this->obj            = obj;
    this->rangeL         = new double [dim];
    this->rangeU         = new double [dim];
    this->rangeInter     = new double [dim];
    this->gBest          = new double [dim];
	this->lBest          = new double [dim];
	this->nBest          = new double [dim];

    for (int i = 0; i < dim; i++) {
        this->rangeL[i]     = p.rangeL[i];
        this->rangeU[i]     = p.rangeU[i];
        this->rangeInter[i] = p.rangeInter[i];
        this->gBest[i]      = p.gBest[i];
		this->nBest[i]      = p.nBest[i];
    }

	// initialize localContainer
	localContainer.resize(particleNum);
}

PSOSolver& PSOSolver::operator=(const PSOSolver &p) {
    if (dim != p.dim) {
        this->dim = p.dim;

        if (this->rangeL)     delete [] this->rangeL;
        if (this->rangeU)     delete [] this->rangeU;
        if (this->rangeInter) delete [] this->rangeInter;
        if (this->gBest)      delete [] this->gBest;
		if (this->nBest)      delete [] this->nBest;

        this->rangeL     = new double [dim];
        this->rangeU     = new double [dim];
        this->rangeInter = new double [dim];
        this->gBest      = new double [dim];
		this->nBest      = new double [dim];
    }
    this->maxIteration   = p.maxIteration;
    this->iteration      = p.iteration;
    this->particleNum    = p.particleNum;
    this->particles      = p.particles;
    this->gBestScore     = p.gBestScore;
    this->iw             = p.iw;
    this->pw             = p.pw;
    this->gw             = p.gw;
	this->lw             = p.lw;
	this->nw             = p.nw;
	this->localK         = p.localK;
	this->enableGLNPSO   = p.enableGLNPSO;
    this->convT          = p.convT;
    this->getScore       = p.getScore;
    this->gBestIteration = p.gBestIteration;
	this->obj            = obj;

    for (int i = 0; i < dim; i++) {
        this->rangeL[i]     = p.rangeL[i];
        this->rangeU[i]     = p.rangeU[i];
        this->rangeInter[i] = p.rangeInter[i];
        this->gBest[i]      = p.gBest[i];
		this->nBest[i]      = p.nBest[i];
    }

	// initialize localContainer
	localContainer.resize(particleNum);

    return *this;
}

PSOSolver::~PSOSolver(void)
{
    if (this->rangeL) {
        delete [] this->rangeL;
        this->rangeL = NULL;
    }
    if (this->rangeU) {
        delete [] this->rangeU;
        this->rangeU = NULL;
    }
    if (this->rangeInter) {
        delete [] this->rangeInter;
        this->rangeInter = NULL;
    }
    if (this->gBest) {
        delete [] this->gBest;
        this->gBest = NULL;
    }
	if (this->nBest) {
		delete [] this->nBest;
		this->nBest = NULL;
	}
}

void PSOSolver::init() {
    // allocate particle container
    particles.clear();
    particles = vector<Particle>(particleNum, Particle(dim));

    // uniform random parameter between range
    for (int d = 0; d < dim; d++) {
        for (int i = 0; i < particleNum; i++) {
            // random position parameter (L~U)
            particles[i].pos[d] = (rangeInter[d] * random()) + rangeL[d];
            // random velocity parameter (-|U-L| ~ |U-L|), known as velocity inertia
            particles[i].vec[d] = (2*rangeInter[d] * random()) - rangeInter[d];
            // set pBest as initial position
            particles[i].pBest[d] = particles[i].pos[d];
        }
    }
    
    // get particle score
	#pragma omp parallel for
    for (int i = 0; i < particleNum; i++) {
        Particle &p = particles[i];
        // get current score
        p.score = getScore(p, obj);
        // set initial best score as current score
        p.pBestScore = p.score;
    }

	// get gBest
	updateGbest();
}

void PSOSolver::run() {
	double dispersionIDX, velocityIDX;
    for (iteration = 0; iteration < maxIteration; iteration++) {
		// move particle
		moveParticle();

        // update particle pbest
        updatePbest();

        // update global gbest
        updateGbest();

        // show particle image in parameter space
        // showParticles();

        // convergence check
        dispersionIDX = getDispersionIDX();
        velocityIDX   = getVelocityIDX();
		if (dispersionIDX < convT && velocityIDX < convT) {
			break;
		}

        // linear interia weighting adjustment
		iw = max(iw - 1.0/maxIteration, 0.4);
    } // end of iteration
}

void PSOSolver::runFullSearch(const int stepNum) {
	particleNum = 0;
	particles.clear();

	/*
	double *step = new step [dim];;
	for (int i = 0; i < dim; i++) {
		step[i] = rangeInter[i] / stepNum;
	}

	Particle p(dim);
	for (int s = 0; s < stepNum; s++) {
		for (int d = 0; d < dim; d++) {
			p.pos[d] = rangeL[d] + step[d] * s;
			p.pBest[d] = p.pos[d];
		}
	}
	*/

	double step1 = rangeInter[0] / stepNum;
	double step2 = rangeInter[1] / stepNum;
	double step3 = rangeInter[2] / stepNum;

	for (double pos1 = rangeL[0]; pos1 < rangeU[0]; pos1+=step1) {
		for (double pos2 = rangeL[1]; pos2 < rangeU[1]; pos2+=step2) {
			for (double pos3 = rangeL[2]; pos3 < rangeU[2]; pos3+=step3) {
				Particle p(dim);
				p.pos[0] = pos1;
				p.pos[1] = pos2;
				p.pos[2] = pos3;
				p.pBest[0] = pos1;
				p.pBest[1] = pos2;
				p.pBest[2] = pos3;
				p.score = getScore(p, obj);
				p.pBestScore = p.score;

				particles.push_back(p);
				particleNum++;
			}
		}
	}
	
	updateGbest();

	// delete [] step;
}

inline void PSOSolver::moveParticle() {
	double pVecW, gVecW, lVecW, nVecW;

	for (int j = 0; j < particleNum; j++) {
        // current particle
        Particle &p = particles[j];

		// get weighting noise w * [0 ~ 1]
		// pBest, gBest, lBest, nBest weighting noise
		pVecW = pw * random();
        gVecW = gw * random();

		if (enableGLNPSO) {
			// get lBest and nBest weighting
			lVecW = lw * random();
			nVecW = nw * random();
            // get lBest and nBest
			lBest = getLocalBest(j);
			getNearBest(j);
		}
			
        // new velocity and position
        for (int d = 0; d < dim; d++) {
			// update velocity (GLN-PSO)
			if (enableGLNPSO) {
				p.vec[d] =  iw*p.vec[d]                 +
							pVecW*(p.pBest[d]-p.pos[d]) + 
							gVecW*(gBest[d]-p.pos[d])   +
					        lVecW*(lBest[d]-p.pos[d])   +
						    nVecW*(nBest[d]-p.pos[d]);
			} else { // update velocity (original PSO) 
				p.vec[d] =  iw*p.vec[d]                 +
							pVecW*(p.pBest[d]-p.pos[d]) + 
							gVecW*(gBest[d]-p.pos[d])   ;
			}

            // update position
            p.pos[d] += p.vec[d];

            // parameter bound check
            if (p.pos[d] > rangeU[d]) p.pos[d] = rangeU[d];
            if (p.pos[d] < rangeL[d]) p.pos[d] = rangeL[d];
        }    
    } // end of movs particles
}

inline void PSOSolver::updatePbest() {
	#pragma omp parallel for
    for (int j = 0; j < particleNum; j++) {
        // current particle
        Particle &p = particles[j];

        p.score = getScore(p, obj);

        // update pBest
        if (p.score < p.pBestScore) {
            p.pBestScore = p.score;
            for (int d = 0; d < dim; d++) {
                p.pBest[d] = p.pos[d];
            }
        }
    } // end of update particles
}

inline void PSOSolver::updateGbest() {
    for (int j = 0; j < particleNum; j++) {
        // current particle
        const Particle &p = particles[j];

        if (p.pBestScore < gBestScore) {
            gBestScore = p.pBestScore;
            gBestIteration = iteration;

            for (int d = 0; d < dim; d++) {
                gBest[d] = p.pBest[d];
            }
        }
    }
}

inline const double* PSOSolver::getLocalBest(const int pidx) const {
	// current pBest position
	const double *pos = particles[pidx].pBest;

	// get Euclidean distence from pBest to current pBest position
	for (int i = 0; i < particleNum; i++) {
		const Particle &p = particles[i];
		LocalParticle &localP = localContainer[i];
		localP.dist = 0;
		localP.p = &p;

		if (i == pidx) { // skip self
			localP.dist = DBL_MAX;
			continue;
		}
		
		for (int j = 0; j < dim; j++) {
			localP.dist += (pos[j] - p.pBest[j])*(pos[j] - p.pBest[j]);
		}
	}

	// sort by distance
	sort(localContainer.begin(), localContainer.end(), sortLocalParticle);

	// find the minimum fitness pbest as lbest from localK nearest neighbors
	double minFitness = DBL_MAX;
	const double *lBest = pos;
	for (int k = 0; k < localK; k++) {
		const LocalParticle &localP = localContainer[k];
		const Particle &p           = *localP.p;
		if (p.pBestScore < minFitness) {
			minFitness = p.pBestScore;
			lBest = p.pBest;
		}
	}

	return lBest;
}

inline void PSOSolver::getNearBest(const int pidx) const {
	// current fitness
	const double fitness = particles[pidx].score;
	// current position
	const double *pos     = particles[pidx].pos;

	double FDR;
	double maxFDR;

	for (int d = 0; d < dim; d++) { // loop dimension
		maxFDR = -DBL_MAX;
		for (int i = 0; i < particleNum; i++) { // loop particle
			if (i == pidx) continue; // skip current particle
			const Particle &p = particles[i];

			FDR = (fitness - p.pBestScore) / abs(pos[d] - p.pBest[d]);

			if (FDR > maxFDR) {
				maxFDR = FDR;
				nBest[d] = p.pBest[d];
			}
		}
	}
}

inline double PSOSolver::getDispersionIDX() const {
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

inline double PSOSolver::getVelocityIDX() const {
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

void PSOSolver::setGLNPSO(const bool flag) {
	enableGLNPSO = flag;
}

void PSOSolver::setConvergenceThreshold(const double threshold) {
    this->convT = threshold;
}

void PSOSolver::setParticle(const int pidx, const double *param) {
	Particle &p = particles[pidx];
	for (int i = 0; i < dim; i++) {
		p.pos[i]   = param[i];
		p.pBest[i] = param[i];
	}

	// get current score
    p.score = getScore(p, obj);
    // set initial best score as current score
    p.pBestScore = p.score;
	// get gBest
	updateGbest();
}