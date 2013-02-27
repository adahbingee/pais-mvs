#include "psosolver.h"
#include "../io/logmanager.h"
#include <math.h>

bool PsoSolver::sortLocalParticle (const LocalParticle &i, const LocalParticle &j) {
    return (i.dist < j.dist); 
}

PsoSolver::PsoSolver(const int dim, 
			      const double *rangeL, const double *rangeU, 
				  double (*getFitness)(const Particle &p, void *obj), 
				  void *obj,
				  int maxIteration, int particleNum,
				  double dDegPhi, // added by Chaody, for defining degree of phi
				  bool bRandomInit, // added by Chaody, for random or even init. particles distribution
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

	// added by Chaody, for defining degree of phi
	this->dDegPhi = dDegPhi;

	//added by Chaody, for full output, 2013.01.28
	this->depthIteration		 = new double[maxIteration];
	this->normal1Iteration		 = new double[maxIteration];
	this->normal2Iteration		 = new double[maxIteration];
	this->fitnessIteration		 = new double[maxIteration];
	
	for (int i = 0; i < maxIteration; i++) {
		this->depthIteration[i]     = 0;
		this->normal1Iteration[i]     = 0;
		this->normal2Iteration[i]     = 0;
		this->fitnessIteration[i]     = 0;
	}

	this->gBest          = NULL;
	this->gBestFitness   = DBL_MAX;
	this->gBestIteration = -1;

	for (int i = 0; i < dim; i++) {
		this->rangeL[i]     = rangeL[i];
		this->rangeU[i]     = rangeU[i];
		this->rangeInter[i] = rangeU[i] - rangeL[i];
	}

	setRandomSeed();

	if (bRandomInit)
		initParticlesRand();
	else
		initParticlesEven();
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

	//added by Chaody, for full output, 2013.01.28
	if (depthIteration) {
		delete [] depthIteration;
		depthIteration = NULL;
	}
	if (normal1Iteration) {
		delete [] normal1Iteration;
		normal1Iteration = NULL;
	}
	if (normal2Iteration) {
		delete [] normal2Iteration;
		normal2Iteration = NULL;
	}
	if (fitnessIteration) {
		delete [] fitnessIteration;
		fitnessIteration = NULL;
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

// set initial particle position and velocity in random distribution
void PsoSolver::initParticlesRand() {
	// allocate particle container
	particles.clear();
	particles.resize(particleNum, Particle(dim));

	// uniform random parameter between range
	for (int i = 0; i < particleNum; i++) {
		for (int d = 0; d < dim; d++) {
			// random position parameter (L~U)
            particles[i].pos[d] = (rangeInter[d] * random()) + rangeL[d];
            // random velocity parameter (-|U-L| ~ |U-L|), known as velocity inertia
            particles[i].vec[d] = (2.0 * rangeInter[d] * random()) - rangeInter[d];
            // set pBest as initial position
            particles[i].pBest[d] = particles[i].pos[d];
		}
		LogManager::log("00\t%03d\t%f\t%f\t%f\t0.0\t0.0", i+1, particles[i].pos[0], particles[i].pos[1], particles[i].pos[2]);
	}
}


// set initial particle position and velocity in even distribution
void PsoSolver::initParticlesEven() {
	// allocate particle container
	particles.clear();
	particles.resize(particleNum, Particle(dim));

	int iCnt = 0;
	// uniform & grid parameter between range

	int iOneThirdParticleNum = (int)(particleNum/3);
	
	////////////////////////////////////////////////////////////
	// Using double ring or not, can be extented in the future
	////////////////////////////////////////////////////////////
	//for (int i = 0; i < particleNum; i++) {
	//	if (i < iOneThirdParticleNum) {   // inner ring
	//		particles[i].pos[0] = (rangeInter[0] * ((double)i/(double)iOneThirdParticleNum)) + rangeL[0];
	//		particles[i].pos[1] = rangeInter[1] / 2 + rangeL[1];
	//		//particles[i].pos[0] = (rangeInter[0]/4.0)+rangeL[0];
	//		//particles[i].pos[1] = (rangeInter[1] * ((double)i/(double)iOneThirdParticleNum)) + rangeL[1];
	//		particles[i].pos[2] = (rangeInter[2] * ((double)i/(double)particleNum)) + rangeL[2];
	//	}
	//	else { // outer ring
	//		particles[i].pos[0] = (rangeInter[0] * ((double)(i-iOneThirdParticleNum)/(double)(iOneThirdParticleNum*2))) + rangeL[0];
	//		particles[i].pos[1] = rangeInter[1] / 4 + rangeL[1];
	//		//particles[i].pos[0] = (rangeInter[0]/2.0)+rangeL[0];
	//		//particles[i].pos[1] = (rangeInter[1] * ((double)(i-iOneThirdParticleNum)/(double)(iOneThirdParticleNum*2))) + rangeL[1];
	//		particles[i].pos[2] = (rangeInter[2] * ((double)i/(double)particleNum)) + rangeL[2];
	//	}
	//	for (int d = 0; d < dim; d++) {
	//		// random velocity parameter (-|U-L| ~ |U-L|), known as velocity inertia
	//		particles[i].vec[d] = (2.0 * rangeInter[d] * random()) - rangeInter[d];
	//		// set pBest as initial position
	//		particles[i].pBest[d] = particles[i].pos[d];
	//	}

	//	LogManager::log("00\t%03d\t%f\t%f\t%f\t0.0", i+1, particles[i].pos[0], particles[i].pos[1], particles[i].pos[2]);
	//}
	////////////////////////////////////////////////////////////

	Vec3d refCamNormal, particleVec;
	Vec2d refCamNormalS, particleVecS;
	refCamNormalS[0] = rangeInter[0]/2 + rangeL[0];	
	refCamNormalS[1] = rangeInter[1]/2 + rangeL[1];
	Utility::spherical2Normal(refCamNormalS, refCamNormal);
	Utility::normal2Spherical(refCamNormal, refCamNormalS);
	
	// setup rotation matrix Ry and Rz
	Mat_<double> Ry(3,3), Rz(3,3), Rxz(3,3); 
	Rxz.at<double>(0, 0) = 0; Rxz.at<double>(0, 1) = 0; Rxz.at<double>(0, 2) = 1;
    Rxz.at<double>(1, 0) = 0; Rxz.at<double>(1, 1) = 1; Rxz.at<double>(1, 2) = 0;
    Rxz.at<double>(2, 0) = 1; Rxz.at<double>(2, 1) = 0; Rxz.at<double>(2, 2) = 0;

	double dRy = -(refCamNormalS[1]);
	Ry.at<double>(0, 0) = cos(dRy); Ry.at<double>(0, 1) = 0; Ry.at<double>(0, 2) = sin(dRy);
    Ry.at<double>(1, 0) = 0;        Ry.at<double>(1, 1) = 1; Ry.at<double>(1, 2) = 0;
    Ry.at<double>(2, 0) = -sin(dRy);Ry.at<double>(2, 1) = 0; Ry.at<double>(2, 2) = cos(dRy);

	double dRz = refCamNormalS[0];
	Rz.at<double>(0, 0) = cos(dRz); Rz.at<double>(0, 1) = -sin(dRz); Rz.at<double>(0, 2) = 0;
    Rz.at<double>(1, 0) = sin(dRz); Rz.at<double>(1, 1) = cos(dRz);  Rz.at<double>(1, 2) = 0;
    Rz.at<double>(2, 0) = 0;        Rz.at<double>(2, 1) = 0;         Rz.at<double>(2, 2) = 1;

	for (int i = 0; i < particleNum; i++) {
		particles[i].pos[0] = (2*M_PI) * ((double)i/(double)particleNum);
		particles[i].pos[1] = (90-dDegPhi) * (M_PI/180);
		particles[i].pos[2] = (rangeInter[2] * ((double)i/(double)particleNum)) + rangeL[2];

		// transformation (rotation in Cartesian coordinate)
		// 先轉至 Cartesian coordinate
		particleVecS[0] = particles[i].pos[0];         particleVecS[1] = particles[i].pos[1]; 
		Utility::spherical2Normal(particleVecS, particleVec);

		// rotation
		Mat_<double> tmpVec(3,1); 
		tmpVec.at<double>(0, 0) = particleVec[0]; 
		tmpVec.at<double>(1, 0) = particleVec[1]; 
		tmpVec.at<double>(2, 0) = particleVec[2]; 

		tmpVec = Rz * Ry * Rxz * tmpVec;

		particleVec[0] = tmpVec.at<double>(0, 0);
		particleVec[1] = tmpVec.at<double>(1, 0);
		particleVec[2] = tmpVec.at<double>(2, 0);

		// 再轉回 spherical coordinate
		Utility::normal2Spherical(particleVec, particleVecS);
		particles[i].pos[0] = particleVecS[0];         particles[i].pos[1] = particleVecS[1]; 

		for (int d = 0; d < dim; d++) {
			// random velocity parameter (-|U-L| ~ |U-L|), known as velocity inertia
			particles[i].vec[d] = (2.0 * rangeInter[d] * random()) - rangeInter[d];
			// set pBest as initial position
			particles[i].pBest[d] = particles[i].pos[d];
		}

		LogManager::log("00\t%03d\t%f\t%f\t%f\t0.0\t0.0", i+1, particles[i].pos[0], particles[i].pos[1], particles[i].pos[2]);
	}
}

// not grid distribution
//void PsoSolver::initParticlesFix() {
//	// allocate particle container
//	particles.clear();
//	particles.resize(particleNum, Particle(dim));
//
//	//LogManager::log("ite\tp\tn1\tn2\td");
//	// uniform random parameter between range
//	for (int i = 0; i < particleNum; i++) {
//		for (int d = 0; d < dim; d++) {
//			// random position parameter (L~U)
//            //particles[i].pos[d] = (rangeInter[d] * random()) + rangeL[d];
//			// fixed position parameter (L~U)
//            particles[i].pos[d] = (rangeInter[d] * ((double)i/(double)particleNum)) + rangeL[d];
//            // random velocity parameter (-|U-L| ~ |U-L|), known as velocity inertia
//            particles[i].vec[d] = (2.0 * rangeInter[d] * random()) - rangeInter[d];
//            // set pBest as initial position
//            particles[i].pBest[d] = particles[i].pos[d];
//		}
//		LogManager::log("00\t%02d\t%f\t%f\t%f", i+1, particles[i].pos[0], particles[i].pos[1], particles[i].pos[2]);
//	}
//}



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
	int iTmpCnt; //Chaody

	for (int j = 0; j < particleNum; j++) {
        // current particle
        const Particle &p = particles[j];

		if (p.pBestFitness <= gBestFitness) {
            gBestFitness   = p.pBestFitness;
			gBest          = p.pBest;
			gBestIteration = iteration;
			iTmpCnt = j; //Chaody
			// printf("update: %f\n", gBestFitness);
        }
    }
	
	LogManager::log("%02d\t888\t%f\t%f\t%f\t%f\t%f", 
		iteration+1, particles[iTmpCnt].pos[0], particles[iTmpCnt].pos[1], particles[iTmpCnt].pos[2], gBestFitness, getDispersionIDX());
}

// for better PSO log output, by Chaody
void PsoSolver::updateGbest_init() {
	int iTmpCnt; //Chaody

	for (int j = 0; j < particleNum; j++) {
        // current particle
        const Particle &p = particles[j];

		if (p.pBestFitness <= gBestFitness) {
            gBestFitness   = p.pBestFitness;
			gBest          = p.pBest;
			gBestIteration = iteration;
			iTmpCnt = j; //Chaody
        }
    }
	
	LogManager::log("00\t888\t%f\t%f\t%f\t%f\t%f", 
		particles[iTmpCnt].pos[0], particles[iTmpCnt].pos[1], particles[iTmpCnt].pos[2], gBestFitness, getDispersionIDX());
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
	gBest        = particles[0].pBest;
	gBestFitness = particles[0].pBestFitness;
	//updateGbest();
	updateGbest_init(); //Chaody, for better log output

	for (iteration = 0; iteration < maxIteration; iteration++) {

		//if (getDispersionIDX() < convergenceThreshold && getVelocityIDX() < convergenceThreshold) {
		if (getDispersionIDX() < convergenceThreshold) {
			break;
		}

		moveParticles();
		logParticles(); // 輸出 particle information
		updateFitness();
		updateGbest();

		//added by Chaody, for full output, 2013.01.28
		normal1Iteration[iteration] = gBest[0];
		normal2Iteration[iteration] = gBest[1];
		depthIteration[iteration] = gBest[2];
		fitnessIteration[iteration] = gBestFitness;
		
			
		// linear interia weighting adjustment
		iw = max(iw - 1.0/maxIteration, minIw);
	} // end of iteration
}


void PsoSolver::logParticles() {
	for (int i = 0; i < particleNum; i++) {
		if (particles[i].fitness > 999)
			LogManager::log("%02d\t%03d\t%f\t%f\t%f\t999\t0.0", 
				iteration+1, i+1, particles[i].pos[0], particles[i].pos[1], particles[i].pos[2]);
		else
			LogManager::log("%02d\t%03d\t%f\t%f\t%f\t%f\t%f", 
				iteration+1, i+1, particles[i].pos[0], particles[i].pos[1], particles[i].pos[2], particles[i].fitness, getDispersionIDX());

	}
}