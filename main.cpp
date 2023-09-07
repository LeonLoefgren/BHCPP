#include "particle.h"
#include "node.h"
#include <crtdbg.h>
#include <iostream>
#include <chrono>
#include <string>
#include <fstream>

// Docstrings under main. 
void buildTree(Node* fromNode); 
void sumExternalForces(Particle* p, Node* fromNode, double epsilon, double G, double Theta);
double getMaxSide(Particle** particles, int numParticles);
void writeFrame(int frameNum, Particle** particles, int numParticles);




int main() {
	// Setup. 
	auto start = std::chrono::steady_clock::now();
	std::mt19937 generator(static_cast<unsigned int>(std::time(nullptr)));
	std::uniform_real_distribution<double> distribution(0.0, 1.0);
	Vector3d origin;
	origin << 0, 0, 0;
	int numFrames = 2000;
	int numParticles = 100000;
	double dt = 0.1;
	double G = 20;
	double epsilon = 0.1;
	double Theta = 0.9;

	// Instantiate all particles. 
	Particle** particles = new Particle * [numParticles];
	for (int i = 0; i < numParticles; i++) {
		particles[i] = new Particle(2, generator, distribution, false, 9000, dt);
	}
	

	// Frame loop. 
	for (int i = 0; i < numFrames; i++) {
		std::cout << "Frame " << i << ".\n";

		// Build octree. 
		Node root = Node(origin, getMaxSide(particles, numParticles));
		root.allocParticles(numParticles);
		for (int j = 0; j < numParticles; j++) {
			root.addParticle(*particles[j]);
		}
		root.updateMassAttribs();
		buildTree(&root);

		// Gravity. 
		for (int j = 0; j < numParticles; j++) {
			sumExternalForces(particles[j], &root, epsilon, G, Theta);
			particles[j]->step(false, dt);
			particles[j]->setExtForce(origin); // Very important. 
		}


		writeFrame(i, particles, numParticles);
		root.deallocate();
	}


	// Good manners.  
	for (int i = 0; i < numParticles; i++) {
		if (particles[i] != nullptr) {
			delete particles[i];
			particles[i] = nullptr;
		}
	}
	delete[] particles;
	particles = nullptr;
	
	// Finish. 
	// _CrtDumpMemoryLeaks();
	auto end = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
	std::cout << "Time = " << elapsedTime << " ms.\n";
	std::cout << "Program finished.";
	return 0;
}




/*
When called on the root node (if the root node has been properly initialized including its contents)
recursively generates the octree. 
*/
void buildTree(Node* fromNode) {
	if (fromNode->numParticles() <= 1) {
		return;
	}
	fromNode->subdivide();
	for (int i = 0; i < 8; i++) {
		buildTree(fromNode->children()[i]);
	}
}

/*
Traverses the octree in a DFS manner and sums the external forces acting upon a particular particle 
instance. 
*/
void sumExternalForces(Particle* p, Node* fromNode, double epsilon, double G, double Theta) {
	if (!(fromNode->hasChildren())) {
		if (fromNode->numParticles() == 0) {
			return;
		}
		else {
			if (fromNode->particles()[0] == p) {
				return;
			}
			else {
				Vector3d posVec = p->pos() - fromNode->particles()[0]->pos();
				Vector3d Fdenom = (-1) * G * p->mass() * fromNode->particles()[0]->mass() * posVec;
				double Fnum = pow(pow(posVec.norm(), 2) + pow(epsilon, 2), 3 / 2);
				Vector3d F = Fdenom / Fnum;
				p->addExtForce(F);
			}
		}
	}
	else {
		Vector3d posVec = p->pos() - fromNode->com();
		double posVecMag = posVec.norm();
		double theta = fromNode->side() / posVecMag;
		if (theta < Theta) {
			Vector3d Fdenom = (-1) * G * p->mass() * fromNode->totalMass() * posVec;
			double Fnum = pow(pow(posVec.norm(), 2) + pow(epsilon, 2), 3 / 2);
			Vector3d F = Fdenom / Fnum;
			p->addExtForce(F);
		}
		else {
			for (int i = 0; i < 8; i++) {
				sumExternalForces(p, fromNode->children()[i], epsilon, G, Theta);
			}
		}
	}
}

/*
Returns the appropriate side-length of the root nodes bounding box depending on the
position of the particle instance furthest away from the origin. 
*/
double getMaxSide(Particle** particles, int numParticles) {
	double maxX = 0;
	double maxY = 0;
	double maxZ = 0;
	for (int i = 0; i < numParticles; i++) {
		if (fabs(particles[i]->pos()[0]) > maxX) {
			maxX = fabs(particles[i]->pos()[0]);
		}
		if (fabs(particles[i]->pos()[1]) > maxY) {
			maxY = fabs(particles[i]->pos()[1]);
		}
		if (fabs(particles[i]->pos()[2]) > maxZ) {
			maxZ = fabs(particles[i]->pos()[2]);
		}
	}
	double maxC = 0;
	if (maxX > maxC) {
		maxC = maxX;
	}
	if (maxY > maxC) {
		maxC = maxY;
	}
	if (maxZ > maxC) {
		maxC = maxZ;
	}
	return 2 * maxC;
}

/*
Writes the positional data of each particle instance at a particular frame.
The fileformat created is .vtk and can therefore be immediately read using Paraview. 
*/
void writeFrame(int frameNum, Particle** particles, int numParticles) {
	std::string filename = "Data/datafile_" + std::to_string(frameNum) + ".vtk";
	std::ofstream activeFile(filename.c_str());
	activeFile << "# vtk DataFile Version 5.1\n"
		<< "vtk output\n"
		<< "ASCII\n"
		<< "DATASET POLYDATA\n"
		<< "POINTS " << std::to_string(numParticles) << " double" << "\n";

	for (int i = 0; i < numParticles; i++) {
		activeFile << particles[i]->pos()[0] << " " << particles[i]->pos()[1] << " " << particles[i]->pos()[2] << "\n";
	}
	activeFile.close();
}


