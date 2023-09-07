#include "node.h"
#include <iostream>

/*
Instantiates a node object without children and without any owned particles. 
*/
Node::Node(Vector3d center, double side) {
	center_ = com_ = center;
	totalMass_ = 0;
	side_ = side;
	b_hasChildren_ = false;
	for (int i = 0; i < 8; i++) {
		children_[i] = nullptr;
	}
	numParticles_ = 0;
	particles_ = nullptr;
}

Node::~Node() {
	if (numParticles_ > 0) {
		delete[] particles_;
		particles_ = nullptr;
		numParticles_ = 0;
	}
}

/*
Getters. 
*/
Vector3d Node::center() { return center_; }
Vector3d Node::com() { return com_; }
double Node::totalMass() { return totalMass_; }
double Node::side() { return side_; }
bool Node::hasChildren() { return b_hasChildren_; };
Node** Node::children() { return children_; }
int Node::numParticles() { return numParticles_; }
Particle** Node::particles() { return particles_; }

/*
Allocates memory for the dynamic particles_ array. 
*/
void Node::allocParticles(int size) {
	particles_ = new Particle * [size];
	for (int i = 0; i < size; i++) {
		particles_[i] = nullptr;
	}
}

/*
Adds a new Particle* to the particles_ array.  
*/
void Node::addParticle(Particle& p) {
	particles_[numParticles_] = &p;
	numParticles_++;
}

/*
Updates the com_ and totalMass_ attributes. 
*/
void Node::updateMassAttribs() {
	assert(numParticles_ > 0);
	double totalMass = 0;
	Vector3d com;
	com << 0, 0, 0;

	for (int i = 0; i < numParticles_; i++) {
		totalMass += particles_[i]->mass();
		com += particles_[i]->pos() * particles_[i]->mass();
	}
	com /= totalMass;
	totalMass_ = totalMass;
	com_ = com;
}

/*
Returns true if point is within the nodes bounding box, else false. 
*/
bool Node::inBounds(Vector3d point) {
	double maxX, minX, maxY, minY, maxZ, minZ;
	maxX = center_(0) + side_ / 2;
	minX = center_(0) - side_ / 2;
	maxY = center_(1) + side_ / 2;
	minY = center_(1) - side_ / 2;
	maxZ = center_(2) + side_ / 2;
	minZ = center_(2) - side_ / 2;

	bool b_X, b_Y, b_Z;
	b_X = b_Y = b_Z = false;

	if ((minX <= point(0)) && (point(0) <= maxX)) { b_X = true; }
	if ((minY <= point(1)) && (point(1) <= maxY)) { b_Y = true; }
	if ((minZ <= point(2)) && (point(2) <= maxZ)) { b_Z = true; }

	return (b_X && b_Y && b_Z);
}

/*
Instantiates all 8 children of the node including their respective contained
particles. 
*/
void Node::subdivide() {
	assert(numParticles_ > 1);
	double childSide = side_ / 2;
	Vector3d childCenters[8];
	double s = childSide / 2;
	childCenters[0] << -s, -s, s;
	childCenters[1] << s, -s, s;
	childCenters[2] << -s, -s, -s;
	childCenters[3] << s, -s, -s;
	childCenters[4] << -s, s, s;
	childCenters[5] << s, s, s;
	childCenters[6] << -s, s, -s;
	childCenters[7] << s, s, -s;

	int numParticles[8] = { 0,0,0,0,0,0,0,0 };

	for (int i = 0; i < 8; i++) {
		children_[i] = new Node(childCenters[i], childSide);
	}

	for (int i = 0; i < numParticles_; i++) {
		for (int j = 0; j < 8; j++) {
			if (children_[j]->inBounds(particles_[i]->pos())) {
				numParticles[j]++;
			}
		}
	}

	for (int i = 0; i < 8; i++) {
		children_[i]->allocParticles(numParticles[i]);
	}

	for (int i = 0; i < numParticles_; i++) {
		for (int j = 0; j < 8; j++) {
			if (children_[j]->inBounds(particles_[i]->pos())) {
				children_[j]->addParticle(*particles_[i]);
			}
		}
	}

	for (int i = 0; i < 8; i++) {
		if (children_[i]->numParticles() > 0) {
			children_[i]->updateMassAttribs();
		}
	}

	b_hasChildren_ = true;
}

/*
(When called on the root node) Recursively deallocates all dynamically
allocated memory pertaining to the octree. 
*/
void Node::deallocate() {
	if (!b_hasChildren_) {
		return;
	}
	for (int i = 0; i < 8; i++) {
		children_[i]->deallocate();
		delete children_[i];
		children_[i] = nullptr;
	}
}
