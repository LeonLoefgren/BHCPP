#pragma once
#include "particle.h"
#include <cassert>

/*
Instances of this class represent nodes in the octtree. 
@attrib center_: the center of the nodes bounding box. 
@attrib com_: the center of mass of all particles contained within the node. 
@attrib totalMass_: the total mass of all particles contained within the node.
@attrib side_: the length of the bounding box's side. 
@attrib b_hasChildren_: true if the node has children. 
@attrib children_: an array of Node pointers. Each pointer points to one of the child-nodes.
@attrib numParticles_: the number of particles contained within the node. 
@attrib particles_: a pointer to a dynamic array containing pointers to each particle instance
					contained within the node. 

See source code for docstrings. 
*/
class Node{
private:
	Vector3d center_, com_;
	double totalMass_, side_;
	bool b_hasChildren_;
	Node* children_[8];
	int numParticles_;
	Particle** particles_;

public:
	Node(Vector3d center, double side);
	~Node();

	Vector3d center();
	Vector3d com();
	double totalMass();
	double side();
	bool hasChildren();
	Node** children();
	int numParticles();
	Particle** particles();

	void allocParticles(int size);
	void addParticle(Particle& p);
	void updateMassAttribs();
	bool inBounds(Vector3d point);
	void subdivide(); 
	void deallocate();
};

