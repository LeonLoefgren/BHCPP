#pragma once
#include <random>
#include "Eigen/Dense"

using Eigen::Vector3d;

/*
Instances of this class represent physical bodies in the simulation. 
@attrib mass_: the mass of a particular particle.
@attrib prevPos_: the position of the particle at the previous timestep. 
@attrib pos_: the position at the current timestep. 
@attrib vel_: the velocity of the particle (if chosen to be updated during runtime). 
			  If not updated during runtime this holds significance only on
			  instantiation and is thereafter completely inaccurate. 
@attrib extForce_: the total external force which acts upon the particle at the 
				   current timestep. 

See source code for docstrings. 
*/
class Particle{
private:
	double mass_;
	Vector3d prevPos_, pos_, vel_, extForce_;

public:
	Particle(double mass, Vector3d prevPos_, Vector3d vel_, double dt); 
	Particle(double mass, std::mt19937& generator, std::uniform_real_distribution<double>& distributition, bool b_randomVel, double range, double dt);

	double mass();
	Vector3d pos();
	Vector3d vel();
	void setExtForce(Vector3d force);

	void addExtForce(Vector3d force);
	void step(bool b_updateVel, double dt);
};

