#include "particle.h"

/*
Random number generator used in one of the constructors. 
@param[out]: A random number x \in [-range, range]. 
*/
double randomNum(std::mt19937& generator, std::uniform_real_distribution<double>& distribution, double range) {
	return distribution(generator) * (2 * range) - range;
}


/*
Intantiates object at a particular position and with 
a particular velocity. Zeroes extForce_. 
*/
Particle::Particle(double mass, Vector3d prevPos, Vector3d vel, double dt) {
	mass_ = mass;
	prevPos_ = prevPos;
	vel_ = vel;
	pos_ = prevPos_ + vel_ * dt;
	extForce_ << 0, 0, 0;
}

/*
Instantiates object with a random position and (if (b_randomVel)) a random velocity. 
The range of the random velocity is the range of the position divided by 10. 
Zeroes extForce_. 
*/
Particle::Particle(double mass, std::mt19937& generator, std::uniform_real_distribution<double>& distributition, bool b_randomVel, double range, double dt) {
	mass_ = mass;
	double pos[3];
	double vel[3];
	for (int i = 0; i < 3; i++) {
		pos[i] = randomNum(generator, distributition, range);
		if (b_randomVel) {
			vel[i] = randomNum(generator, distributition, range / 10);
		}
		else {
			vel[i] = 0;
		}
	}
	prevPos_ << pos[0], pos[1], pos[2];
	vel_ << vel[0], vel[1], vel[2];
	pos_ = prevPos_ + vel_ * dt;
	extForce_ << 0, 0, 0;
}

/*
Getters and setters. 
*/
double Particle::mass() { return mass_; }
Vector3d Particle::pos() { return pos_; }
Vector3d Particle::vel() { return vel_; }
void Particle::setExtForce(Vector3d force) { extForce_ = force; }
void Particle::addExtForce(Vector3d force) { extForce_ += force; }

/*
Uses basic Störmer-Verlet integration to step forward one timestep. 
If (b_updateVel), the vel_ attribute will be continously updated during runtime. 
*/
void Particle::step(bool b_updateVel, double dt) {
	Vector3d nextPos = 2 * pos_ - prevPos_ + (extForce_ / mass_) * pow(dt, 2);
	prevPos_ = pos_;
	pos_ = nextPos;
	if (b_updateVel) {
		vel_ = (pos_ - prevPos_) / dt;
	}
}