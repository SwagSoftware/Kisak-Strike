#ifndef CTODESOLVER_H
#define CTODESOLVER_H

// this code was adapted from an ode solver by Brian Mirtich

#include <assert.h>
#include<hk_physics/types/types.h>

#define ODE_INITIAL_STATE_SIZE	1024


typedef void (*dydt_function)(hk_real t, const hk_real statev[],  hk_real delta_statev[], void *client_data );


// virtual base-class on which concrete ode solver algorithms are built
// NOTE: don't derive any classes from any classes derived from this one.
class OdeSolver 
{
public:
  
	OdeSolver();
	virtual ~OdeSolver();

	// compute a step.  current state in t0, state after time step t0 -> t1 is put in y1
	virtual void calc_step(hk_real y0[], hk_real y1[], unsigned int len, hk_real t0, hk_real t1, dydt_function dydt, void *client_data ) = 0;
  
protected:

  virtual void ode_realloc(int new_size);
	unsigned int state_size;  // limit of ~65,000 right now
	hk_real *dy;  //the derivatives
	hk_real *Iy;
};


// Runga-Kutta order 4
// pretty standard dynamics ODE solver.  Good stability/speed trade-off
// NOTE: don't derive any classes from this one
class OdeRungaKutta4 : public OdeSolver
{
public:

	OdeRungaKutta4();
	virtual ~OdeRungaKutta4();

	void calc_step(hk_real y0[], hk_real y1[], unsigned int len, hk_real t0, hk_real t1, dydt_function dydt, void *client_data);
 
private:

	void ode_realloc(int new_size);

	// work variables
	hk_real *k1;    
	hk_real *k2;
	hk_real *k3;
	hk_real *k4;

};


// the fastest and least stable ODE-solver.
// not recomended unless you REALLY need the speed.
// use mid-point instead if you need speed. 
class OdeEuler : public OdeSolver
{
public:

	OdeEuler();

	void calc_step(hk_real y0[], hk_real y1[], unsigned int len, hk_real t0, hk_real t1, dydt_function dydt, void *client_data);

};


// If you need speed and don't have many forces more complicated than gravity
// or propulsion this is a good one to use.  Better than euler, but not as
// stable as runga-kutta ( much faster though ). 
class OdeMidPoint : public OdeSolver
{
public:

	OdeMidPoint();

	void calc_step(hk_real y0[], hk_real y1[], unsigned int len, hk_real t0, hk_real t1, dydt_function dydt, void *client_data);

};

#endif
