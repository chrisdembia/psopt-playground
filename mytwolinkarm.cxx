//////////////////////////////////////////////////////////////////////////
//////////////////        twolinkarm.cxx        //////////////////////////
//////////////////////////////////////////////////////////////////////////
////////////////           PSOPT  Example             ////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////// Title:                 Two link arm problem      ////////////////
//////// Last modified:         04 January 2009           ////////////////
//////// Reference:             PROPT users guide         ////////////////
//////// (See PSOPT handbook for full reference)          ////////////////
//////////////////////////////////////////////////////////////////////////
////////     Copyright (c) Victor M. Becerra, 2009        ////////////////
//////////////////////////////////////////////////////////////////////////
//////// This is part of the PSOPT software library, which ///////////////
//////// is distributed under the terms of the GNU Lesser ////////////////
//////// General Public License (LGPL)                    ////////////////
//////////////////////////////////////////////////////////////////////////

#include "psopt.h"

//////////////////////////////////////////////////////////////////////////
///////////////////  Define the end point (Mayer) cost function //////////
//////////////////////////////////////////////////////////////////////////

adouble endpoint_cost(adouble* initial_states, adouble* final_states, 
                      adouble* parameters,adouble& t0, adouble& tf, 
                      adouble* xad, int iphase)
{
    return tf;
} 

//////////////////////////////////////////////////////////////////////////
///////////////////  Define the integrand (Lagrange) cost function  //////
//////////////////////////////////////////////////////////////////////////

adouble integrand_cost(adouble* states, adouble* controls, 
                       adouble* parameters, adouble& time, adouble* xad, 
                       int iphase)
{
    return  0.0;
} 

//////////////////////////////////////////////////////////////////////////
///////////////////  Define the DAE's ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

void dae(adouble* derivatives, adouble* path, adouble* states, 
         adouble* controls, adouble* parameters, adouble& time, 
         adouble* xad, int iphase)
{
    adouble g = 9.81;
    adouble L1 = 1;
    adouble L2 = 1;
    adouble m1 = 1;
    adouble m2 = 1;

    adouble theta1 = states[ CINDEX(1) ];
    adouble theta2 = states[ CINDEX(2) ];
    adouble theta1dot = states[ CINDEX(3) ];
    adouble theta2dot = states[ CINDEX(4) ];

    adouble c1 = cos(theta1);
    adouble s2 = sin(theta2);
    adouble c2 = cos(theta2);
    adouble s1 = sin(theta1);

    adouble cd = cos(theta2 - theta1);
    adouble sd = sin(theta2 - theta1);

    theta1dotdot_numerator = m2 * L1 * pow(theta1dot, 2) * sd * cd +
        m2 * g * s2 * cd + m2 * L2 * pow(theat2dot, 2) * sd -
        (m1 + m2) * g * s1;
    theta1dotdot_denominator = (m1 + m2) * L1 - m2 * L1 * pow(cd, 2);

    theta2dotdot_numerator = -m2 * L2 * pow(theta2dot, 2) * sd * cd +
        (m1 + m2) * (g * s1 * cd - L1 * pow(theta1dot, 2) * sd - g * s2);
    theta2dotdot_denominator = (m1 + m2) * L2 - m2 * L2 * pow(cd, 2);

    derivatives[ CINDEX(1) ] = theta1dot;
    derivatives[ CINDEX(2) ] = theta2dot;
    derivatives[ CINDEX(3) ] = theta1dotdot_numerator / theta1dotdot_denominator;
    derivatives[ CINDEX(4) ] = theta2dotdot_numerator / theta2dotdot_denominator;

   /*
    adouble z1 = m2 * L1 * L2 * c2;
    adouble z2 = m2 * L1 * L2 * s2;

    adouble M11 = m1 * pow(L1, 2) + m2 * (pow(L1, 2) + pow(L2, 2)) + 2 * z1;
    adouble M12 = m2 * pow(L2, 2) + z1;
    adouble M22 = m2 * pow(L2, 2);

    DMatrix M(2, 2, M11, M12,
                    M12, M22);

    DMatrix V(2, 1, 
   adouble xdot, ydot, vdot;

   adouble x1 = states[ CINDEX(1) ];
   adouble x2 = states[ CINDEX(2) ];
   adouble x3 = states[ CINDEX(3) ];
   adouble x4 = states[ CINDEX(4) ];

   adouble u1 = controls[ CINDEX(1) ];
   adouble u2 = controls[ CINDEX(2) ];

   adouble num1 =  sin(x3)*( (9.0/4.0)*cos(x3)*x1*x1+2*x2*x2 )
                   + (4.0/3.0)*(u1-u2)-(3.0/2.0)*cos(x3)*u2;

   adouble num2 =  -(sin(x3)*((7.0/2.0)*x1*x1+(9.0/4.0)*cos(x3)*x2*x2)
                   -(7.0/3.0)*u2+(3.0/2.0)*cos(x3)*(u1-u2));

   adouble den  =  31.0/36.0 + 9.0/4.0*pow(sin(x3),2);

   derivatives[ CINDEX(1) ] = num1/den;
   derivatives[ CINDEX(2) ] = num2/den;
   derivatives[ CINDEX(3) ] = x2 - x1;
   derivatives[ CINDEX(4) ] = x1;
   */

}

////////////////////////////////////////////////////////////////////////////
///////////////////  Define the events function ////////////////////////////
////////////////////////////////////////////////////////////////////////////

void events(adouble* e, adouble* initial_states, adouble* final_states, 
            adouble* parameters,adouble& t0, adouble& tf, adouble* xad, 
            int iphase) 
{
   adouble x10 = initial_states[ CINDEX(1) ];
   adouble x20 = initial_states[ CINDEX(2) ];
   adouble x30 = initial_states[ CINDEX(3) ];
   adouble x40 = initial_states[ CINDEX(4) ];
   adouble x1f = final_states[ CINDEX(1) ];
   adouble x2f = final_states[ CINDEX(2) ];
   adouble x3f = final_states[ CINDEX(3) ];
   adouble x4f = final_states[ CINDEX(4) ];

   e[ CINDEX(1) ] = x10;
   e[ CINDEX(2) ] = x20;
   e[ CINDEX(3) ] = x30;
   e[ CINDEX(4) ] = x40;
   e[ CINDEX(5) ] = x1f;
   e[ CINDEX(6) ] = x2f;
   e[ CINDEX(7) ] = x3f;
   e[ CINDEX(8) ] = x4f;

}

///////////////////////////////////////////////////////////////////////////
///////////////////  Define the phase linkages function ///////////////////
///////////////////////////////////////////////////////////////////////////

void linkages( adouble* linkages, adouble* xad)
{
  // No linkages as this is a single phase problem
}


////////////////////////////////////////////////////////////////////////////
///////////////////  Define the main routine ///////////////////////////////
////////////////////////////////////////////////////////////////////////////


int main(void)
{

////////////////////////////////////////////////////////////////////////////
///////////////////  Declare key structures ////////////////////////////////
////////////////////////////////////////////////////////////////////////////

    Alg  algorithm;
    Sol  solution;
    Prob problem;

////////////////////////////////////////////////////////////////////////////
///////////////////  Register problem name  ////////////////////////////////
////////////////////////////////////////////////////////////////////////////

    problem.name        		= "Two link robotic arm";

    problem.outfilename                 = "twolink.txt";

////////////////////////////////////////////////////////////////////////////
////////////  Define problem level constants & do level 1 setup ////////////
////////////////////////////////////////////////////////////////////////////

    problem.nphases   			= 1;
    problem.nlinkages                   = 0;

    psopt_level1_setup(problem);


/////////////////////////////////////////////////////////////////////////////
/////////   Define phase related information & do level 2 setup /////////////
/////////////////////////////////////////////////////////////////////////////

    problem.phases(1).nstates   		= 4;
    problem.phases(1).ncontrols 		= 2;
    problem.phases(1).nevents   		= 8;
    problem.phases(1).npath     		= 0;
    problem.phases(1).nodes                     = 40;  

    psopt_level2_setup(problem, algorithm);
  

////////////////////////////////////////////////////////////////////////////
///////////////////  Declare DMatrix objects to store results //////////////
////////////////////////////////////////////////////////////////////////////

    DMatrix x, u, t;
    DMatrix lambda, H;

////////////////////////////////////////////////////////////////////////////
///////////////////  Enter problem bounds information //////////////////////
////////////////////////////////////////////////////////////////////////////

    problem.phases(1).bounds.lower.states(1) = -2.0;
    problem.phases(1).bounds.lower.states(2) = -2.0;
    problem.phases(1).bounds.lower.states(3) = -2.0;
    problem.phases(1).bounds.lower.states(4) = -2.0;

    problem.phases(1).bounds.upper.states(1) = 2.0;
    problem.phases(1).bounds.upper.states(2) = 2.0;
    problem.phases(1).bounds.upper.states(3) = 2.0;
    problem.phases(1).bounds.upper.states(4) = 2.0;

    problem.phases(1).bounds.lower.controls(1) = -1.0;
    problem.phases(1).bounds.lower.controls(2) = -1.0;

    problem.phases(1).bounds.upper.controls(1) = 1.0;
    problem.phases(1).bounds.upper.controls(2) = 1.0;

    problem.phases(1).bounds.lower.events(1) = 0.0;
    problem.phases(1).bounds.lower.events(2) = 0.0;
    problem.phases(1).bounds.lower.events(3) = 0.5;
    problem.phases(1).bounds.lower.events(4) = 0.0;
    problem.phases(1).bounds.lower.events(5) = 0.0;
    problem.phases(1).bounds.lower.events(6) = 0.0;
    problem.phases(1).bounds.lower.events(7) = 0.5;
    problem.phases(1).bounds.lower.events(8) = 0.522;

    problem.phases(1).bounds.upper.events = problem.phases(1).bounds.lower.events;



    problem.phases(1).bounds.lower.StartTime    = 0.0;
    problem.phases(1).bounds.upper.StartTime    = 0.0;

    problem.phases(1).bounds.lower.EndTime      = 1.0;
    problem.phases(1).bounds.upper.EndTime      = 10.0;


////////////////////////////////////////////////////////////////////////////
///////////////////  Register problem functions  ///////////////////////////
////////////////////////////////////////////////////////////////////////////


    problem.integrand_cost 	= &integrand_cost;
    problem.endpoint_cost 	= &endpoint_cost;
    problem.dae 		= &dae;
    problem.events 		= &events;
    problem.linkages		= &linkages;



////////////////////////////////////////////////////////////////////////////
///////////////////  Define & register initial guess ///////////////////////
////////////////////////////////////////////////////////////////////////////


    DMatrix x0(4,40);

    x0(1,colon()) = linspace(0.0,0.0, 40);
    x0(2,colon()) = linspace(0.0,0.0, 40);
    x0(3,colon()) = linspace(0.5,0.5, 40);
    x0(4,colon()) = linspace(0.522,0.522, 40);

    problem.phases(1).guess.controls       = zeros(1,40);
    problem.phases(1).guess.states         = x0;
    problem.phases(1).guess.time           = linspace(0.0, 3.0, 40); 

////////////////////////////////////////////////////////////////////////////
///////////////////  Enter algorithm options  //////////////////////////////
////////////////////////////////////////////////////////////////////////////


    algorithm.nlp_method                  = "IPOPT";
    algorithm.scaling                     = "automatic";
    algorithm.derivatives                 = "automatic";
    algorithm.nlp_iter_max                = 1000;
    algorithm.nlp_tolerance               = 1.e-6;

////////////////////////////////////////////////////////////////////////////
///////////////////  Now call PSOPT to solve the problem   /////////////////
////////////////////////////////////////////////////////////////////////////

    psopt(solution, problem, algorithm);

////////////////////////////////////////////////////////////////////////////
///////////  Extract relevant variables from solution structure   //////////
////////////////////////////////////////////////////////////////////////////

    x 		= solution.get_states_in_phase(1);
    u 		= solution.get_controls_in_phase(1);
    t 		= solution.get_time_in_phase(1);

////////////////////////////////////////////////////////////////////////////
///////////  Save solution data to files if desired ////////////////////////
////////////////////////////////////////////////////////////////////////////

    x.Save("x.dat");
    u.Save("u.dat");
    t.Save("t.dat");
 

////////////////////////////////////////////////////////////////////////////
///////////  Plot some results if desired (requires gnuplot) ///////////////
////////////////////////////////////////////////////////////////////////////

    plot(t,x,problem.name + ": states", "time (s)", "states", "x1 x2 x3 x4");

    plot(t,u,problem.name + ": controls", "time (s)", "controls", "u1 u2");


    plot(t,x,problem.name + ": states", "time (s)", "states", "x1 x2 x3 x4", 
                                  "pdf", "twolinkarm_states.pdf");

    plot(t,u,problem.name + ": controls", "time (s)", "controls", "u1 u2", 
                              "pdf", "twolinkarm_controls.pdf");


}

////////////////////////////////////////////////////////////////////////////
///////////////////////      END OF FILE     ///////////////////////////////
////////////////////////////////////////////////////////////////////////////
