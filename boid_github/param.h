/*
 * Visualization:
 * Copyright (c) https://learnopengl.com/Getting-started/Hello-Triangle
 * License: https://creativecommons.org/licenses/by-nc/4.0/legalcode
 * Joey de Vries
 * https://learnopengl.com
 * https://twitter.com/JoeyDeVriez.
 * 
 * Also, uses some boid/boundary functions from a javascript code: 
 * https://github.com/aquillen/boids_oval
 */

/* 
 * File:   main.c
 * Author: jonas
 *
 * Created on July 4, 2019, 10:09 AM
 */

#include <math.h>
#ifndef CONSTANTS_H
#define CONSTANTS_H

//Note: some of these parameters are initialized in the function on the bottom of the page to allow for slider variation

//toggle tree mode
const int tree = 0;

//dimensionless numbers
double mass_ratio = 10; //total mass of nodes/total mass of boids

//currently not used
double drepel_ratio = .044444; //drepel/R
double urepel_ratio = .253750; //urepel/v^2
double aalign_ratio = 2.8125; //aalign*t_r
double dalign_ratio = .044444; //d_align/R
double abend_ratio = .166667; //abend/(m_node*v^2*R)
double damping_ratio = .05625; //gamma*t_r

int nnodes = 180;    //number of nodes
const double Node_mass = 6.; // mass of nodes
int nboids = 140;   //number of boids
int nb; //dont change (variable boid size incase boids are deleted)
double M;       //boid mass 
double boidSpeed = 4.;
const int nslider = 6;      //number of sliders

//Boundary parameters
const double rad_fac = 0.75; // sets radius of node boundary 
const double Dx = 1.; // grid spacing
double Ks = 3*10 / Dx; // spring constant
const double Gammas = 0.0; //  damping parm
double Gamma_node = 0.001; // damping on nodes (force depends on velocity)

//node force amplitudes
double force_amp = 10.; // for interactions between boids and nodes
double force_k = .2; // 1/scale for interactions between boids and nodes
const double vforce_amp = 0.00; // damping

//canvas parameters
const double canvasSize = 600;
double xw = 130;
double vw = .1;
const double xwidth = canvasSize*Dx;
const double yheight = canvasSize*Dx;
double big_radius = rad_fac*xwidth / 2;
float normalizeArr;

//distances for forces
double D_repel = 50; // distance for attraction/repulsion forces between Boid
double D_align = 50; // distance for alignment


//boid force amplitude
double Repel_force = 4; 
double Align_force = .05; 

//screen dimensions
unsigned int SCR_WIDTH = 800;
unsigned int SCR_HEIGHT = 800;

//time step
double total_t = 0;
const double dt = .3;

//boundary force amplitude
double hinge_amp = 600 * Node_mass; //N m^2 

//convertions
void convert() {
    nb = nboids;
    M = (1./mass_ratio) * (Node_mass * nnodes) / nboids;
    normalizeArr = big_radius*1.5;

//    D_repel = drepel_ratio*big_radius;
//    D_align = dalign_ratio*big_radius;
//    Repel_force = urepel_ratio*boidSpeed*boidSpeed;
//    Align_force = aalign_ratio/(big_radius/boidSpeed);
//    hinge_amp = abend_ratio*Node_mass*boidSpeed*boidSpeed*big_radius;
//    Gamma_node = damping_ratio/(big_radius/boidSpeed);
}

#endif /* CONSTANTS_H */

