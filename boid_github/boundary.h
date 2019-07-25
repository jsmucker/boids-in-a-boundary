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

#ifndef BOUNDARY_H
#define BOUNDARY_H

struct Spring {
    int i;
    int j;
    double L0;
};

struct massSpringSystem {
    struct particle nodeSet[1000];
    struct Spring springSet[3000];
    int nspring;
    double eps;
    double dx;
};

//function prototypes
void add_spring_set(struct massSpringSystem *mms, int nene, double ks0, double gammas0);
void rest_spring(struct massSpringSystem *mms, int k);
void set_spring_lengths(struct massSpringSystem *mms);
void updateks(struct massSpringSystem *mms);
void springforce(struct massSpringSystem *, int);
void applysprings(struct massSpringSystem *);
void zeroaccel(struct massSpringSystem *mms);
void centroid(struct massSpringSystem *mms, double *sumVec);
void shift(struct massSpringSystem *mms, struct flock *f);
void node_damp(struct massSpringSystem *mms);
void compute_accel(struct massSpringSystem *mms);
void single_timestep(struct massSpringSystem *mms);
void fillArray(struct massSpringSystem *mss, float *bound);
void findCurvature(struct massSpringSystem *mss);
void hingeforce(struct massSpringSystem *mss);

//definitions
void add_spring_set(struct massSpringSystem *mms, int nene, double ks0, double gammas0) {
    // construct springs with nearest neighbor separation integer nene
    // ks are adjusted with ksfac*ks
    // damping is adjusted with gammafac*gammas
    for (int i = 0, j = mms->nspring; i < nnodes; i++, j++) {
        struct Spring s;
        s.i = i;
        s.j = (i + nene) % nnodes;
        s.L0 = 1.;
        mms->springSet[j] = s;
        mms->nspring += 1;
    }
}

void rest_spring(struct massSpringSystem *mms, int k) {
    int i = mms->springSet[k].i;
    int j = mms->springSet[k].j;
    struct particle *nodei = &mms->nodeSet[i];
    struct particle *nodej = &mms->nodeSet[j];
    double distij = distance(nodei->position, nodej->position);
    mms->springSet[k].L0 = distij;
}

void set_spring_lengths(struct massSpringSystem *mms) {
    for (int k = 0; k < mms->nspring; k++) {
        rest_spring(mms, k);
    }
}

void springforce(struct massSpringSystem *mms, int k) {
    int i = mms->springSet[k].i;
    int j = mms->springSet[k].j;
    struct particle *nodei = &mms->nodeSet[i];
    struct particle *nodej = &mms->nodeSet[j];
    double L0 = mms->springSet[k].L0;

    double Lvec[2];
    double Lhat[2];

    subtract(nodei->position, nodej->position, Lvec);
    copy(Lvec, Lhat);
    normalize(Lhat);

    double L = mag(Lvec);

    double kForce[2];
    double kai[2];
    double kaj[2];

    // kForce is spring force
    s_multr(Lhat, Ks * (L - L0), kForce);
    s_multr(kForce, 1.0 / Node_mass, kai);
    s_multr(kForce, 1.0 / Node_mass, kaj);

    // sum spring accelerations on nodes
    subtract(nodei->acceleration, kai, nodei->acceleration);
    add(nodej->acceleration, kaj, nodej->acceleration);

    //dForce is damping force
    if (Gammas > 0) {
        double dV[2];
        subtract(nodei->velocity, nodej->velocity, dV);
        double dxdotdv = dot(Lvec, dV);

        double mu_mass = Node_mass * Node_mass / (Node_mass + Node_mass);
        double dForce[2];
        s_multr(Lhat, Gammas * dxdotdv * mu_mass / (L + mms->eps), dForce);
        //acceleration on node i and j
        double dai[2];
        s_multr(dForce, 1.0 / Node_mass, dai);
        double daj[2];
        s_multr(dForce, 1.0 / Node_mass, daj);
        //sum damping accelerations on nodes
        subtract(nodei->acceleration, dai, nodei->acceleration);
        add(nodej->acceleration, daj, nodej->acceleration);
    }
}

void applysprings(struct massSpringSystem *mms) {
    for (int i = 0; i < mms->nspring; i++) {
        springforce(mms, i);
    }
}

void zeroaccel(struct massSpringSystem *mms) {
    for (int i = 0; i < nnodes; i++) {
        for (int j = 0; j < 2; j++)
            mms->nodeSet[i].acceleration[j] = 0;
    }
}

void centroid(struct massSpringSystem *mms, double *sumvec) {
    zero(sumvec);

    for (int i = 0; i < nnodes; i++) {
        struct particle *nodei = &mms->nodeSet[i];
        add(nodei->position, sumvec, sumvec);
    }
    s_mult(sumvec, 1. / nnodes);
}

void shift(struct massSpringSystem *mms, struct flock *f) {
    double sumVec[2];
    centroid(mms, sumVec);
    for (int i = 0; i < nnodes; i++) {
        struct particle *nodei = &mms->nodeSet[i];
        subtract(nodei->position, sumVec, nodei->position);
    }
    for (int i = 0; i < nb; i++) {
        struct particle *b = &f->b[i];
        subtract(b->position, sumVec, b->position);
    }
}

void node_damp(struct massSpringSystem *mms) {
    for (int i = 0; i < nnodes; i++) {
        struct particle *nodei = &mms->nodeSet[i];
        double dForce[2];
        s_multr(nodei->velocity, Gamma_node, dForce);
        subtract(nodei->acceleration, dForce, nodei->acceleration);
    }
}

void compute_accel(struct massSpringSystem *mms) {
    applysprings(mms);
    node_damp(mms);
}

void single_timestep(struct massSpringSystem *mms) {
    // accelerations have to be already computed
    for (int i = 0; i < nnodes; i++) {
        struct particle *nodei = &mms->nodeSet[i];
        double dv[2];
        s_multr(nodei->acceleration, dt, dv);
        add(nodei->velocity, dv, nodei->velocity);
        double dr[2];
        s_multr(nodei->velocity, dt, dr);
        add(nodei->position, dr, nodei->position);
    }
}

void hingeforce(struct massSpringSystem *mss) {
    for (int i = 0; i < nnodes; i++) {
        //select some nodes
        struct particle *nodeh = NULL;
        struct particle *nodei = NULL;
        if (i == 0) {
            nodei = &mss->nodeSet[nnodes - 1];
            nodeh = &mss->nodeSet[nnodes - 2];
        } else {
            nodei = &mss->nodeSet[i - 1];
            if (i == 1) {
                nodeh = &mss->nodeSet[nnodes - 1];
            } else {
                nodeh = &mss->nodeSet[i - 2];
            }
        }

        struct particle *nodej = &mss->nodeSet[i];

        struct particle *nodek = NULL;
        struct particle *nodel = NULL;
        if (i == nnodes - 1) {
            nodek = &mss->nodeSet[0];
            nodel = &mss->nodeSet[1];
        } else {
            nodek = &mss->nodeSet[i + 1];
            if (i == nnodes - 2) {
                nodel = &mss->nodeSet[0];
            } else {
                nodel = &mss->nodeSet[i + 2];
            }
        }

        //h i j k l
        double x4[2] = {0, 0};
        double space = 2 * M_PI * big_radius / nnodes;

        for (int i = 0; i < 2; i++) {
            x4[i] = (nodeh->position[i] - 4 * nodei->position[i] + 6 * nodej->position[i] - 4 * nodek->position[i] + nodel->position[i]) / (space * space * space * space);
        }
        s_mult(x4, -space * hinge_amp / Node_mass);
        add(x4, nodej->acceleration, nodej->acceleration);
    }
}

void fillArray(struct massSpringSystem *mss, float bound[]) {
    //fill node data
    int j = 0;
    for (int i = 0; i < nnodes; i++) {
        bound[i + j] = (float) mss->nodeSet[i].position[0] / normalizeArr;
        bound[i + j + 1] = (float) mss->nodeSet[i].position[1] / normalizeArr;
        bound[i + j + 2] = 0.;
        j += 2;
    }

}

void boid_node_interac(struct flock *f, struct massSpringSystem *mss) {
    int n_nod = nnodes;
    int n_bd = nb;
    for (int i = 0; i < n_nod; i++) {
        struct particle *nodei = &mss->nodeSet[i];
        for (int j = 0; j < n_bd; j++) {
            struct particle *boidj = &f->b[j];
            double dr[2];
            subtract(boidj->position, nodei->position, dr);
            double d = mag(dr);

            if (d < 3 / force_k) {
                double drhat[2];
                copy(dr, drhat);
                normalize(drhat);
                double Force[2];
                copy(drhat, Force);

                s_mult(Force, -force_amp * exp(-force_k * d));
                if (vforce_amp > 0) {
                    double dv[2];
                    subtract(boidj->velocity, nodei->velocity, dv);
                    double vForce[2];
                    copy(dv, vForce);
                    s_mult(vForce, vforce_amp);
                    add(Force, vForce, Force);
                }
                double a_boid[2];
                double a_node[2];
                copy(Force, a_boid);
                copy(Force, a_node);

                s_mult(a_boid, 1. / (-M));
                s_mult(a_node, 1. / (Node_mass));
                add(boidj->acceleration, a_boid, boidj->acceleration);
                add(nodei->acceleration, a_node, nodei->acceleration);
            }
        }

    }
}

#endif /* BOUNDARY_H */

