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

#ifndef BOIDS_H
#define BOIDS_H

struct particle {
    double position[2];
    double velocity[2];
    double acceleration[2];
    double phi;
    double phidot;
    double dx;

};

struct flock {
    struct particle b[2000];
};

int checkBounds(struct flock *f) {
    int s = 0;
    for (int i = 0; i < nb; i++) {
        if (f->b[i].position[0] / normalizeArr > 1 || f->b[i].position[1] / normalizeArr > 1 || f->b[i].position[0] / normalizeArr < -1 || f->b[i].position[1] / normalizeArr<-1) {
            randVec(f->b[i].position, -xw, xw);

            zero(f->b[i].acceleration);
            s++;
        }
    }
    return s;
}

void zeroAccel(struct flock *f) {
    for (int i = 0; i < nb; i++) {
        for (int j = 0; j < 2; j++) {
            f->b[i].acceleration[j] = 0.;
        }
    }
}

void step(struct flock *f) {
    for (int i = 0; i < nb; i++) {
        for (int j = 0; j < 2; j++) {
            f->b[i].velocity[j] += f->b[i].acceleration[j] * dt;
            f->b[i].position[j] += f->b[i].velocity[j] * dt;
        }
    }
}

void transformArr(struct flock *f, float new[]) {
    //boid triangles
    for (int i = 0, j = 0; i < nb; i++, j += 9) {
        double unitv[2];
        double pvec[2];
        pvec[0] = f->b[i].position[0] / normalizeArr;
        pvec[1] = f->b[i].position[1] / normalizeArr;

        copy(f->b[i].velocity, unitv);

        s_mult(unitv, .01);

        //vertex 1
        new[j] = unitv[0] + pvec[0]; //arr[k];
        new[j + 1] = unitv[1] + pvec[1]; //arr[k+1];
        new[j + 2] = 0;

        //vertex 2 (place 2 and three along perpendicular direction)
        //perpendicular vector
        unitv[1] = -unitv[0] / unitv[1];
        unitv[0] = 1;
        normalize(unitv);
        s_mult(unitv, .015);

        new[j + 3] = pvec[0] + unitv[0];
        new[j + 4] = pvec[1] + unitv[1];
        new[j + 5] = 0;

        //vertex 3
        new[j + 6] = pvec[0] - unitv[0];
        new[j + 7] = pvec[1] - unitv[1];
        new[j + 8] = 0;

    }
}

void alig(struct flock *f) {
    if (Align_force == 0) return;

    double v_ave[2];
    double dr[2];
    zero(dr);
    zero(v_ave);
    for (int i = 0, count = 0; i < nb; i++) {
        struct particle *b1 = &f->b[i];
        for (int j = 0; j < nb; j++) {
            struct particle *b2 = &f->b[j];
            double r;
            r = distance(b1->position, b2->position);
            if (r > 0 && (r < D_align)) {
                add(v_ave, b2->velocity, v_ave);
                count++;
            }
        }
        if (count > 0) {
            double steer[2];
            normalize(v_ave);
            s_mult(v_ave, boidSpeed);
            subtract(v_ave, b1->velocity, steer);

            if (mag(steer) > 0) {
                double steer2[2];
                b1->acceleration[0] += (steer[0] * M * Align_force / M);
                b1->acceleration[1] += (steer[1] * M * Align_force / M);
            }
        } else {
            //if no nearby neighbors, propel boid
            double steer[2];
            v_ave[0] = b1->velocity[0];
            v_ave[1] = b1->velocity[1];

            double holder = sqrt(v_ave[0] * v_ave[0] + v_ave[1] * v_ave[1]);
            steer[0] = boidSpeed * (v_ave[0] / holder) - b1->velocity[0];
            steer[1] = boidSpeed * (v_ave[1] / holder) - b1->velocity[1];
            b1->acceleration[0] += (steer[0] * M * Align_force / M);
            b1->acceleration[1] += (steer[1] * M * Align_force / M);
        }
    }
}

void repe(struct flock *f) {
    if (Repel_force == 0) {
        return;
    }
    
    for (int i = 0; i < nb - 1; i++) {
        struct particle *b1 = &f->b[i];
        for (int j = i + 1; j < nb; j++) {
            struct particle *b2 = &f->b[j];
            double r_len = distance(b1->position, b2->position);
            if (r_len < 2 * D_repel) {
                double force = (Repel_force * M / D_repel) * exp(-r_len / D_repel);
                double dr[2];
                unit_dr(b1->position, b2->position, dr);

                s_mult(dr, force * (1 / M));
                add(b1->acceleration, dr, b1->acceleration);
                s_mult(dr, (M / M));
                subtract(b2->acceleration, dr, b2->acceleration);
            }

        }
    }
}


#endif /* BOIDS_H */

