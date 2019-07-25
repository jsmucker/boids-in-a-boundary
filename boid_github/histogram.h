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

#ifndef HISTOGRAM_H
#define HISTOGRAM_H

struct bin {
    float location[3];
    float o_location[3];
    float freq;

};

//internal param
int histogram = 0;
int slow = 0;
int toggleMode = 0;
double norm = -1;

//resolution
const int bins = 10;

//other params
const double p_range = 1;
const double v_range = 1;
const double normScale = .8;
const double space_width = (2. * p_range) / bins;
const double padding = space_width / 100.;
const double z_offset = -.6;
float xvec[3] = {1, 0, 0};
float yvec[3] = {0, 1, 0};
float zvec[3] = {0, 0, 1};


struct bin B[bins*bins];

void p_createBars(float *, float*, float *, float*, struct flock *);
void v_createBars(float *, float*, float *, float*, struct flock *);
void quadrants(float *);
void setupBin(struct flock *);
void p_refreshBin();
void v_refreshBin();
void xrotate(double);
void yrotate(double);
void zrotate(double);

void quadrants(float arr[]) {
    arr[0] = -xvec[0] + z_offset * zvec[0];
    arr[1] = -xvec[1] + z_offset * zvec[1];
    arr[2] = -xvec[2] + z_offset * zvec[2];

    arr[3] = xvec[0] + z_offset * zvec[0];
    arr[4] = xvec[1] + z_offset * zvec[1];
    arr[5] = xvec[2] + z_offset * zvec[2];

    arr[6] = -yvec[0] + z_offset * zvec[0];
    arr[7] = -yvec[1] + z_offset * zvec[1];
    arr[8] = -yvec[2] + z_offset * zvec[2];

    arr[9] = yvec[0] + z_offset * zvec[0];
    arr[10] = yvec[1] + z_offset * zvec[1];
    arr[11] = yvec[2] + z_offset * zvec[2];
}

void setupBin(struct flock *f) {
    double cur[3] = {-(p_range) + space_width / 2., -(p_range) + space_width / 2., 0};

    //initialize bins
    for (int i = 0; i < bins * bins; i++) {
        B[i].freq = 0;
        B[i].location[0] = cur[0];
        B[i].location[1] = cur[1];
        B[i].location[2] = z_offset + cur[2];

        if (cur[0] < p_range - space_width) {
            cur[0] += space_width;
        } else {
            cur[0] = -p_range + space_width / 2.;
            cur[1] += space_width;
        }
        for (int k = 0; k < 3; k++)
            B[i].o_location[k] = B[i].location[k];
    }
    double vmax = -1;
    //find largest velocity
    for (int i = 0; i < nb; i++) {
        if (vmax < mag(f->b[i].velocity)) vmax = mag(f->b[i].velocity);
    }
    norm = vmax * 5;

    xrotate(-4. / 170.);
    yrotate(-4. / 75.);
    zrotate(4. / 70.);
}

void p_refreshBin() {
    double cur[3] = {-(p_range) + space_width / 2., -(p_range) + space_width / 2., 0};

    for (int i = 0; i < bins * bins; i++) {
        B[i].freq = 0;
        B[i].location[0] = cur[0];
        B[i].location[1] = cur[1];
        B[i].location[2] = -.6 + cur[2];

        if (cur[0] < p_range - space_width) {
            cur[0] += space_width;
        } else {
            cur[0] = -p_range + space_width / 2.;
            cur[1] += space_width;
        }
    }
}

void v_refreshBin() {
    double cur[3] = {-(v_range) + space_width / 2., -(v_range) + space_width / 2., 0};

    for (int i = 0; i < bins * bins; i++) {
        B[i].freq = 0;
        B[i].location[0] = cur[0];
        B[i].location[1] = cur[1];
        B[i].location[2] = -.6 + cur[2];

        if (cur[0] < v_range - space_width) {
            cur[0] += space_width;
        } else {
            cur[0] = -v_range + space_width / 2.;
            cur[1] += space_width;
        }
    }
}

void v_createBars(float base[], float side1[], float side2[], float top[], struct flock *f) {
    //find frequency
    int max = -1;

    for (int i = 0; i < bins * bins; i++) {
        B[i].freq = 0;
        for (int j = 0; j < nb; j++) {
            if (f->b[j].velocity[0] / (norm) > B[i].location[0] - space_width / 2. && f->b[j].velocity[0] / (norm) < B[i].location[0] + space_width / 2. && f->b[j].velocity[1] / (norm) > B[i].location[1] - space_width / 2. && f->b[j].velocity[1] / (norm) < B[i].location[1] + space_width / 2.) {
                B[i].freq++;
            }
        }
        if (max < B[i].freq) max = B[i].freq;

    }

    //normalize vectors
    double holder = sqrt(zvec[0] * zvec[0] + zvec[1] * zvec[1] + zvec[2] * zvec[2]);
    double holder1 = sqrt(yvec[0] * yvec[0] + yvec[1] * yvec[1] + yvec[2] * yvec[2]);
    double holder2 = sqrt(xvec[0] * xvec[0] + xvec[1] * xvec[1] + xvec[2] * xvec[2]);

    for (int i = 0; i < 3; i++) {
        xvec[i] /= holder2;
        yvec[i] /= holder1;
        zvec[i] /= holder;
    }

    //now that frequency has been checked, create array
    int place = 0;
    //render base
    for (int i = 0; i < bins * bins; i++) {
        B[i].freq /= (max + 1);

        //render base
        base[place + 0] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
        base[place + 1] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        base[place + 2] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;

        base[place + 3] = (B[i].o_location[0] - xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + padding * xvec[0] - padding * yvec[0]) * normScale; // + padding;
        base[place + 4] = (B[i].o_location[1] - xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        base[place + 5] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + padding * xvec[2] - yvec[2] * padding) * normScale;

        base[place + 6] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 - padding * xvec[0] - padding * yvec[0]) * normScale; // - padding;
        base[place + 7] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        base[place + 8] = (B[i].o_location[2] + 0 + xvec[2] * space_width / 2 + yvec[2] * space_width / 2 - padding * xvec[2] - yvec[2] * padding) * normScale;

        base[place + 9] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
        base[place + 10] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        base[place + 11] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;

        base[place + 12] = (B[i].o_location[0] + xvec[0] * space_width / 2 - yvec[0] * space_width / 2 - padding * xvec[0] + padding * yvec[0]) * normScale; // - padding;
        base[place + 13] = (B[i].o_location[1] + xvec[1] * space_width / 2 - yvec[1] * space_width / 2 - padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        base[place + 14] = (B[i].o_location[2] + 0 + xvec[2] * space_width / 2 - yvec[2] * space_width / 2 - padding * xvec[2] + yvec[2] * padding) * normScale;

        base[place + 15] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 - padding * xvec[0] - padding * yvec[0]) * normScale; // - padding;
        base[place + 16] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        base[place + 17] = (B[i].o_location[2] + 0 + xvec[2] * space_width / 2 + yvec[2] * space_width / 2 - padding * xvec[2] - yvec[2] * padding) * normScale;
        place += 18;
    }
    place = 0;
    for (int i = 0; i < bins * bins; i++) {
        //render faces
        //f1
        side1[place + 0] = (B[i].o_location[0] - xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + padding * xvec[0] - padding * yvec[0]) * normScale; // + padding;
        side1[place + 1] = (B[i].o_location[1] - xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side1[place + 2] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + padding * xvec[2] - yvec[2] * padding) * normScale;

        side1[place + 3] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
        side1[place + 4] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side1[place + 5] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;

        side1[place + 6] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
        side1[place + 7] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side1[place + 8] = (B[i].o_location[2] + B[i].freq * zvec[2] - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;

        side1[place + 12] = (B[i].o_location[0] - xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + padding * xvec[0] - padding * yvec[0]) * normScale; // + padding;
        side1[place + 13] = (B[i].o_location[1] - xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side1[place + 14] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + padding * xvec[2] - yvec[2] * padding) * normScale;

        side1[place + 9] = (B[i].o_location[0] - xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + zvec[0] * B[i].freq + padding * xvec[0] - padding * yvec[0]) * normScale; // + padding;
        side1[place + 10] = (B[i].o_location[1] - xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + zvec[1] * B[i].freq + padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side1[place + 11] = (B[i].o_location[2] + B[i].freq * zvec[2] - xvec[2] * space_width / 2 + yvec[0] * space_width / 2 - yvec[2] * padding + padding * xvec[2]) * normScale;

        side1[place + 15] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
        side1[place + 16] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side1[place + 17] = (B[i].o_location[2] + B[i].freq * zvec[2] - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;
        //
        //        //f2
        side1[place + 18] = (B[i].o_location[0] + xvec[0] * space_width / 2 - yvec[0] * space_width / 2 - padding * xvec[0] + padding * yvec[0]) * normScale; // - padding;
        side1[place + 19] = (B[i].o_location[1] + xvec[1] * space_width / 2 - yvec[1] * space_width / 2 - padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side1[place + 20] = (B[i].o_location[2] + 0 + xvec[2] * space_width / 2 - yvec[2] * space_width / 2 - padding * xvec[2] + yvec[2] * padding) * normScale;

        side1[place + 21] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 - padding * xvec[0] - padding * yvec[0]) * normScale; // - padding;
        side1[place + 22] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side1[place + 23] = (B[i].o_location[2] + 0 + xvec[2] * space_width / 2 + yvec[2] * space_width / 2 - padding * xvec[2] - yvec[2] * padding) * normScale;

        side1[place + 24] = (B[i].o_location[0] + xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0] + padding * yvec[0]) * normScale; // - padding;
        side1[place + 25] = (B[i].o_location[1] + xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side1[place + 26] = (B[i].o_location[2] + xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + B[i].freq * zvec[2] - padding * xvec[2] + yvec[2] * padding) * normScale;

        side1[place + 27] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 - padding * xvec[0] - padding * yvec[0]) * normScale;
        side1[place + 28] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding
        side1[place + 29] = (B[i].o_location[2] + 0 + xvec[2] * space_width / 2 + yvec[2] * space_width / 2 - padding * xvec[2] - yvec[2] * padding) * normScale;

        side1[place + 30] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0] - padding * yvec[0]) * normScale; // - padding;
        side1[place + 31] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side1[place + 32] = (B[i].o_location[2] + xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + B[i].freq * zvec[2] - padding * xvec[2] - yvec[2] * padding) * normScale;

        side1[place + 33] = (B[i].o_location[0] + xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0] + padding * yvec[0]) * normScale; // - padding;
        side1[place + 34] = (B[i].o_location[1] + xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side1[place + 35] = (B[i].o_location[2] + xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + B[i].freq * zvec[2] - padding * xvec[2] + yvec[2] * padding) * normScale;
        place += 36;
    }
    place = 0;
    //f3
    for (int i = 0; i < bins * bins; i++) {
        side2[place + 0] = (B[i].o_location[0] + xvec[0] * space_width / 2 - yvec[0] * space_width / 2 - padding * xvec[0] + padding * yvec[0]) * normScale; // - padding;
        side2[place + 1] = (B[i].o_location[1] + xvec[1] * space_width / 2 - yvec[1] * space_width / 2 - padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side2[place + 2] = (B[i].o_location[2] + 0 + xvec[2] * space_width / 2 - yvec[2] * space_width / 2 - padding * xvec[2] + yvec[2] * padding) * normScale;

        side2[place + 3] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
        side2[place + 4] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side2[place + 5] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;

        side2[place + 6] = (B[i].o_location[0] + xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0] + padding * yvec[0]) * normScale; // - padding;
        side2[place + 7] = (B[i].o_location[1] + xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side2[place + 8] = (B[i].o_location[2] + xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + B[i].freq * zvec[2] - padding * xvec[2] + yvec[2] * padding) * normScale;

        side2[place + 9] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
        side2[place + 10] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side2[place + 11] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;

        side2[place + 12] = (B[i].o_location[0] + xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0] + padding * yvec[0]) * normScale; // - padding;
        side2[place + 13] = (B[i].o_location[1] + xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side2[place + 14] = (B[i].o_location[2] + xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + B[i].freq * zvec[2] - padding * xvec[2] + yvec[2] * padding) * normScale;

        side2[place + 15] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
        side2[place + 16] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side2[place + 17] = (B[i].o_location[2] + B[i].freq * zvec[2] - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;

        //f4
        side2[place + 18] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 - padding * xvec[0] - padding * yvec[0]) * normScale; // - padding;
        side2[place + 19] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side2[place + 20] = (B[i].o_location[2] + 0 + xvec[2] * space_width / 2 + yvec[2] * space_width / 2 - padding * xvec[2] - yvec[2] * padding) * normScale;

        side2[place + 21] = (B[i].o_location[0] - xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + padding * xvec[0] - padding * yvec[0]) * normScale; // + padding;
        side2[place + 22] = (B[i].o_location[1] - xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side2[place + 23] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + padding * xvec[2] - yvec[2] * padding) * normScale;

        side2[place + 24] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0] - padding * yvec[0]) * normScale; // - padding;
        side2[place + 25] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side2[place + 26] = (B[i].o_location[2] + xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + B[i].freq * zvec[2] - padding * xvec[2] - yvec[2] * padding) * normScale;

        side2[place + 27] = (B[i].o_location[0] - xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + padding * xvec[0] - padding * yvec[0]) * normScale; // + padding;
        side2[place + 28] = (B[i].o_location[1] - xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + padding * xvec[1] - yvec[1] * padding) * normScale; //- padding;
        side2[place + 29] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + padding * xvec[2] - yvec[2] * padding) * normScale;

        side2[place + 30] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0] - padding * yvec[0]) * normScale; // - padding;
        side2[place + 31] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side2[place + 32] = (B[i].o_location[2] + xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + B[i].freq * zvec[2] - padding * xvec[2] - yvec[2] * padding) * normScale;

        side2[place + 33] = (B[i].o_location[0] - xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + zvec[0] * B[i].freq + padding * xvec[0] - padding * yvec[0]) * normScale; // + padding;
        side2[place + 34] = (B[i].o_location[1] - xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + zvec[1] * B[i].freq + padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side2[place + 35] = (B[i].o_location[2] + B[i].freq * zvec[2] - xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + padding * xvec[2] - yvec[2] * padding) * normScale;
        place += 36;
    }
    place = 0;
    //f5
    for (int i = 0; i < bins * bins; i++) {
        if (B[i].freq > 0) {
            top[place + 0] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
            top[place + 1] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
            top[place + 2] = (B[i].o_location[2] + B[i].freq * zvec[2] - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;

            top[place + 3] = (B[i].o_location[0] - xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + zvec[0] * B[i].freq + padding * xvec[0] - padding * yvec[0]) * normScale; // + padding;
            top[place + 4] = (B[i].o_location[1] - xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + zvec[1] * B[i].freq + padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
            top[place + 5] = (B[i].o_location[2] + B[i].freq * zvec[2] - xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + padding * xvec[2] - yvec[2] * padding) * normScale;

            top[place + 6] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0] - padding * yvec[0]) * normScale; // - padding;
            top[place + 7] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
            top[place + 8] = (B[i].o_location[2] + xvec[2] * space_width / 2 + B[i].freq * zvec[2] + yvec[2] * space_width / 2 - padding * xvec[2] - yvec[2] * padding) * normScale;

            top[place + 9] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
            top[place + 10] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
            top[place + 11] = (B[i].o_location[2] + B[i].freq * zvec[2] - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;

            top[place + 12] = (B[i].o_location[0] + xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0]) * normScale; // - padding;
            top[place + 13] = (B[i].o_location[1] + xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1]) * normScale; // + padding;
            top[place + 14] = (B[i].o_location[2] + xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + B[i].freq * zvec[2] - padding * xvec[2]) * normScale;

            top[place + 15] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0] - padding * yvec[0]) * normScale; // - padding;
            top[place + 16] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
            top[place + 17] = (B[i].o_location[2] + xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + B[i].freq * zvec[2] - padding * xvec[2] - yvec[2] * padding) * normScale;
        } else {
            for (int j = 0; j < 18; j++) {
                top[place + j] = 0;
            }
        }

        place += 18;
    }

}

void p_createBars(float base[], float side1[], float side2[], float top[], struct flock *f) {
    //find frequency
    int max = -1;
    for (int i = 0; i < bins * bins; i++) {
        B[i].freq = 0;
        for (int j = 0; j < nb; j++) {
            if (f->b[j].position[0] / (normalizeArr) > B[i].location[0] - space_width / 2. && f->b[j].position[0] / (normalizeArr) < B[i].location[0] + space_width / 2. && f->b[j].position[1] / (normalizeArr) > B[i].location[1] - space_width / 2. && f->b[j].position[1] / (normalizeArr) < B[i].location[1] + space_width / 2.) {
                B[i].freq++;
            }
        }
        if (max < B[i].freq) max = B[i].freq;
    }

    //normalize vectors
    double holder = sqrt(zvec[0] * zvec[0] + zvec[1] * zvec[1] + zvec[2] * zvec[2]);
    double holder1 = sqrt(yvec[0] * yvec[0] + yvec[1] * yvec[1] + yvec[2] * yvec[2]);
    double holder2 = sqrt(xvec[0] * xvec[0] + xvec[1] * xvec[1] + xvec[2] * xvec[2]);

    for (int i = 0; i < 3; i++) {
        xvec[i] /= holder2;
        yvec[i] /= holder1;
        zvec[i] /= holder;
    }

    //now that frequency has been checked, create array
    int place = 0;
    //render base
    for (int i = 0; i < bins * bins; i++) {
        B[i].freq /= (max + 1);

        //render base
        base[place + 0] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
        base[place + 1] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        base[place + 2] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;

        base[place + 3] = (B[i].o_location[0] - xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + padding * xvec[0] - padding * yvec[0]) * normScale; // + padding;
        base[place + 4] = (B[i].o_location[1] - xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        base[place + 5] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + padding * xvec[2] - yvec[2] * padding) * normScale;

        base[place + 6] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 - padding * xvec[0] - padding * yvec[0]) * normScale; // - padding;
        base[place + 7] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        base[place + 8] = (B[i].o_location[2] + 0 + xvec[2] * space_width / 2 + yvec[2] * space_width / 2 - padding * xvec[2] - yvec[2] * padding) * normScale;

        base[place + 9] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
        base[place + 10] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        base[place + 11] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;

        base[place + 12] = (B[i].o_location[0] + xvec[0] * space_width / 2 - yvec[0] * space_width / 2 - padding * xvec[0] + padding * yvec[0]) * normScale; // - padding;
        base[place + 13] = (B[i].o_location[1] + xvec[1] * space_width / 2 - yvec[1] * space_width / 2 - padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        base[place + 14] = (B[i].o_location[2] + 0 + xvec[2] * space_width / 2 - yvec[2] * space_width / 2 - padding * xvec[2] + yvec[2] * padding) * normScale;

        base[place + 15] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 - padding * xvec[0] - padding * yvec[0]) * normScale; // - padding;
        base[place + 16] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        base[place + 17] = (B[i].o_location[2] + 0 + xvec[2] * space_width / 2 + yvec[2] * space_width / 2 - padding * xvec[2] - yvec[2] * padding) * normScale;
        place += 18;
    }
    //printf("%d\n",place);
    place = 0;
    for (int i = 0; i < bins * bins; i++) {
        //render faces
        //f1
        side1[place + 0] = (B[i].o_location[0] - xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + padding * xvec[0] - padding * yvec[0]) * normScale; // + padding;
        side1[place + 1] = (B[i].o_location[1] - xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side1[place + 2] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + padding * xvec[2] - yvec[2] * padding) * normScale;

        side1[place + 3] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
        side1[place + 4] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side1[place + 5] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;

        side1[place + 6] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
        side1[place + 7] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side1[place + 8] = (B[i].o_location[2] + B[i].freq * zvec[2] - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;

        side1[place + 12] = (B[i].o_location[0] - xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + padding * xvec[0] - padding * yvec[0]) * normScale; // + padding;
        side1[place + 13] = (B[i].o_location[1] - xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side1[place + 14] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + padding * xvec[2] - yvec[2] * padding) * normScale;

        side1[place + 9] = (B[i].o_location[0] - xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + zvec[0] * B[i].freq + padding * xvec[0] - padding * yvec[0]) * normScale; // + padding;
        side1[place + 10] = (B[i].o_location[1] - xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + zvec[1] * B[i].freq + padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side1[place + 11] = (B[i].o_location[2] + B[i].freq * zvec[2] - xvec[2] * space_width / 2 + yvec[0] * space_width / 2 - yvec[2] * padding + padding * xvec[2]) * normScale;

        side1[place + 15] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
        side1[place + 16] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side1[place + 17] = (B[i].o_location[2] + B[i].freq * zvec[2] - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;
        //
        //        //f2
        side1[place + 18] = (B[i].o_location[0] + xvec[0] * space_width / 2 - yvec[0] * space_width / 2 - padding * xvec[0] + padding * yvec[0]) * normScale; // - padding;
        side1[place + 19] = (B[i].o_location[1] + xvec[1] * space_width / 2 - yvec[1] * space_width / 2 - padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side1[place + 20] = (B[i].o_location[2] + 0 + xvec[2] * space_width / 2 - yvec[2] * space_width / 2 - padding * xvec[2] + yvec[2] * padding) * normScale;

        side1[place + 21] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 - padding * xvec[0] - padding * yvec[0]) * normScale; // - padding;
        side1[place + 22] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side1[place + 23] = (B[i].o_location[2] + 0 + xvec[2] * space_width / 2 + yvec[2] * space_width / 2 - padding * xvec[2] - yvec[2] * padding) * normScale;

        side1[place + 24] = (B[i].o_location[0] + xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0] + padding * yvec[0]) * normScale; // - padding;
        side1[place + 25] = (B[i].o_location[1] + xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side1[place + 26] = (B[i].o_location[2] + xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + B[i].freq * zvec[2] - padding * xvec[2] + yvec[2] * padding) * normScale;

        side1[place + 27] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 - padding * xvec[0] - padding * yvec[0]) * normScale;
        side1[place + 28] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding
        side1[place + 29] = (B[i].o_location[2] + 0 + xvec[2] * space_width / 2 + yvec[2] * space_width / 2 - padding * xvec[2] - yvec[2] * padding) * normScale;

        side1[place + 30] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0] - padding * yvec[0]) * normScale; // - padding;
        side1[place + 31] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side1[place + 32] = (B[i].o_location[2] + xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + B[i].freq * zvec[2] - padding * xvec[2] - yvec[2] * padding) * normScale;

        side1[place + 33] = (B[i].o_location[0] + xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0] + padding * yvec[0]) * normScale; // - padding;
        side1[place + 34] = (B[i].o_location[1] + xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side1[place + 35] = (B[i].o_location[2] + xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + B[i].freq * zvec[2] - padding * xvec[2] + yvec[2] * padding) * normScale;
        place += 36;
    }
    place = 0;
    //f3
    for (int i = 0; i < bins * bins; i++) {
        side2[place + 0] = (B[i].o_location[0] + xvec[0] * space_width / 2 - yvec[0] * space_width / 2 - padding * xvec[0] + padding * yvec[0]) * normScale; // - padding;
        side2[place + 1] = (B[i].o_location[1] + xvec[1] * space_width / 2 - yvec[1] * space_width / 2 - padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side2[place + 2] = (B[i].o_location[2] + 0 + xvec[2] * space_width / 2 - yvec[2] * space_width / 2 - padding * xvec[2] + yvec[2] * padding) * normScale;

        side2[place + 3] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
        side2[place + 4] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side2[place + 5] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;

        side2[place + 6] = (B[i].o_location[0] + xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0] + padding * yvec[0]) * normScale; // - padding;
        side2[place + 7] = (B[i].o_location[1] + xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side2[place + 8] = (B[i].o_location[2] + xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + B[i].freq * zvec[2] - padding * xvec[2] + yvec[2] * padding) * normScale;

        side2[place + 9] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
        side2[place + 10] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side2[place + 11] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;

        side2[place + 12] = (B[i].o_location[0] + xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0] + padding * yvec[0]) * normScale; // - padding;
        side2[place + 13] = (B[i].o_location[1] + xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side2[place + 14] = (B[i].o_location[2] + xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + B[i].freq * zvec[2] - padding * xvec[2] + yvec[2] * padding) * normScale;

        side2[place + 15] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
        side2[place + 16] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
        side2[place + 17] = (B[i].o_location[2] + B[i].freq * zvec[2] - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;

        //f4
        side2[place + 18] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 - padding * xvec[0] - padding * yvec[0]) * normScale; // - padding;
        side2[place + 19] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side2[place + 20] = (B[i].o_location[2] + 0 + xvec[2] * space_width / 2 + yvec[2] * space_width / 2 - padding * xvec[2] - yvec[2] * padding) * normScale;

        side2[place + 21] = (B[i].o_location[0] - xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + padding * xvec[0] - padding * yvec[0]) * normScale; // + padding;
        side2[place + 22] = (B[i].o_location[1] - xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side2[place + 23] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + padding * xvec[2] - yvec[2] * padding) * normScale;

        side2[place + 24] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0] - padding * yvec[0]) * normScale; // - padding;
        side2[place + 25] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side2[place + 26] = (B[i].o_location[2] + xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + B[i].freq * zvec[2] - padding * xvec[2] - yvec[2] * padding) * normScale;

        side2[place + 27] = (B[i].o_location[0] - xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + padding * xvec[0] - padding * yvec[0]) * normScale; // + padding;
        side2[place + 28] = (B[i].o_location[1] - xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + padding * xvec[1] - yvec[1] * padding) * normScale; //- padding;
        side2[place + 29] = (B[i].o_location[2] + 0 - xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + padding * xvec[2] - yvec[2] * padding) * normScale;

        side2[place + 30] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0] - padding * yvec[0]) * normScale; // - padding;
        side2[place + 31] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side2[place + 32] = (B[i].o_location[2] + xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + B[i].freq * zvec[2] - padding * xvec[2] - yvec[2] * padding) * normScale;

        side2[place + 33] = (B[i].o_location[0] - xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + zvec[0] * B[i].freq + padding * xvec[0] - padding * yvec[0]) * normScale; // + padding;
        side2[place + 34] = (B[i].o_location[1] - xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + zvec[1] * B[i].freq + padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
        side2[place + 35] = (B[i].o_location[2] + B[i].freq * zvec[2] - xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + padding * xvec[2] - yvec[2] * padding) * normScale;
        place += 36;
    }
    place = 0;
    //f5
    for (int i = 0; i < bins * bins; i++) {
        if (B[i].freq > 0) {
            top[place + 0] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
            top[place + 1] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
            top[place + 2] = (B[i].o_location[2] + B[i].freq * zvec[2] - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;

            top[place + 3] = (B[i].o_location[0] - xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + zvec[0] * B[i].freq + padding * xvec[0] - padding * yvec[0]) * normScale; // + padding;
            top[place + 4] = (B[i].o_location[1] - xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + zvec[1] * B[i].freq + padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
            top[place + 5] = (B[i].o_location[2] + B[i].freq * zvec[2] - xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + padding * xvec[2] - yvec[2] * padding) * normScale;

            top[place + 6] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0] - padding * yvec[0]) * normScale; // - padding;
            top[place + 7] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
            top[place + 8] = (B[i].o_location[2] + xvec[2] * space_width / 2 + B[i].freq * zvec[2] + yvec[2] * space_width / 2 - padding * xvec[2] - yvec[2] * padding) * normScale;

            top[place + 9] = (B[i].o_location[0] - xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq + padding * xvec[0] + padding * yvec[0]) * normScale; // + padding;
            top[place + 10] = (B[i].o_location[1] - xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq + padding * xvec[1] + padding * yvec[1]) * normScale; // + padding;
            top[place + 11] = (B[i].o_location[2] + B[i].freq * zvec[2] - xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + padding * xvec[2] + yvec[2] * padding) * normScale;

            top[place + 12] = (B[i].o_location[0] + xvec[0] * space_width / 2 - yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0]) * normScale; // - padding;
            top[place + 13] = (B[i].o_location[1] + xvec[1] * space_width / 2 - yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1]) * normScale; // + padding;
            top[place + 14] = (B[i].o_location[2] + xvec[2] * space_width / 2 - yvec[2] * space_width / 2 + B[i].freq * zvec[2] - padding * xvec[2]) * normScale;

            top[place + 15] = (B[i].o_location[0] + xvec[0] * space_width / 2 + yvec[0] * space_width / 2 + zvec[0] * B[i].freq - padding * xvec[0] - padding * yvec[0]) * normScale; // - padding;
            top[place + 16] = (B[i].o_location[1] + xvec[1] * space_width / 2 + yvec[1] * space_width / 2 + zvec[1] * B[i].freq - padding * xvec[1] - yvec[1] * padding) * normScale; // - padding;
            top[place + 17] = (B[i].o_location[2] + xvec[2] * space_width / 2 + yvec[2] * space_width / 2 + B[i].freq * zvec[2] - padding * xvec[2] - yvec[2] * padding) * normScale;
        } else {
            for (int j = 0; j < 18; j++) {
                top[place + j] = 0;
            }
        }

        place += 18;
    }

}

void xrotate(double sign) {
    double rotate = M_PI / (50 * sign);

    //rotate quadrants
    for (int i = 0; i < bins * bins; i++) {
        //translate to center
        float loc[3] = {B[i].o_location[0], B[i].o_location[1], B[i].o_location[2]};

        float holder[3] = {B[i].o_location[0], B[i].o_location[1], B[i].o_location[2]};
        B[i].o_location[1] = cos(rotate) * holder[1] - sin(rotate) * holder[2];
        B[i].o_location[2] = sin(rotate) * holder[1] + cos(rotate) * holder[2];
    }

    //rotate pvec
    float holder[3] = {zvec[0], zvec[1], zvec[2]};
    zvec[1] = cos(rotate) * holder[1] - sin(rotate) * holder[2];
    zvec[2] = sin(rotate) * holder[1] + cos(rotate) * holder[2];

    float holder1[3] = {yvec[0], yvec[1], yvec[2]};
    yvec[1] = cos(rotate) * holder1[1] - sin(rotate) * holder1[2];
    yvec[2] = sin(rotate) * holder1[1] + cos(rotate) * holder1[2];

    float holder2[3] = {xvec[0], xvec[1], xvec[2]};
    xvec[1] = cos(rotate) * holder2[1] - sin(rotate) * holder2[2];
    xvec[2] = sin(rotate) * holder2[1] + cos(rotate) * holder2[2];
}

void yrotate(double sign) {
    double rotate = M_PI / (50 * sign);

    //rotate quadrants
    for (int i = 0; i < bins * bins; i++) {
        float holder[3] = {B[i].o_location[0], B[i].o_location[1], B[i].o_location[2]};
        B[i].o_location[0] = cos(rotate) * holder[0] + sin(rotate) * holder[2];
        B[i].o_location[2] = -sin(rotate) * holder[0] + cos(rotate) * holder[2];
    }

    //rotate pvec
    float holder[3] = {zvec[0], zvec[1], zvec[2]};
    zvec[0] = cos(rotate) * holder[0] + sin(rotate) * holder[2];
    zvec[2] = -sin(rotate) * holder[0] + cos(rotate) * holder[2];

    float holder1[3] = {yvec[0], yvec[1], yvec[2]};
    yvec[0] = cos(rotate) * holder1[0] + sin(rotate) * holder1[2];
    yvec[2] = -sin(rotate) * holder1[0] + cos(rotate) * holder1[2];

    float holder2[3] = {xvec[0], xvec[1], xvec[2]};
    xvec[0] = cos(rotate) * holder2[0] + sin(rotate) * holder2[2];
    xvec[2] = -sin(rotate) * holder2[0] + cos(rotate) * holder2[2];
}

void zrotate(double sign) {
    double rotate = M_PI / (50 * sign);

    //rotate quadrants
    for (int i = 0; i < bins * bins; i++) {
        float holder[3] = {B[i].o_location[0], B[i].o_location[1], B[i].o_location[2]};
        B[i].o_location[0] = cos(rotate) * holder[0] - sin(rotate) * holder[1];
        B[i].o_location[1] = sin(rotate) * holder[0] + cos(rotate) * holder[1];
    }

    //rotate pvec
    float holder[3] = {zvec[0], zvec[1], zvec[2]};
    zvec[0] = cos(rotate) * holder[0] - sin(rotate) * holder[1];
    zvec[1] = sin(rotate) * holder[0] + cos(rotate) * holder[1];

    float holder1[3] = {yvec[0], yvec[1], yvec[2]};
    yvec[0] = cos(rotate) * holder1[0] - sin(rotate) * holder1[1];
    yvec[1] = sin(rotate) * holder1[0] + cos(rotate) * holder1[1];

    float holder2[3] = {xvec[0], xvec[1], xvec[2]};
    xvec[0] = cos(rotate) * holder2[0] - sin(rotate) * holder2[1];
    xvec[1] = sin(rotate) * holder2[0] + cos(rotate) * holder2[1];
}


#endif /* HISTOGRAM_H */

