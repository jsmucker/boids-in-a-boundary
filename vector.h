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

#ifndef VECTOR_H
#define VECTOR_H


void randVec(double *vec, double min, double max);
void printVec(double * v);
void unit_dr(double *, double *, double *);
void normalize(double *);
double mag(double *);
void difference(double *, double *, double *);
double distance(double *, double *);
void s_mult(double *, double);
void s_multr(double *, double, double *);
void add(double *v1, double *v2, double *v3);
void subtract(double *v1, double *v2, double *v3);
void copy(double *, double *);
void zero(double *v);
double dot(double *, double *);
void printArr(float *arr, int size);

void printVec(double vec[]) {
    printf("%f\t%f\n", vec[0], vec[1]);
}

double dot(double v1[], double v2[]) {
    return (v1[0] * v2[0] + v1[1] * v2[1]);
}

void zero(double v[]) {
    for (int i = 0; i < 2; i++) {
        v[i] = 0;
    }
}

void randVec(double vec[], double min, double max) {
    for (int i = 0; i < 2; i++) {
        int m = 10000 * (2 * max + 1);
        vec[i] = (rand() % m) / 10000. + min;
    }
}

//returns a unit vector pointing from v1 to v2

void unit_dr(double v1[], double v2[], double v3[]) {
    difference(v1, v2, v3);
    normalize(v3);
}

void normalize(double v[]) {
    double holder = mag(v);
    for (int i = 0; i < 2; i++) {
        v[i] /= holder;
    }
}

double mag(double v[]) {
    return sqrt(v[0] * v[0] + v[1] * v[1]);
}

void difference(double v1[], double v2[], double v3[]) {

    for (int i = 0; i < 2; i++) {
        v3[i] = v1[i] - v2[i];
    }
}

void s_mult(double v1[], double a) {
    for (int i = 0; i < 2; i++) {
        v1[i] *= a;
    }
}

void s_multr(double v1[], double a, double v2[]) {
    for (int i = 0; i < 2; i++) {
        v2[i] = v1[i] * a;
    }
}

double distance(double v1[], double v2[]) {
    double v3[2];
    difference(v1, v2, v3);
    return mag(v3);
}

void add(double v1[], double v2[], double v3[]) {
    for (int i = 0; i < 2; i++) {
        v3[i] = v1[i] + v2[i];
    }
}

void subtract(double v1[], double v2[], double v3[]) {
    for (int i = 0; i < 2; i++) {
        v3[i] = v1[i] - v2[i];
    }
}

void copy(double v[], double c[]) {
    for (int i = 0; i < 2; i++) {
        c[i] = v[i];
    }
}

void printArr(float arr[], int size) {
    for (int i = 0; i < size; i += 3) {
        printf("%f\t%f\t%f\n", arr[i], arr[i + 1], arr[i + 2]);
    }
}

//used in finding normal vectors. Not exactly the cross product.
void cross(double *v1, double *v2, double *r) {
    r[0] = v1[1] * v2[2];
    r[1] = -v1[0] * v2[2];
    return;
}

#endif /* VECTOR_H */

