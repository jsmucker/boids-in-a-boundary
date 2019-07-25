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

#ifndef SLIDER_H
#define SLIDER_H

struct slider {
    double position[2];
    double limit;
    double height;
    double status;
    double *param;
    double sensativity;
};

struct slider s_array[nslider];
char *sliderNames[] = {"Repel_force: ", "Align Force: ", "D_repel: ", "D_align: ", "hinge_amp: ", "ks: ", "Mass_RATIO : "};
double *p[] = {&Repel_force, &Align_force, &D_repel, &D_align, &hinge_amp, &Ks, &mass_ratio};

void setupSliders() {
    double pad = .01;
    double h = .06;
    double loc = h / 2.;
    for (int i = 0; i < nslider; i++) {
        s_array[i].limit = .25;
        s_array[i].position[0] = -1. + s_array[i].limit / 2.;
        s_array[i].position[1] = 1. - (loc + pad * (i + 1));
        s_array[i].height = h;
        s_array[i].status = 0;

        s_array[i].param = p[i];
        s_array[i].sensativity = s_array[i].limit / (2 * (*p[i]));
        loc += h;
    }
}

void sliderVertices(float arr[]) {
    //find vertices
    for (int i = 0, j = 0; i < 18 * nslider; i += 18, j++) {
        arr[i + 0] = s_array[j].position[0] - s_array[j].limit / 2;
        arr[i + 1] = s_array[j].position[1] - s_array[j].height / 2;
        arr[i + 2] = 0;

        arr[i + 3] = s_array[j].position[0] - s_array[j].limit / 2;
        arr[i + 4] = s_array[j].position[1] + s_array[j].height / 2;
        arr[i + 5] = 0;

        arr[i + 6] = s_array[j].position[0] + s_array[j].status;
        arr[i + 7] = s_array[j].position[1] + s_array[j].height / 2;
        arr[i + 8] = 0;

        arr[i + 9] = s_array[j].position[0] - s_array[j].limit / 2;
        arr[i + 10] = s_array[j].position[1] - s_array[j].height / 2;
        arr[i + 11] = 0;

        arr[i + 12] = s_array[j].position[0] + s_array[j].status;
        arr[i + 13] = s_array[j].position[1] - s_array[j].height / 2;
        arr[i + 14] = 0;

        arr[i + 15] = s_array[j].position[0] + s_array[j].status;
        arr[i + 16] = s_array[j].position[1] + s_array[j].height / 2;
        arr[i + 17] = 0;
    }
}

int searchSliders(double xpos, double ypos) {
    //convert to other coordinate system
    xpos = (xpos - SCR_WIDTH / 2.) / (SCR_WIDTH / 2.);
    ypos = -(ypos - (SCR_HEIGHT / 2.)) / (SCR_HEIGHT / 2.);
    for (int i = 0; i < nslider; i++) {
        if (xpos > s_array[i].position[0] - s_array[i].limit / 2 && xpos < s_array[i].position[0] + s_array[i].limit / 2 && ypos > s_array[i].position[1] - s_array[i].height / 2 && ypos < s_array[i].position[1] + s_array[i].height / 2) {
            return i;
        }
    }
    return -1;
}

void updateSlider(int s, double dx) {
    if (*s_array[s].param + dx / s_array[s].sensativity < 0) {
        dx = -s_array[s].sensativity * (*s_array[s].param);
    }
    if (s_array[s].status + dx > s_array[s].limit / 2) {
        s_array[s].status = s_array[s].limit / 2;
    } else if (s_array[s].status + dx < -s_array[s].limit / 2) {
        s_array[s].status = -s_array[s].limit / 2;
    } else {
        s_array[s].status += dx;
    }

    //change parameters
    *s_array[s].param += dx / s_array[s].sensativity;
    convert();
}

int callCount = 0;

void printValues(struct flock *f, struct massSpringSystem *mss) {
    FILE *fp;
    char buffer[32];
    snprintf(buffer, sizeof (char) * 32, "parameters%d.txt", callCount);

    fp = fopen(buffer, "wb");
    categorize(f, mss, fp);

    fprintf(fp, "\n\n$TOTALBOIDS$: %d\n$N_{boids}$: %d\n$N_{nodes}$: %d\n$\\frac{M_{nodes}}{M_{boids}}$: %f\n$\\frac{d_{repel}}{R}$: %f\n$\\frac{U_{repel}}{v_{0}^2}$: %f\n", nboids, nb, nnodes, mass_ratio, D_repel / big_radius, Repel_force / (boidSpeed * boidSpeed));
    fprintf(fp, "$\\alpha_{align}t_R$: %f\n$\\frac{d_{align}}{R}$: %f\n$\\frac{\\alpha_{bend}}{m_{node}v_{0}^2R}$: %f\n$\\gamma_{node}t_R$: %f\n", Align_force * big_radius / boidSpeed, D_align / big_radius, hinge_amp / (Node_mass * boidSpeed * boidSpeed * big_radius), Gamma_node * big_radius / boidSpeed);
    fprintf(fp, "OTHER DIMENSIONLESS NUMBERS:\n");
    fprintf(fp, "$\\Pi_6$: %f\n", pow(nb, -1. / 3.));
    fprintf(fp, "$\\Pi_7$: %f\n", pow(nb, -1. / 3.) * big_radius / D_repel);
    fprintf(fp, "$\\frac{m_{boid}v_0^2d_{repel}^2}{2\\pi R^2\\alpha_{align}\\alpha_{bend}}$: %f\n", wspeed_ratio());
    fclose(fp);
    callCount++;
    return;
}
#endif /* SLIDER_H */

