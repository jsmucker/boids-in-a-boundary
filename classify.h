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

#ifndef CLASSIFY_H
#define CLASSIFY_H

struct data {
    double p[2];
    double pr[2]; // r, theta
    double v[2];
    double vr[2]; //r,theta
    double force;
};

const int numsession = 3; //number of nested loops
const int depth = 10; //number of increments

struct variation {
    double *param[numsession]; //pointers to parameters to vary
    double start[numsession]; //where to start from
    double cur[numsession];
    double inc[numsession]; //how much to increment
};

struct variation session;

void setupSession();
int categorize(struct flock *f, struct massSpringSystem *mss, FILE *fp);
void rank(double *arr, double *rank, int *index_rank, int size);
void fillData(struct data *boid, struct data *node, struct flock *f, struct massSpringSystem *mss);
int sprocket(struct data *node);
int bullet(struct data *boid);
int densityGrad(struct data *boid);
int oval_rotation(struct data *node);
int circulation(struct data *boid);
int gas(struct data *boid);
int mostNeighbors(struct data *boid);
void loadFile();
double wspeed_ratio();

void setupSession() {
    session.param[0] = &D_repel;
    session.start[0] = 0;
    session.inc[0] = 11;
    session.param[1] = &D_align;
    session.start[1] = 0;
    session.inc[1] = 11;
    //        session.param[2] = &mass_ratio;
    //        session.start[2] = 100.;
    //        session.inc[2] = -10;
    session.param[2] = &hinge_amp;
    session.start[2] = 0. * Node_mass;
    session.inc[2] = (600. * Node_mass) / 10.;

    for (int i = 0; i < numsession; i++) {
        *session.param[i] = session.start[i];
    }
}

void increment(int index) {
    *session.param[index] += session.inc[index];
    return;
}

void startOver(int index) {
    *session.param[index] = session.start[index];
    session.cur[index] = session.start[index];
}

int categorize(struct flock *f, struct massSpringSystem *mss, FILE *fp) {
    //determine number of modes on boundary
    //start by converting boundary positions to polar form
    struct data node[nnodes];
    struct data boid[nb];
    fillData(boid, node, f, mss);

    //perform checks. Note order is important
    char t1 = 'E';
    char t2 = 'E';
    int numSprockets = 0;

    //print tags to file
    if (nb < .25 * nboids) {
        return 0;
    }
    if (oval_rotation(node)) {
        t1 = 'R';
        numSprockets = sprocket(node);
        if (numSprockets) { //else
            t1 = 'S';
        }
    }

    if (bullet(boid)) {
        t2 = 'B';
    } else if (densityGrad(boid)) {
        t2 = 'W';
    } else if (circulation(boid)) {
        t2 = 'C';
    } else if (gas(boid)) {
        t2 = 'G';
    } else if (t1 == 'E' && t2 == 'E') {
        return 0;
    }

    printf("\n\n");
    if (fp)
        fprintf(fp, "TAGS: %c|%c    %d", t1, t2, numSprockets);
    return 1;
}

//sort data

void rank(double arr[], double rank[], int index_rank[], int size) {
    int pos = 0;
    for (int i = 0; i < size; i++) {
        double max = -1;
        double index = -1;
        for (int j = 0; j < size; j++) {
            if (max < arr[j]) {
                int flag = 0;
                for (int k = 0; k < pos; k++) {
                    if (arr[j] == rank[k]) {
                        flag = 1;
                        break;
                    }
                }

                if (flag == 1) {
                    continue;
                }

                max = arr[j];
                index = j;
            }
        }

        rank[pos] = max;
        index_rank[pos] = index;
        pos++;
    }
}

int sprocket(struct data node[]) {
    //find coefficients
    double An[30] = {0};
    double Bn[30] = {0};
    double Cn[30] = {0};
    double max[30] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    int max_int[30] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    int index = 0;
    for (int i = 0; i < 30; i++) {
        for (int j = 0; j < nnodes; j++) {
            double dtheta = node[j].pr[1];

            //sometimes the endpoints get messed up. Apply correction if neccessary
            if (dtheta > 3) dtheta -= 2 * M_PI;
            else if (dtheta<-3) dtheta += 2 * M_PI;

            An[i] += node[j].pr[0] * sin(i * dtheta) / (nnodes * big_radius);
            Bn[i] += node[j].pr[0] * cos(i * dtheta) / (nnodes * big_radius);
        }

        Cn[i] = sqrt(An[i] * An[i] + Bn[i] * Bn[i]);
    }

    //sort coefficients and cancel out noise
    rank(Cn, max, max_int, 30);

    int numAmp = 0;
    for (int i = 0; i < 30; i++) {
        if (max[i] < max[0]*.07) max_int[i] = -1;
        else {
            numAmp++;
            //printf("max_int: %d\tMag: %f\n", max_int[i], max[i]);
        }

    }

    //categorize
    //first check if there are a lot of high amplitudes. This identifies a junk sprocket
    if (max_int[1] >= 3) {
        printf("SPROCKET\n");
        return max_int[1];
    } else if (max[2] + max[1]*.5 >= max[1] && max_int[2] >= 3) {
        printf("SPROCKET\n");
        return max_int[2];
    } else return 0;

    //check for sprocket first
    //    for (int i = 10; i > -1; i--) {
    //        if (max_int[i] > 4) {
    //            printf("Sprocket\n");
    //            return 1;
    //        }
    //    }
    //    return 0;
}

void fillData(struct data boid[], struct data node[], struct flock *f, struct massSpringSystem *mss) {
    //nodes
    for (int i = 0; i < nnodes; i++) {
        for (int j = 0; j < 2; j++) {
            node[i].p[j] = mss->nodeSet[i].position[j];
            node[i].v[j] = mss->nodeSet[i].velocity[j];
        }

        node[i].pr[0] = sqrt(node[i].p[0] * node[i].p[0] + node[i].p[1] * node[i].p[1]);
        node[i].pr[1] = acos(node[i].p[0] / node[i].pr[0]);

        if (node[i].p[1] < 0) {
            node[i].pr[1] *= -1;
            node[i].pr[1] += 2 * M_PI;
        }

        node[i].vr[0] = sqrt(node[i].v[0] * node[i].v[0] + node[i].v[1] * node[i].v[1]);
        node[i].vr[1] = acos(node[i].v[0] / node[i].vr[0]);

        if (node[i].v[1] < 0) {
            node[i].vr[1] *= -1;
            node[i].vr[1] += 2 * M_PI;
        }

        node[i].force = 0;
    }

    for (int i = 0; i < nnodes; i++) {
        double dtheta;
        if (i != nnodes - 1) {
            dtheta = (node[i].pr[1] - node[i + 1].pr[1]);
        } else {
            dtheta = (node[i].pr[1] - node[i + 1].pr[0]);
        }

        //sometimes the endpoints get messed up. Apply correction if neccessary
        if (dtheta > 3) dtheta -= 2 * M_PI;
        else if (dtheta<-3) dtheta += 2 * M_PI;
    }
    //boids
    for (int i = 0; i < nb; i++) {
        for (int j = 0; j < 2; j++) {
            boid[i].p[j] = f->b[i].position[j];
            boid[i].v[j] = f->b[i].velocity[j];
        }

        boid[i].pr[0] = sqrt(boid[i].p[0] * boid[i].p[0] + boid[i].p[1] * boid[i].p[1]);
        boid[i].pr[1] = acos(boid[i].p[0] / boid[i].pr[0]);
        if (boid[i].p[1] < 0)boid[i].pr[1] *= -1;

        boid[i].vr[0] = sqrt(boid[i].v[0] * boid[i].v[0] + boid[i].v[1] * boid[i].v[1]);
        boid[i].vr[1] = acos(boid[i].v[0] / boid[i].vr[0]);
        if (boid[i].v[1] < 0)boid[i].vr[1] *= -1;

    }
}

int bullet(struct data boid[]) {
    //find average velocity
    double p[2] = {0};
    for (int i = 0; i < nb; i++) {
        add(p, boid[i].v, p);
    }
    s_mult(p, 1. / nb);

    double stdev = 0;
    //find standard deviation
    for (int i = 0; i < nb; i++) {
        double diff[2] = {0};
        subtract(boid[i].v, p, diff);
        stdev += mag(diff) * mag(diff);
    }
    stdev /= (nb - 1);
    stdev = sqrt(stdev);

    if (stdev < .1 * boidSpeed) {
        printf("BULLET\n");
        return 1;
    }

    return 0;
}

int densityGrad(struct data boid[]) {
    //select boid 
    int num = mostNeighbors(boid);
    int count = 0;

    //count how many boids are in proximity of base boid
    for (int i = 0; i < nb; i++) {
        if (i == num) continue;
        double a = distance(boid[num].p, boid[i].p);
        if (a < big_radius / sqrt(2.)) {
            count++;
        }
    }

    if (count > .75 * nb) {
        printf("WANDERING FLOCK");
        return 1;
    } else return 0;
}

int oval_rotation(struct data node[]) {
    //now find tangent component of velocity to curve
    double tangent_v = 0;
    for (int i = 0; i < nnodes; i++) {
        //start by finding normal vector to radial position
        double r[2];
        double z[3] = {0, 0, 1};
        double p[2];
        copy(node[i].p, r);
        cross(r, z, p);
        normalize(p);
        tangent_v += dot(node[i].v, p);
    }
    tangent_v /= nnodes;

    if (fabs(tangent_v) > .03 * boidSpeed) {
        printf("ROTATION\n");
        return 1;
    } else return 0;
}

int circulation(struct data boid[]) {
    double tangent_v = 0;
    for (int i = 0; i < nnodes; i++) {
        double r[2];
        double z[3] = {0, 0, 1};
        double p[2];
        copy(boid[i].p, r);
        cross(r, z, p);
        normalize(p);

        tangent_v += dot(boid[i].v, p);
    }
    tangent_v /= nb;

    if (fabs(tangent_v) > .2 * boidSpeed) {
        printf("CIRCULATING\n");
        return 1;
    }
    return 0;
}

int gas(struct data boid[]) {
    //check tangential velocities
    double total_vt = 0;
    double v_[2] = {0};

    for (int i = 0; i < nb; i++) {
        double v[2] = {0};
        double t[2] = {0};
        double z[3] = {0, 0, -1};
        copy(boid[i].v, v);
        cross(boid[i].p, z, t);
        normalize(t);

        total_vt += dot(t, v);
    }

    s_mult(v_, 1. / nb);

    if (fabs(total_vt) < boidSpeed * .2 /*|| mag(v_) < mag_v * .2*/) {
        printf("GAS\n");
        return 1;
    } else return 0;
}

int mostNeighbors(struct data boid[]) {
    double checkRadius = big_radius / sqrt(2.);
    int neighborCount[2000] = {0};

    for (int i = 0; i < nb; i++) {
        int count = 0;
        for (int j = 0; j < nb; j++) {
            double a = distance(boid[i].p, boid[j].p);
            if (a < checkRadius) count++;
        }
        neighborCount[i] = count;
    }

    //find max
    int max = neighborCount[0];
    int index = 0;
    for (int i = 1; i < nb; i++) {
        if (max < neighborCount[i]) {
            max = neighborCount[i];
            index = i;
        }
    }
    return index;
}

void loadFile() {
    //get input
    printf("Enter the file index: ");
    int index = 0;
    scanf("%d", &index);
    printf("\n\n");

    //open file
    char name[32];
    snprintf(name, sizeof (char) * 32, "parameters%d.txt", index);
    printf("%s\n", name);
    FILE *fp;
    fp = fopen(name, "r");
    if (!fp) printf("PROBLEM\n");

    //read file
    int ival = 0;
    float dval = 0;
    int junk = 0;
    char dump[100];
    char label[30];

    fgets(dump, 100, fp);

    //get nboids
    fscanf(fp, "%s%d", label, &ival);
    nboids = ival;
    printf("%s%d\n", label, ival);
    printf("%d\n\n", nboids);
    fgets(dump, 100, fp);
    fgets(dump, 100, fp);

    //get nodes
    fscanf(fp, "%s%d", label, &ival);
    printf("%s%d\n", label, ival);
    nnodes = ival;
    printf("%d\n\n", nnodes);
    fgets(dump, 100, fp);

    //get mass ratio
    fscanf(fp, "%s%f", label, &dval);
    printf("%s %f\n", label, dval);
    mass_ratio = dval;
    printf("%f\n\n", mass_ratio);
    fgets(dump, 100, fp);

    //get D_repel
    fscanf(fp, "%s%f", label, &dval);
    printf("%s %f\n", label, dval);
    D_repel = dval*big_radius;
    printf("%f\n\n", D_repel);
    fgets(dump, 100, fp);

    //get urepel
    fscanf(fp, "%s%f", label, &dval);
    printf("%s %f\n", label, dval);
    Repel_force = boidSpeed * boidSpeed*dval;
    printf("%f\n\n", Repel_force);
    fgets(dump, 100, fp);

    //get align amp
    fscanf(fp, "%s%f", label, &dval);
    printf("%s %f\n", label, dval);
    Align_force = boidSpeed / big_radius*dval;
    printf("%f\n\n", Align_force);
    fgets(dump, 100, fp);

    //get d_align
    fscanf(fp, "%s%f", label, &dval);
    printf("%s %f\n", label, dval);
    D_align = big_radius*dval;
    printf("%f\n\n", D_align);
    fgets(dump, 100, fp);

    //get hinge_amp
    fscanf(fp, "%s%f", label, &dval);
    printf("%s %f\n", label, dval);
    // hinge_amp = mass_ratio * M * boidSpeed * boidSpeed * big_radius * dval;
    printf("%f\n\n", hinge_amp / Node_mass);
    fgets(dump, 100, fp);

    //get gamm_node
    fscanf(fp, "%s%f", label, &dval);
    printf("%s %f\n", label, dval);
    Gamma_node = (boidSpeed / big_radius) * dval;
    printf("%f\n\n", Gamma_node);
    fgets(dump, 100, fp);

    fclose(fp);
}

//not currently correct

double wspeed_ratio() {
    double swim_p = (nb / (2 * M_PI * big_radius * big_radius)) * boidSpeed * boidSpeed * 1. / Align_force;
    return swim_p * D_repel * D_repel / hinge_amp;
}
#endif /* CLASSIFY_H */

