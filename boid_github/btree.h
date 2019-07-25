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

#ifndef TREE_H
#define TREE_H

#include "boundary.h"

struct btree {
    double loc[2];
    double width;
    int key;
    int q;

    struct particle *data;
    struct btree *up;
    struct btree *quad[4];
};

struct btree *parent = NULL;

double leafSpace(struct btree *bt, int depth);
void boid_node_interactHelper(struct btree *t);
void alignHelper(struct btree *t);
void repelHelper(struct btree *t);
void refreshTree(struct btree *t);
void alignHelper(struct btree *root);
int height(struct btree *);
void addBoid(struct particle *b, struct btree *t, int key);
int findQuad(struct particle *b, struct btree *t);
struct btree * initTree();
void resetTree(struct btree *t);
void destroy(struct btree *t);
void setRoot(struct btree *t);
void boidboid(struct btree *t);
void boidboidHelper(struct btree *t);
struct btree *restore(struct btree *t, struct flock *f);

void setRoot(struct btree *t) {
    parent = t;
}

struct btree * initTree() {
    struct btree *n = malloc(sizeof (struct btree));
    //initialization
    for (int i = 0; i < 4; i++) {
        n->quad[i] = NULL;
        n->data = NULL;
    }

    n->loc[0] = 0;
    n->loc[1] = 0;
    n->up = NULL;
    n->q = -1;
    n->key = -1;
    n->width = normalizeArr * 2; 
    return n;
}

void fillTree(struct flock *f) {
    struct btree *t = parent;

    //add the node
    for (int i = 0; i < nb; i++) {
        addBoid(&f->b[i], t, i);
    }
}

void addBoid(struct particle *b, struct btree *t, int k) {
    //if there is not a node at quad[num] then add one
    int num = findQuad(b, t);

    if (!t->quad[num]) {
        struct btree *n = initTree();
        n->data = b;
        n->q = num;
        n->up = t;
        n->width = t->width / 2;
        if (num == 0) {
            n->loc[0] = t->loc[0] + .5 * n->width;
            n->loc[1] = t->loc[1] + .5 * n->width;
        } else if (num == 1) {
            n->loc[0] = t->loc[0] - .5 * n->width;
            n->loc[1] = t->loc[1] + .5 * n->width;
        } else if (num == 2) {
            n->loc[0] = t->loc[0] - .5 * n->width;
            n->loc[1] = t->loc[1] - .5 * n->width;
        } else if (num == 3) {
            n->loc[0] = t->loc[0] + .5 * n->width;
            n->loc[1] = t->loc[1] - .5 * n->width;
        }
        n->key = k;

        t->quad[num] = n;
        return;
    }
        //if there is data, move parent and daughter down a level
    else if (t->quad[num]->data) {
        struct btree *treeptr = t->quad[num];

        //move parent that was in node to new location
        addBoid(treeptr->data, treeptr, treeptr->key);
        treeptr->data = NULL;

        treeptr->key = -1;
        //move daughter
        addBoid(b, treeptr, k);
    } else if (t->quad[num] && !t->quad[num]->data) {
        struct btree *treeptr = t->quad[num];
        addBoid(b, t->quad[num], k);
    } else printf("NO");
    return;
}

int findQuad(struct particle *b, struct btree *t) {
    double location[2];
    zero(location);

    //find relative position
    subtract(b->position, t->loc, location);

    if (location[0] > 0 && location[1] >= 0) return 0;
    else if (location[0] <= 0 && location[1] >= 0) return 1;
    else if (location[0] <= 0 && location[1] < 0) return 2;
    else if (location[0] > 0 && location[1] < 0) return 3;
    else printf("PROBLEM");
}

int height(struct btree *root) {
    if (root == NULL) return 0;
    else {
        int h[4];
        for (int i = 0; i < 4; i++) {
            h[i] = height(root->quad[i]);
        }

        int holder = h[0];
        //find highest height
        for (int i = 1; i < 4; i++) {
            if (holder < h[i]) holder = h[i];
        }
        return holder + 1;
    }
}

double v_ave[2] = {0, 0};
struct btree *b = NULL;
int count = 0;

void align(struct btree *t) {
    if (!t)
        return;
    //process data
    if (t->data) {
        //set base
        if (!b) {
            b = t;
            //start over with base set
            alignHelper(parent);

            //now that all data has been processed, compute acceleration on base
            if (count > 0) {
                double steer[2];

                //normalize(v_ave);
                double holder = sqrt(v_ave[0] * v_ave[0] + v_ave[1] * v_ave[1]);
                steer[0] = boidSpeed * (v_ave[0] / holder) - b->data->velocity[0];
                steer[1] = boidSpeed * (v_ave[1] / holder) - b->data->velocity[1];

                if (sqrt(steer[0] * steer[0] + steer[1] * steer[1]) > 0) {
                    b->data->acceleration[0] += (steer[0] * M * Align_force / M);
                    b->data->acceleration[1] += (steer[1] * M * Align_force / M);
                }
            } else {
                //if no nearby neighbors, propel boid
                double steer[2];
                v_ave[0] = b->data->velocity[0];
                v_ave[1] = b->data->velocity[1];

                //normalize(v_ave);
                double holder = sqrt(v_ave[0] * v_ave[0] + v_ave[1] * v_ave[1]);
                steer[0] = boidSpeed * (v_ave[0] / holder) - b->data->velocity[0];
                steer[1] = boidSpeed * (v_ave[1] / holder) - b->data->velocity[1];

                if (sqrt(steer[0] * steer[0] + steer[1] * steer[1]) > 0) {
                    b->data->acceleration[0] += (steer[0] * M * Align_force / M);
                    b->data->acceleration[1] += (steer[1] * M * Align_force / M);
                }
            }

            v_ave[0] = 0;
            v_ave[1] = 0;
            count = 0;
            b = NULL;
        }
    }

    //traverse further starting from 0 to 3
    for (int i = 0; i < 4; i++) {
        align(t->quad[i]);
    }
}

void alignHelper(struct btree *t) {
    if (!t)
        return;
    //process data
    if (t->data) {
        //perform pairwise calculations
        if (t->key != b->key) {

            double r = (b->data->position[0] - t->data->position[0])*(b->data->position[0] - t->data->position[0]) + (b->data->position[1] - t->data->position[1])*(b->data->position[1] - t->data->position[1]);
            if (r > 0 && r < D_align * D_align) {
                v_ave[0] += t->data->velocity[0];
                v_ave[1] += t->data->velocity[1];
                count++;
            }
        }
    }

    //traverse further starting from 0 to 3
    for (int i = 0; i < 4; i++) {
        if (t->quad[i]) {
            if ((sqrt((b->data->position[0] - t->quad[i]->loc[0])*(b->data->position[0] - t->quad[i]->loc[0]) + (b->data->position[1] - t->quad[i]->loc[1])*(b->data->position[1] - t->quad[i]->loc[1])) - t->quad[i]->width * 1.4143) > D_align) {
                continue;
            }

            alignHelper(t->quad[i]);
        }
    }
}

int score = 0;

void repel(struct btree *t) {
    if (!t)
        return;
    //process data
    if (t->data) {
        //set base
        if (!b) {
            b = t;
            //start over with base set
            repelHelper(parent);
            score = 0;

            b = NULL;
        }
    }

    //traverse further starting from 0 to 3
    for (int i = 0; i < 4; i++) {
        repel(t->quad[i]);
    }
}

void repelHelper(struct btree *t) {
    if (!t)
        return;

    //process data
    if (t->data) {
        if (score == 1) {
            //calculate pairwise accelerations
            struct particle *b1 = b->data;
            struct particle *b2 = t->data;
            double r_len = sqrt((b1->position[0] - b2->position[0])*(b1->position[0] - b2->position[0]) + (b1->position[1] - b2->position[1])*(b1->position[1] - b2->position[1]));
            if (r_len < 2 * D_repel) {
                double force = (Repel_force * M / D_repel) * exp(-r_len / D_repel);
                double dr[2];

                dr[0] = b1->position[0] - b2->position[0];
                dr[1] = b1->position[1] - b2->position[1];
                double holder = sqrt(dr[0] * dr[0] + dr[1] * dr[1]);
                for (int i = 0; i < 2; i++) dr[i] = (dr[i] * force) / (holder * M);

                for (int i = 0; i < 2; i++) {
                    b1->acceleration[i] += dr[i];
                    b2->acceleration[i] -= dr[i];
                }

            }

        } else if (t->key == b->key) {
            score = 1;
        }
    }

    //traverse further starting from 0 to 3
    for (int i = 0; i < 4; i++) {
        if (t->quad[i]) {
            if ((sqrt((b->data->position[0] - t->quad[i]->loc[0])*(b->data->position[0] - t->quad[i]->loc[0]) + (b->data->position[1] - t->quad[i]->loc[1])*(b->data->position[1] - t->quad[i]->loc[1])) - t->quad[i]->width * 1.4143) > 2 * D_repel) {
                continue;
            }

            repelHelper(t->quad[i]);
        }
    }
}

void destroy(struct btree *t) {
    if (t == NULL)
        return;

    for (int i = 0; i < 4; i++) {
        if (t->quad[i])
            destroy(t->quad[i]);
    }

    free(t);
    t = NULL;
}

struct particle *nodei;

void boid_node_interact(struct btree *t, struct massSpringSystem *mss) {
    int n_nod = nnodes;
    int n_bd = nb;
    for (int i = 0; i < n_nod; i++) {
        nodei = &mss->nodeSet[i];
        boid_node_interactHelper(t);
    }
}

void boid_node_interactHelper(struct btree *t) {
    if (!t)
        return;

    //process data
    if (t->data) {
        struct particle *boidj = t->data;
        double dr[2];
        double d = 0;
        for (int i = 0; i < 2; i++) {
            dr[i] = boidj->position[i] - nodei->position[i];
            d += dr[i] * dr[i];
        }
        d = sqrt(d);

        if (d < 3. / force_k) {
            double drhat[2] = {dr[0] / d, dr[1] / d};

            double r_len = distance(boidj->position, nodei->position);

            double Force[2] = {drhat[0]*-force_amp * 1. / r_len/*exp(-force_k*d)*/, drhat[1]*-force_amp * 1. / r_len/*exp(-force_k*d)*/};

            if (vforce_amp > 0) {
                double dv[2];
                for (int i = 0; i < 2; i++) {
                    dv[i] = boidj->velocity[i] - nodei->velocity[i];
                }
                double vForce[2] = {dv[0] * vforce_amp, dv[1] * vforce_amp};

                for (int i = 0; i < 2; i++) {
                    Force[i] += vForce[i];
                }
            }
            double a_boid[2] = {Force[0]*1. / (-M), Force[1]*1. / (-M)};
            double a_node[2] = {Force[0]*1. / (Node_mass), Force[1]*1. / (Node_mass)};

            for (int i = 0; i < 2; i++) {
                boidj->acceleration[i] += a_boid[i];
                nodei->acceleration[i] += a_node[i];
            }
        }

    }

    //traverse further starting from 0 to 3
    for (int i = 0; i < 4; i++) {
        if (t->quad[i]) {
            double d = sqrt((nodei->position[0] - t->quad[i]->loc[0])*(nodei->position[0] - t->quad[i]->loc[0]) + (nodei->position[1] - t->quad[i]->loc[1])*(nodei->position[1] - t->quad[i]->loc[1])) - t->quad[i]->width * 1.4143;
            if (d > 3. / force_k) {
                continue;
            }

            boid_node_interactHelper(t->quad[i]);
        }
    }
}

//if boids move out of cell, iterate through tree and add boids
void refreshTree(struct btree *t) {
    if (!t) return;

    //if there is data and it is outside of the cell
    if (t->data && (t->data->position[0] > t->loc[0] + t->width / 2 || t->data->position[0] < t->loc[0] - t->width / 2 || t->data->position[1] > t->loc[1] + t->width / 2 || t->data->position[1] < t->loc[1] - t->width / 2)) {
        struct particle *b = t->data;
        int k = t->key;
        t->data = NULL;
        t->key = -1;
        struct btree *p = t->up;
        int qua = t->q;

        free(t);
        t = NULL;
        p->quad[qua] = NULL;
        addBoid(b, parent, k);
        return;
    } else {
        for (int i = 0; i < 4; i++) {
            if (t->quad[i])
                refreshTree(t->quad[i]);
        }
    }
}

struct btree *restore(struct btree *t, struct flock *f) {
    destroy(t);
    t = initTree();
    setRoot(t);
    fillTree(f);
    return t;
}

//checks density of boids 
double leafSpace(struct btree *bt, int depth) {
    if (depth == 0) return 0;
    double sum = 0;
    for (int i = 0; i < 4; i++) {
        if (!bt->quad[i]) sum += (bt->width / 2)*(bt->width / 2);
        else if (bt->quad[i]->data) sum += 0;
        else sum += leafSpace(bt->quad[i], depth - 1);
    }
    return sum;
}
#endif /* TREE_H */

