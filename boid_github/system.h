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

#ifndef SYSTEM_H
#define SYSTEM_H

void printValues();
void initSystem(struct flock *f, struct massSpringSystem *mss);
void naught(struct flock *f, struct massSpringSystem *mss);
void processInput(GLFWwindow *window, struct flock *f, struct massSpringSystem *mss);

void initSystem(struct flock *f, struct massSpringSystem *mss) {
    //create boids
    for (int i = 0; i < nboids; i++) {
        randVec(f->b[i].position, -xw, xw);
        randVec(f->b[i].velocity, -vw, vw);

        zero(f->b[i].acceleration);
        f->b[i].phi = (rand() % (int) (1000 * 2 * M_PI)) / 1000;
        f->b[i].phidot = (rand() % 1000) / 1000;
    }

    //initialize parameters
    mss->nspring = 0;
    mss->eps = pow(1., -6);
    mss->dx = Dx;

    //construct nodes in circle
    for (int i = 0; i < nnodes; i++) {
        struct particle b;
        b.position[0] = big_radius * cos(i * 2.0 * M_PI / nnodes);
        b.position[1] = big_radius * sin(i * 2.0 * M_PI / nnodes);
        zero(b.acceleration);
        zero(b.velocity);
        b.dx = Dx;
        mss->nodeSet[i] = b;
    }

    add_spring_set(mss, 1, 1.0 * Ks, 1.0 * Gammas);
    set_spring_lengths(mss);
}

void processInput(GLFWwindow *window, struct flock *f, struct massSpringSystem *mss) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, 1);
    else if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS) {
        printf("TEXTFILE WRITTEN (data.dat)\n");
        usleep(200000);

        //output starting parameters to a text file
        FILE *fp;
        fp = fopen("data.dat", "w");
        for (int i = 0; i < nboids; i++) {
            fprintf(fp, "%f\t%f\n", f->b[i].position[0], f->b[i].position[1]);

        }
        fclose(fp);
    } else if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS) {
        if (histogram == 1) {
            histogram = 0;
            printf("HISTOGRAM OFF\n");
        } else {
            histogram = 1;
            printf("HISTOGRAM ON\n");
        }
        usleep(200000);
    } else if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
        xrotate(1);
        //usleep(200000);
    } else if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
        xrotate(-1);
        //usleep(200000);
    } else if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
        yrotate(1);
        //usleep(200000);
    } else if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
        yrotate(-1);
        //usleep(200000);
    } else if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS) {
        zrotate(-1);
    } else if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS) {
        zrotate(1);
    } else if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        if (slow == 0) {
            slow = 1;
            printf("SLOWING DOWN\n");
            usleep(20000);
        } else {
            slow = 0;
            printf("SPEEDING UP\n");
            usleep(20000);

        }
    } else if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS) {
        if (toggleMode == 0) {
            toggleMode = 1;
            printf("VELOCITY MODE\n");
            v_refreshBin();
        } else {
            toggleMode = 0;
            printf("POSITION MODE\n");
            p_refreshBin();
        }
        usleep(200000);
    } else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        nb = 0;
    } else if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS) {
        nb = nboids;
        for (int i = 0; i < nboids; i++) {
            randVec(f->b[i].position, -xw, xw);
            randVec(f->b[i].velocity, -vw, vw);
        }
        for (int i = 0; i < nnodes; i++) {
            struct particle b;
            b.position[0] = big_radius * cos(i * 2.0 * M_PI / nnodes);
            b.position[1] = big_radius * sin(i * 2.0 * M_PI / nnodes);
            zero(b.acceleration);
            zero(b.velocity);
            b.dx = Dx;
            mss->nodeSet[i] = b;
        }
        if (tree) {
            destroy(parent);
            parent = initTree();
            fillTree(f);
        }
    } else if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
        printValues(f, mss);
        usleep(200000);
    } else if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS) {
        categorize(f, mss, NULL);
        usleep(200000);
    } else if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS) {
        loadFile();
        convert();
        nb = nboids;
        for (int i = 0; i < nboids; i++) {
            randVec(f->b[i].position, -xw, xw);
            randVec(f->b[i].velocity, -vw, vw);
        }
        for (int i = 0; i < nnodes; i++) {
            struct particle b;
            b.position[0] = big_radius * cos(i * 2.0 * M_PI / nnodes);
            b.position[1] = big_radius * sin(i * 2.0 * M_PI / nnodes);
            zero(b.acceleration);
            zero(b.velocity);
            b.dx = Dx;
            mss->nodeSet[i] = b;
        }
        //        initSystem(f, mss);
        usleep(200000);
    }
}

void naught(struct flock *f, struct massSpringSystem *mss) {
    zeroAccel(f);
    zeroaccel(mss);
}

void integrate(struct btree *bt, struct flock *f, struct massSpringSystem *mss) {
    for (int k = 0; k < 1; k++) {
        //find accelerations
        if (tree) {
            align(bt);
            repel(bt);
            boid_node_interact(bt, mss);
        } else {
            alig(f);
            repe(f);
            boid_node_interac(f, mss);
        }
        hingeforce(mss);
        compute_accel(mss);

        //update position and velocity
        single_timestep(mss);
        step(f);

        naught(f, mss);
        total_t += dt;
    }
}

#endif /* SYSTEM_H */

