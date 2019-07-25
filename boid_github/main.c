/*
 * Copyright (c) https://learnopengl.com/Getting-started/Hello-Triangle
 * License: https://creativecommons.org/licenses/by-nc/4.0/legalcode
 * Joey de Vries
 * https://learnopengl.com
 * https://twitter.com/JoeyDeVriez.
 * 
 * Also, uses some code from a javascript code: 
 * https://github.com/aquillen/boids_oval
 */

/* 
 * File:   main.c
 * Author: jonas
 *
 * Created on July 4, 2019, 10:09 AM
 */

#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "vector.h"
#include "param.h"
#include "boids.h"
#include "btree.h"
#include "boundary.h"
#include "histogram.h"
#include "classify.h"
#include "system.h"
#include "slider.h"
#include "display.h"

int main() {
    //initialize parameters
    convert();

    //configure window and set callback functions
    GLFWwindow *window = createWindow();

    //setup and link shaders to programs
    shaders();

    //initialize flock and boundary
    srand(time(0));
    struct flock f;
    struct massSpringSystem mss;
    initSystem(&f, &mss);

    //bins and sliders
    setupBin(&f);
    setupSliders();

    //quadtree
    struct btree *bt = NULL;

    // render loop
    int index = 0;
    while (!glfwWindowShouldClose(window)) {
        //setup vertex objects
        setupVertexObjects();

        //reset bt in case 'j' has been pressed
        bt = parent;

        //maintain tree (check if boids left screen and if tree should be refreshed)
        if (tree) {
            if ((index % 100 == 0) || checkBounds(&f)) {
                bt = restore(bt, &f);
            } else refreshTree(bt);
        } else {
            checkBounds(&f);
        }

        //step
        integrate(bt, &f, &mss);

        //center system
        shift(&mss, &f);

        // input
        processInput(window, &f, &mss);

        //clear background and depth buffer
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        glClear(GL_DEPTH_BUFFER_BIT);

        //draw particles and histogram
        if (histogram == 0) {
            drawParticles(&f, &mss);

        } else {
            drawHistogram(base, side1, side2, top, &f);

            if (toggleMode == 0) {
                p_createBars(base, side1, side2, top, &f);
            } else {
                v_createBars(base, side1, side2, top, &f);
            }

            quadrants(bounds);

            if (slow == 1) {
                usleep(100000);
            }
        }

        //swap buffers and poll IO events
        glfwSwapBuffers(window);
        glfwPollEvents();

        //de-allocate all resources
        glDeleteVertexArrays(8, VAO);
        glDeleteBuffers(8, VBO);
        index++;
    }

    printf("TIME ELAPSED:%f", total_t);
    glfwTerminate();

    return 0;
}

