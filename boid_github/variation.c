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

//toggle display on/off
const int display = 0;

int main() {
    srand(time(0));

    setupSession();

    // render loop
    int index = 0;

    //quadtree
    struct btree *bt = NULL;

    //increment loop
    for (int i = 0; i < depth; i++) {
        for (int j = 0; j < depth; j++) {
            for (int k = 0; k < depth; k++) {
                //initialize some of the parameters
                convert();

                GLFWwindow *window;
                if (display) {
                    //configure window and set callback functions
                    window = createWindow();

                    //setup and link shaders to programs
                    shaders();
                }

                //initialize flock and boundary
                struct flock f;
                struct massSpringSystem mss;
                initSystem(&f, &mss);

                while (((display) ? !glfwWindowShouldClose(window) : 1) && total_t < 2000) {
                    if (display) {
                        //setup vertex objects
                        setupVertexObjects();
                    }

                    //maintain tree
                    if (tree) {
                        if ((index % 20 == 0) || checkBounds(&f)) {
                            bt = restore(bt, &f);
                        } else refreshTree(bt);
                    } else {
                        checkBounds(&f);
                    }
                    integrate(bt, &f, &mss);

                    //if the number of boids are low, we have bad data. So break loop
                    if (nb < nboids * .75) break;

                    //center system
                    shift(&mss, &f);

                    if (display) {
                        //clear background and depth buffer
                        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
                        glClear(GL_COLOR_BUFFER_BIT);
                        glClear(GL_DEPTH_BUFFER_BIT);

                        drawParticles(&f, &mss);

                        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
                        glfwSwapBuffers(window);
                        glfwPollEvents();

                        // optional: de-allocate all resources once they've outlived their purpose:
                        glDeleteVertexArrays(8, VAO);
                        glDeleteBuffers(8, VBO);
                    }
                    index++;
                }

                //make sure we can describe the system. If not integrate foward
                while (!categorize(&f, &mss, NULL)) {
                    checkBounds(&f);
                    integrate(bt, &f, &mss);
                    shift(&mss, &f);
                }

                printValues(&f, &mss);
                total_t = 0;
                index = 0;
                if (display) {
                    glfwTerminate();
                }
                increment(0);
            }
            startOver(0);
            increment(1);
        }
        startOver(1);
        increment(2);
    }
    return 0;
}

