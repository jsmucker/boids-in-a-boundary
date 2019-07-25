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

#ifndef DISPLAY_H
#define DISPLAY_H

//shaders
const char *vertexShaderSource = "#version 330 core\n"
        "layout (location = 0) in vec3 aPos;\n"
        "void main()\n"
        "{\n"
        "   gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
        "}\0";
const char *darkOrange = "#version 330 core\n"
        "out vec4 FragColor;\n"
        "void main()\n"
        "{\n"
        "   FragColor = vec4(1.f, 0.5f, 0.2f, 1.0f);\n"
        "}\n\0";
const char *lightOrange = "#version 330 core\n"
        "out vec4 FragColor;\n"
        "void main()\n"
        "{\n"
        "   FragColor = vec4(.5f, .25f, 0.1f, 1.0f);\n"
        "}\n\0";
const char *mediumOrange = "#version 330 core\n"
        "out vec4 FragColor;\n"
        "void main()\n"
        "{\n"
        "   FragColor = vec4(.75f, .375f, 0.15f, 1.0f);\n"
        "}\n\0";
const char *green = "#version 330 core\n"
        "out vec4 FragColor;\n"
        "void main()\n"
        "{\n"
        "   FragColor = vec4(1.0f, 1.0f, 0.0f, 1.0f);\n"
        "}\n\0";
const char *blue = "#version 330 core\n"
        "out vec4 FragColor;\n"
        "void main()\n"
        "{\n"
        "   FragColor = vec4(0.f, 0.f, 1.0f, 1.0f);\n"
        "}\n\0";

//programs
unsigned int blueProgram = 0;;
unsigned int greenProgram = 0;
int lOrangeProgram = 0;
int mOrangeProgram = 0;
int dOrangeProgram = 0;

//visualization arrays
float triangles[9 * (2000)];
float vertices[3 * 1000];
float base[bins * bins * 18];
float side1[bins * bins * 36];
float side2[bins * bins * 36];
float top[bins * bins * 18];
float bounds[12];
float slider[nslider * 18];

//vertex objects
unsigned int VBO[8], VAO[8];

//function parameters
GLFWwindow *createWindow();
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void setupVertexObjects();
void drawParticles(struct flock *f, struct massSpringSystem *mss);
void drawHistogram(float *base, float *side1, float *side2, float *top, struct flock *f);


//function definitions
GLFWwindow *createWindow() {
    // glfw: initialize and configure
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Birds in a Bag", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);

    // glad: load all OpenGL function pointers
    gladLoadGLLoader((GLADloadproc) glfwGetProcAddress);

    return window;
}

void shaders() {
    // build and compile shader program
    // vertex shader
    int vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    // fragment shader1
    int dOrange = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(dOrange, 1, &darkOrange, NULL);
    glCompileShader(dOrange);

    int lOrange = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(lOrange, 1, &lightOrange, NULL);
    glCompileShader(lOrange);

    int mOrange = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(mOrange, 1, &mediumOrange, NULL);
    glCompileShader(mOrange);

    int lBlue = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(lBlue, 1, &blue, NULL);
    glCompileShader(lBlue);

    //fragment shader2
    unsigned int fragmentShader2 = glCreateShader(GL_FRAGMENT_SHADER); // the second fragment shader that outputs the color yellow
    glShaderSource(fragmentShader2, 1, &green, NULL);
    glCompileShader(fragmentShader2);


    // link shaders
    dOrangeProgram = glCreateProgram();
    glAttachShader(dOrangeProgram, vertexShader);
    glAttachShader(dOrangeProgram, dOrange);
    glLinkProgram(dOrangeProgram);

    lOrangeProgram = glCreateProgram();
    glAttachShader(lOrangeProgram, vertexShader);
    glAttachShader(lOrangeProgram, lOrange);
    glLinkProgram(lOrangeProgram);

    mOrangeProgram = glCreateProgram();
    glAttachShader(mOrangeProgram, vertexShader);
    glAttachShader(mOrangeProgram, mOrange);
    glLinkProgram(mOrangeProgram);

    greenProgram = glCreateProgram();
    glAttachShader(greenProgram, vertexShader);
    glAttachShader(greenProgram, fragmentShader2);
    glLinkProgram(greenProgram);

    blueProgram = glCreateProgram();
    glAttachShader(blueProgram, vertexShader);
    glAttachShader(blueProgram, lBlue);
    glLinkProgram(blueProgram);

    glDeleteShader(vertexShader);
    glDeleteShader(dOrange);
    glDeleteShader(lOrange);
    glDeleteShader(mOrange);
    glDeleteShader(lBlue);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

//global slider variables
double dx = 0;
double s = 0;

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS && !dx) {
        double xpos, ypos;
        //getting cursor position
        glfwGetCursorPos(window, &xpos, &ypos);
        s = searchSliders(xpos, ypos);
        if (s != -1) {
            dx = xpos;
        }

    } else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE && dx) {
        double xpos, ypos;
        //getting cursor position
        glfwGetCursorPos(window, &xpos, &ypos);
        dx -= xpos;
        dx *= -1 / 500.;
        updateSlider(s, dx);

        dx = 0;
        s = -1;
    }
}

void setupVertexObjects() {
        glGenVertexArrays(8, VAO);
        glGenBuffers(8, VBO);
        
        glBindVertexArray(VAO[0]);
        glBindBuffer(GL_ARRAY_BUFFER, VBO[0]);
        glBufferData(GL_ARRAY_BUFFER, sizeof (triangles), triangles, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof (float), (void*) 0);
        glEnableVertexAttribArray(0);

        //boundaries
        glBindVertexArray(VAO[1]); // note that we bind to a different VAO now
        glBindBuffer(GL_ARRAY_BUFFER, VBO[1]); // and a different VBO
        glBufferData(GL_ARRAY_BUFFER, sizeof (vertices), vertices, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0); // because the vertex data is tightly packed we can also specify 0 as the vertex attribute's stride to let OpenGL figure it out
        glEnableVertexAttribArray(0);

        //histogram
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glBindVertexArray(VAO[2]);
        glBindBuffer(GL_ARRAY_BUFFER, VBO[2]);
        glBufferData(GL_ARRAY_BUFFER, sizeof (base), base, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof (float), (void*) 0);
        glEnableVertexAttribArray(0);

        glBindVertexArray(VAO[3]);
        glBindBuffer(GL_ARRAY_BUFFER, VBO[3]);
        glBufferData(GL_ARRAY_BUFFER, sizeof (side1), side1, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof (float), (void*) 0);
        glEnableVertexAttribArray(0);

        glBindVertexArray(VAO[4]);
        glBindBuffer(GL_ARRAY_BUFFER, VBO[4]);
        glBufferData(GL_ARRAY_BUFFER, sizeof (side2), side2, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof (float), (void*) 0);
        glEnableVertexAttribArray(0);

        glBindVertexArray(VAO[5]);
        glBindBuffer(GL_ARRAY_BUFFER, VBO[5]);
        glBufferData(GL_ARRAY_BUFFER, sizeof (top), top, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof (float), (void*) 0);
        glEnableVertexAttribArray(0);

        glBindVertexArray(VAO[6]);
        glBindBuffer(GL_ARRAY_BUFFER, VBO[6]);
        glBufferData(GL_ARRAY_BUFFER, sizeof (bounds), bounds, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof (float), (void*) 0);
        glEnableVertexAttribArray(0);

        glBindVertexArray(VAO[7]);
        glBindBuffer(GL_ARRAY_BUFFER, VBO[7]);
        glBufferData(GL_ARRAY_BUFFER, sizeof (slider), slider, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof (float), (void*) 0);
        glEnableVertexAttribArray(0);
}

void drawParticles(struct flock *f, struct massSpringSystem *mss) {
    fillArray(mss, vertices);
    transformArr(f, triangles);

//    printf("%f\t%f\n",triangles[0],triangles[1]);
    // show Boids
    glUseProgram(dOrangeProgram);
    glBindVertexArray(VAO[0]);
glDrawArrays(GL_TRIANGLES, 0, 3 * nb);
    glUseProgram(greenProgram);
    glBindVertexArray(VAO[1]);
    glPointSize(10);
    glDrawArrays(GL_POINTS, 0, nnodes);

    //draw sliders
    glUseProgram(mOrangeProgram);
    sliderVertices(slider);
    glBindVertexArray(VAO[7]);
    glDrawArrays(GL_TRIANGLES, 0, 6 * nslider);
}

void drawHistogram(float base[], float side1[], float side2[], float top[], struct flock *f) {
    if (toggleMode == 0) {
        p_createBars(base, side1, side2, top, f);
    } else {
        v_createBars(base, side1, side2, top, f);
    }
    quadrants(bounds);

    //draw base
    glUseProgram(dOrangeProgram);
    glBindVertexArray(VAO[2]); // seeing as we only have a single VAO there's no need to bind it every time, but we'll do so to keep things a bit more organized

    glDrawArrays(GL_TRIANGLES, 0, 6 * bins * bins);

    //draw sides
    glUseProgram(lOrangeProgram);
    glBindVertexArray(VAO[3]);
    glDrawArrays(GL_TRIANGLES, 0, 12 * bins * bins);
    glUseProgram(mOrangeProgram);
    glBindVertexArray(VAO[4]);
    glDrawArrays(GL_TRIANGLES, 0, 12 * bins * bins);
    glUseProgram(dOrangeProgram);
    glBindVertexArray(VAO[5]);
    glDrawArrays(GL_TRIANGLES, 0, 6 * bins * bins);

    //draw quadrants
    glUseProgram(greenProgram);
    //glLineWidth(100);
    glBindVertexArray(VAO[6]);
    glDrawArrays(GL_LINES, 0, 4);
}
#endif /* DISPLAY_H */

