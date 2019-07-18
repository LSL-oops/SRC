#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "SoftBodyController.h"
#include "SoftBodySimulator.h"
#include "configHeader.h"
#include "Camera.h"
#include "Shader.h"
#include "volumetricMeshLoader.h"
#include "generateSurfaceMesh.h"
#include "objMesh.h"

const char * config_filename = "./beam_tet.config";
GLFWwindow *window;

int initWindowSystem();

void framebuffer_size_callback(GLFWwindow *window,int width,int height);
void scroll_callback(GLFWwindow *window,double xoffset,double yoffset);
void mouse_callback(GLFWwindow *window,double xpos,double ypos);
void processInput(GLFWwindow* window);
void loadingObjectData(const char *filename,GLdouble **p_vertices,GLuint **p_elements,size_t *NV,size_t *NF);

const unsigned int WINDOW_WIDTH = 800;
const unsigned int WINDOW_HEIGHT = 600;

Camera camera(glm::vec3(0.0f,0.5f,2.0f));
float lastX = WINDOW_WIDTH / 2.0;
float lastY = WINDOW_HEIGHT / 2.0;
bool firstMouse = true;

float deltaTime = 0.0f;
float lastFrame = 0.0f;

glm::vec3 lightPos(1.2f,1.0f,2.0f);

double scroll_pos_cur,scroll_pos_pre;
bool first_time_scroll = true;

SoftBodyController *sbc;

GLuint VBOs[2],VAOs[2],EBO;
size_t number_vertices = 0;
size_t number_facets = 0;
GLdouble *vertex_buffer;
GLuint *facet_buffer;

VolumetricMesh *vm;

enum mesh{REST,DEFORMED};

int main() {
    if(!initWindowSystem()){
        std::cerr << "FAILED INITIALIZAING GLFW WINDOW" << std::endl;
        return -1;
    }

    if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "FAILED INITIALIZING GLAD WINDOW" << std::endl;
        glfwTerminate();
        return -1;
    }


    glEnable(GL_DEPTH_TEST);
    Shader lightingShader("shader/lighting.vs","./shader/lighting.fs");

    loadingObjectData("model/beam3_tet.veg",&vertex_buffer,&facet_buffer,&number_vertices,&number_facets);

    std::cout << "NUMBER_VERTICES : " << number_vertices << std::endl;
    std::cout << "NUMBER_FACETS : " << number_facets << std::endl;
/*
    std::cout << "VERTEX::" << std::endl;
    for(int i = 0;i < number_vertices;++i){
        std::cout << vertex_buffer[i*3 + 0] << " "\
                    << vertex_buffer[i*3 + 1] << " "\
                    << vertex_buffer[i*3 + 2] << std::endl;
    }

    std::cout << "FACET::" << std::endl;
    for(int i = 0;i < number_facets;++i){
        std::cout << facet_buffer[i*3 + 0] << " "\
                    << facet_buffer[i*3 + 1] << " "\
                    << facet_buffer[i*3 + 2] << std::endl;
    }
*/
    glGenBuffers(1,&EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,number_facets*3*sizeof(GLuint),facet_buffer,GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);

    glGenVertexArrays(2,VAOs);
    glGenBuffers(2,VBOs);


    glBindVertexArray(VAOs[REST]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,EBO);
    glBindBuffer(GL_ARRAY_BUFFER,VBOs[REST]);
    glBufferData(GL_ARRAY_BUFFER,number_vertices*3*sizeof(GLdouble),vertex_buffer,GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0,3,GL_DOUBLE,GL_FALSE,3*sizeof(GLdouble),(void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    glBindVertexArray(VAOs[DEFORMED]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,EBO);
    glBindBuffer(GL_ARRAY_BUFFER,VBOs[DEFORMED]);
    glBufferData(GL_ARRAY_BUFFER,number_vertices*3*sizeof(GLdouble),vertex_buffer,GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0,3,GL_DOUBLE,GL_FALSE,3*sizeof(GLdouble),(void*)0);
    glEnableVertexAttribArray(0);



    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);


    while(!glfwWindowShouldClose(window))
    {
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
        processInput(window);

        glClearColor(1.0f,1.0f,1.0f,1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        lightingShader.use();

        // view/projection transformations
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), \
                               (float)WINDOW_WIDTH / (float)WINDOW_HEIGHT, 0.1f, 100.0f);
        glm::mat4 view = camera.GetViewMatrix();
        lightingShader.setMat4("projection", projection);
        lightingShader.setMat4("view", view);
        glm::mat4 model = glm::mat4(1.0f);


        sbc->DoTimeStep();
        sbc->updateGPUVertexBuffer(VBOs);


        glBindVertexArray(VAOs[REST]);
        model = glm::translate(model,glm::vec3(0.2,0.0,0.0));
        lightingShader.setMat4("model", model);
        glDrawElements(GL_TRIANGLES, number_facets*3,GL_UNSIGNED_INT,0);

        glBindVertexArray(VAOs[DEFORMED]);
        model = glm::mat4(1.0f);
        model = glm::translate(model,glm::vec3(-0.2,0.0,0.0));
        lightingShader.setMat4("model", model);
        glDrawElements(GL_TRIANGLES, number_facets*3,GL_UNSIGNED_INT,0);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}

int initWindowSystem()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
    glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OENGL_FORWARD_COMPAT,GL_TRUE);
#endif

    window = glfwCreateWindow(WINDOW_WIDTH,WINDOW_HEIGHT,"SOFT_BODY_CONTROL",NULL,NULL);
    if(window == NULL){
        glfwTerminate();
        return 0;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window,framebuffer_size_callback);
    glfwSetScrollCallback(window,scroll_callback);
    glfwSetCursorPosCallback(window,mouse_callback);

    return 1;
}

void deinitControlSystem(){
    free(sbc);
}

void framebuffer_size_callback(GLFWwindow *window,int width,int height){
    glViewport(0,0,width,height);
}

void scroll_callback(GLFWwindow *window,double xoffset,double yoffset){
    if(first_time_scroll)
    {
        scroll_pos_cur = yoffset;
        scroll_pos_pre = yoffset;
        first_time_scroll = false;
    }
    else{
        scroll_pos_pre = scroll_pos_cur;
        scroll_pos_cur = yoffset;
    }

    sbc->scalingRestShape(true);
    return;
}

void mouse_callback(GLFWwindow *window,double xpos,double ypos){
    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

   // camera.ProcessMouseMovement(xoffset, yoffset);
}

void processInput(GLFWwindow* window){
    if(glfwGetKey(window,GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window,true);
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
    if(glfwGetKey(window,GLFW_KEY_UP) == GLFW_PRESS)
        sbc->scalingRestShape(true);
    if(glfwGetKey(window,GLFW_KEY_DOWN) == GLFW_PRESS)
        sbc->scalingRestShape(false);
}

void loadingObjectData(const char *filename,GLdouble **p_vertices,GLuint **p_elements,size_t *NV,size_t *NF){
    vm = VolumetricMeshLoader::load(filename,VolumetricMesh::ASCII);
    ObjMesh *om = GenerateSurfaceMesh::ComputeMesh(vm);
    *NV = vm->getNumVertices();
    *NF = om->getNumFaces();

    *p_vertices = (GLdouble*)malloc(sizeof(GLdouble)*(*NV)*3);
    *p_elements = (GLuint*)malloc(sizeof(GLuint)*(*NF)*3);

    int f_index = 0;
    om->forEachFace([=](int gID,int fId,ObjMesh::Face &f){
        (*p_elements)[fId*3 + 0] = f.getVertexPositionIndex(0);
        (*p_elements)[fId*3 + 1] = f.getVertexPositionIndex(1);
        (*p_elements)[fId*3 + 2] = f.getVertexPositionIndex(2);
    });

    for(int i = 0;i < *NV;++i){
        Vec3d v = vm->getVertex(i);
        (*p_vertices)[i*3 + 0] = v[0];
        (*p_vertices)[i*3 + 1] = v[1];
        (*p_vertices)[i*3 + 2] = v[2];
    }

    sbc = new SoftBodyController(vm);
    const Vec3d* p_v = reinterpret_cast<const Vec3d*>(*p_vertices);
    const Vec3i* p_f = reinterpret_cast<const Vec3i*>(*p_elements);
    std::cout << "Output The Data:" << std::endl;
    std::cout << "VERTEX:" << std::endl;

    for(int i = 0;i < *NV;++i)
        std::cout << p_v[i] << std::endl;

    std::cout << "Facet:" << std::endl;
    for(int i = 0;i < *NF;++i)
        std::cout << p_f[i] << std::endl;

}


