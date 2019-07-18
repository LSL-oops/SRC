cmake_minimum_required(VERSION 3.12)
project(SOFT_BODY_CONTROL)

set(CMAKE_CXX_STANDARD 14)
#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

set(VEGA_HEADER_DIR "./include/")
set(VEGA_LIB_DIR "./lib")

include_directories(${VEGA_HEADER_DIR})
link_directories(${VEGA_LIB_DIR})

add_executable(demo demo.cpp glad.c SoftBodyController.cpp SoftBodyController.h SoftBodySimulator.cpp SoftBodySimulator.h configHeader.h Shader.h Camera.h IsotropicHyperelasticFEMWithRestShapeControl.cpp IsotropicHyperelasticFEMWithRestShapeControl.h implicitBackwardEulerSparseWithRestShapeControl.cpp implicitBackwardEulerSparseWithRestShapeControl.h pyBindInterface.cpp pyBindInterface.h RestShapeController.cpp RestShapeController.h)

target_link_libraries(demo volumetricMesh mesh objMesh basicAlgorithms
        sparseMatrix constrainedDOFs corotationalLinearFEM forceModel getopts integrator integratorSparse
        isotropicHyperelasticFEM polarDecomposition performanceCounter sceneObject
        sparseMatrix sparseSolver stencilForceModel stvk  minivector glfw dl ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

target_link_libraries(demo glfw dl ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${LIBS})

# add_library(softBodyControl )
