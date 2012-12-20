noinst_PROGRAMS=BvhBugDemo

BvhBugDemo_SOURCES=BvhBugDemo.cpp BvhBugDemo.h main.cpp
BvhBugDemo_CXXFLAGS=-I@top_builddir@/src -I@top_builddir@/Demos/OpenGL $(CXXFLAGS)
BvhBugDemo_LDADD=-L../OpenGL -lbulletopenglsupport -L../../src -lBulletDynamics -lBulletCollision -lLinearMath @opengl_LIBS@
