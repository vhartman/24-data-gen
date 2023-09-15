#include "line.h"

void Lines::glDraw(OpenGL &gl) {
  if (X.d0 > 0) {
    glColor(1., 1., 1.);
    glLineWidth(2.f);
    glBegin(GL_LINES);
    for (uint i = 0; i < X.d0 - 1; i++) {
      glVertex3dv(&(X[i](0)));
      glVertex3dv(&(X[i + 1](0)));
    }
    glEnd();
    glLineWidth(2.f);
  }
}
