#include <GL/gl.h>
#include <Gui/opengl.h>

#include <KOMO/komo.h>
#include <Kin/F_operators.h>
#include <Kin/F_pose.h>
#include <Kin/F_qFeatures.h>
#include <Kin/featureSymbols.h>
#include <Kin/kin.h>

struct Lines : GLDrawer {
  arr &X;
  Lines(arr &_X) : X(_X){};

  void glDraw(OpenGL &gl);
};
