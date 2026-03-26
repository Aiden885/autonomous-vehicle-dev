#ifndef __vtkRenderingOpenGL2Instantiator_h
#define __vtkRenderingOpenGL2Instantiator_h

#include "vtkInstantiator.h"

#include "vtkRenderingOpenGL2Module.h"


class VTKRENDERINGOPENGL2_EXPORT vtkRenderingOpenGL2Instantiator
{
  public:
  vtkRenderingOpenGL2Instantiator();
  ~vtkRenderingOpenGL2Instantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkRenderingOpenGL2Instantiator vtkRenderingOpenGL2InstantiatorInitializer;

#endif
