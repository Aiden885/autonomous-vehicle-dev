#ifndef __vtkRenderingGL2PSOpenGL2Instantiator_h
#define __vtkRenderingGL2PSOpenGL2Instantiator_h

#include "vtkInstantiator.h"

#include "vtkRenderingGL2PSOpenGL2Module.h"


class VTKRENDERINGGL2PSOPENGL2_EXPORT vtkRenderingGL2PSOpenGL2Instantiator
{
  public:
  vtkRenderingGL2PSOpenGL2Instantiator();
  ~vtkRenderingGL2PSOpenGL2Instantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkRenderingGL2PSOpenGL2Instantiator vtkRenderingGL2PSOpenGL2InstantiatorInitializer;

#endif
