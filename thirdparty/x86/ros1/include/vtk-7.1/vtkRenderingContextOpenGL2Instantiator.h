#ifndef __vtkRenderingContextOpenGL2Instantiator_h
#define __vtkRenderingContextOpenGL2Instantiator_h

#include "vtkInstantiator.h"

#include "vtkRenderingContextOpenGL2Module.h"


class VTKRENDERINGCONTEXTOPENGL2_EXPORT vtkRenderingContextOpenGL2Instantiator
{
  public:
  vtkRenderingContextOpenGL2Instantiator();
  ~vtkRenderingContextOpenGL2Instantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkRenderingContextOpenGL2Instantiator vtkRenderingContextOpenGL2InstantiatorInitializer;

#endif
