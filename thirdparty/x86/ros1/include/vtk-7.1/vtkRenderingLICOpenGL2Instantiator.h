#ifndef __vtkRenderingLICOpenGL2Instantiator_h
#define __vtkRenderingLICOpenGL2Instantiator_h

#include "vtkInstantiator.h"

#include "vtkRenderingLICOpenGL2Module.h"


class VTKRENDERINGLICOPENGL2_EXPORT vtkRenderingLICOpenGL2Instantiator
{
  public:
  vtkRenderingLICOpenGL2Instantiator();
  ~vtkRenderingLICOpenGL2Instantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkRenderingLICOpenGL2Instantiator vtkRenderingLICOpenGL2InstantiatorInitializer;

#endif
