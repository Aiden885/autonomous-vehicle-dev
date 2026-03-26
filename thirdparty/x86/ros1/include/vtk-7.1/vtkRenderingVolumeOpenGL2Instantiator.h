#ifndef __vtkRenderingVolumeOpenGL2Instantiator_h
#define __vtkRenderingVolumeOpenGL2Instantiator_h

#include "vtkInstantiator.h"

#include "vtkRenderingVolumeOpenGL2Module.h"


class VTKRENDERINGVOLUMEOPENGL2_EXPORT vtkRenderingVolumeOpenGL2Instantiator
{
  public:
  vtkRenderingVolumeOpenGL2Instantiator();
  ~vtkRenderingVolumeOpenGL2Instantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkRenderingVolumeOpenGL2Instantiator vtkRenderingVolumeOpenGL2InstantiatorInitializer;

#endif
