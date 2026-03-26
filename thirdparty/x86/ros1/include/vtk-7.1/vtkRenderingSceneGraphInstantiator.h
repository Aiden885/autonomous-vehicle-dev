#ifndef __vtkRenderingSceneGraphInstantiator_h
#define __vtkRenderingSceneGraphInstantiator_h

#include "vtkInstantiator.h"

#include "vtkRenderingSceneGraphModule.h"


class VTKRENDERINGSCENEGRAPH_EXPORT vtkRenderingSceneGraphInstantiator
{
  public:
  vtkRenderingSceneGraphInstantiator();
  ~vtkRenderingSceneGraphInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkRenderingSceneGraphInstantiator vtkRenderingSceneGraphInstantiatorInitializer;

#endif
