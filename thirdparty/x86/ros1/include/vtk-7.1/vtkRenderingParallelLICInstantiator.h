#ifndef __vtkRenderingParallelLICInstantiator_h
#define __vtkRenderingParallelLICInstantiator_h

#include "vtkInstantiator.h"

#include "vtkRenderingParallelLICModule.h"


class VTKRENDERINGPARALLELLIC_EXPORT vtkRenderingParallelLICInstantiator
{
  public:
  vtkRenderingParallelLICInstantiator();
  ~vtkRenderingParallelLICInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkRenderingParallelLICInstantiator vtkRenderingParallelLICInstantiatorInitializer;

#endif
