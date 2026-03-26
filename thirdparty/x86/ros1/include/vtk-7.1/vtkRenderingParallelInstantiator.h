#ifndef __vtkRenderingParallelInstantiator_h
#define __vtkRenderingParallelInstantiator_h

#include "vtkInstantiator.h"

#include "vtkRenderingParallelModule.h"


class VTKRENDERINGPARALLEL_EXPORT vtkRenderingParallelInstantiator
{
  public:
  vtkRenderingParallelInstantiator();
  ~vtkRenderingParallelInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkRenderingParallelInstantiator vtkRenderingParallelInstantiatorInitializer;

#endif
