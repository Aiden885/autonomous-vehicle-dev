#ifndef __vtkRenderingExternalInstantiator_h
#define __vtkRenderingExternalInstantiator_h

#include "vtkInstantiator.h"

#include "vtkRenderingExternalModule.h"


class VTKRENDERINGEXTERNAL_EXPORT vtkRenderingExternalInstantiator
{
  public:
  vtkRenderingExternalInstantiator();
  ~vtkRenderingExternalInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkRenderingExternalInstantiator vtkRenderingExternalInstantiatorInitializer;

#endif
