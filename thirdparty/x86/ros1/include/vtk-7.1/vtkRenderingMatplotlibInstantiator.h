#ifndef __vtkRenderingMatplotlibInstantiator_h
#define __vtkRenderingMatplotlibInstantiator_h

#include "vtkInstantiator.h"

#include "vtkRenderingMatplotlibModule.h"


class VTKRENDERINGMATPLOTLIB_EXPORT vtkRenderingMatplotlibInstantiator
{
  public:
  vtkRenderingMatplotlibInstantiator();
  ~vtkRenderingMatplotlibInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkRenderingMatplotlibInstantiator vtkRenderingMatplotlibInstantiatorInitializer;

#endif
