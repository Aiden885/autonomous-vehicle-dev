#ifndef __vtkIOParallelNetCDFInstantiator_h
#define __vtkIOParallelNetCDFInstantiator_h

#include "vtkInstantiator.h"

#include "vtkIOParallelNetCDFModule.h"


class VTKIOPARALLELNETCDF_EXPORT vtkIOParallelNetCDFInstantiator
{
  public:
  vtkIOParallelNetCDFInstantiator();
  ~vtkIOParallelNetCDFInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkIOParallelNetCDFInstantiator vtkIOParallelNetCDFInstantiatorInitializer;

#endif
