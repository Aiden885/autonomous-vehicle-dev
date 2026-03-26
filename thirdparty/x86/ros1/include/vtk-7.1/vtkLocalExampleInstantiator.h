#ifndef __vtkLocalExampleInstantiator_h
#define __vtkLocalExampleInstantiator_h

#include "vtkInstantiator.h"

#include "vtkLocalExampleModule.h"


class VTKLOCALEXAMPLE_EXPORT vtkLocalExampleInstantiator
{
  public:
  vtkLocalExampleInstantiator();
  ~vtkLocalExampleInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkLocalExampleInstantiator vtkLocalExampleInstantiatorInitializer;

#endif
