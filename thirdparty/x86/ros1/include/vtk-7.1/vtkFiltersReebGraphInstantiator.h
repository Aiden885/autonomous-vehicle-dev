#ifndef __vtkFiltersReebGraphInstantiator_h
#define __vtkFiltersReebGraphInstantiator_h

#include "vtkInstantiator.h"

#include "vtkFiltersReebGraphModule.h"


class VTKFILTERSREEBGRAPH_EXPORT vtkFiltersReebGraphInstantiator
{
  public:
  vtkFiltersReebGraphInstantiator();
  ~vtkFiltersReebGraphInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkFiltersReebGraphInstantiator vtkFiltersReebGraphInstantiatorInitializer;

#endif
