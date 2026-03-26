#ifndef __vtkFiltersParallelDIY2Instantiator_h
#define __vtkFiltersParallelDIY2Instantiator_h

#include "vtkInstantiator.h"

#include "vtkFiltersParallelDIY2Module.h"


class VTKFILTERSPARALLELDIY2_EXPORT vtkFiltersParallelDIY2Instantiator
{
  public:
  vtkFiltersParallelDIY2Instantiator();
  ~vtkFiltersParallelDIY2Instantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkFiltersParallelDIY2Instantiator vtkFiltersParallelDIY2InstantiatorInitializer;

#endif
