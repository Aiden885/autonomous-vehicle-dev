#ifndef __vtkFiltersParallelFlowPathsInstantiator_h
#define __vtkFiltersParallelFlowPathsInstantiator_h

#include "vtkInstantiator.h"

#include "vtkFiltersParallelFlowPathsModule.h"


class VTKFILTERSPARALLELFLOWPATHS_EXPORT vtkFiltersParallelFlowPathsInstantiator
{
  public:
  vtkFiltersParallelFlowPathsInstantiator();
  ~vtkFiltersParallelFlowPathsInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkFiltersParallelFlowPathsInstantiator vtkFiltersParallelFlowPathsInstantiatorInitializer;

#endif
