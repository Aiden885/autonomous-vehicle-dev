#ifndef __vtkFiltersParallelStatisticsInstantiator_h
#define __vtkFiltersParallelStatisticsInstantiator_h

#include "vtkInstantiator.h"

#include "vtkFiltersParallelStatisticsModule.h"


class VTKFILTERSPARALLELSTATISTICS_EXPORT vtkFiltersParallelStatisticsInstantiator
{
  public:
  vtkFiltersParallelStatisticsInstantiator();
  ~vtkFiltersParallelStatisticsInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkFiltersParallelStatisticsInstantiator vtkFiltersParallelStatisticsInstantiatorInitializer;

#endif
