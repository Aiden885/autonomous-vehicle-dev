#ifndef __vtkFiltersParallelMPIInstantiator_h
#define __vtkFiltersParallelMPIInstantiator_h

#include "vtkInstantiator.h"

#include "vtkFiltersParallelMPIModule.h"


class VTKFILTERSPARALLELMPI_EXPORT vtkFiltersParallelMPIInstantiator
{
  public:
  vtkFiltersParallelMPIInstantiator();
  ~vtkFiltersParallelMPIInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkFiltersParallelMPIInstantiator vtkFiltersParallelMPIInstantiatorInitializer;

#endif
