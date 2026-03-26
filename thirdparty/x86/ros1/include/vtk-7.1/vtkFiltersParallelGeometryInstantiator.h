#ifndef __vtkFiltersParallelGeometryInstantiator_h
#define __vtkFiltersParallelGeometryInstantiator_h

#include "vtkInstantiator.h"

#include "vtkFiltersParallelGeometryModule.h"


class VTKFILTERSPARALLELGEOMETRY_EXPORT vtkFiltersParallelGeometryInstantiator
{
  public:
  vtkFiltersParallelGeometryInstantiator();
  ~vtkFiltersParallelGeometryInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkFiltersParallelGeometryInstantiator vtkFiltersParallelGeometryInstantiatorInitializer;

#endif
