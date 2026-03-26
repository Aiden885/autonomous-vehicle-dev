#ifndef __vtkFiltersPointsInstantiator_h
#define __vtkFiltersPointsInstantiator_h

#include "vtkInstantiator.h"

#include "vtkFiltersPointsModule.h"


class VTKFILTERSPOINTS_EXPORT vtkFiltersPointsInstantiator
{
  public:
  vtkFiltersPointsInstantiator();
  ~vtkFiltersPointsInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkFiltersPointsInstantiator vtkFiltersPointsInstantiatorInitializer;

#endif
