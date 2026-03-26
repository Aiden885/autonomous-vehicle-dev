#ifndef __vtkIOMPIParallelInstantiator_h
#define __vtkIOMPIParallelInstantiator_h

#include "vtkInstantiator.h"

#include "vtkIOMPIParallelModule.h"


class VTKIOMPIPARALLEL_EXPORT vtkIOMPIParallelInstantiator
{
  public:
  vtkIOMPIParallelInstantiator();
  ~vtkIOMPIParallelInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkIOMPIParallelInstantiator vtkIOMPIParallelInstantiatorInitializer;

#endif
