#ifndef __vtkParallelMPIInstantiator_h
#define __vtkParallelMPIInstantiator_h

#include "vtkInstantiator.h"

#include "vtkParallelMPIModule.h"


class VTKPARALLELMPI_EXPORT vtkParallelMPIInstantiator
{
  public:
  vtkParallelMPIInstantiator();
  ~vtkParallelMPIInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkParallelMPIInstantiator vtkParallelMPIInstantiatorInitializer;

#endif
