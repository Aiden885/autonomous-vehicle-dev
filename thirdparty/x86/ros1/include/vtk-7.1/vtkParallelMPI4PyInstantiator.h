#ifndef __vtkParallelMPI4PyInstantiator_h
#define __vtkParallelMPI4PyInstantiator_h

#include "vtkInstantiator.h"

#include "vtkParallelMPI4PyModule.h"


class VTKPARALLELMPI4PY_EXPORT vtkParallelMPI4PyInstantiator
{
  public:
  vtkParallelMPI4PyInstantiator();
  ~vtkParallelMPI4PyInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkParallelMPI4PyInstantiator vtkParallelMPI4PyInstantiatorInitializer;

#endif
