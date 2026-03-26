#ifndef __vtkIOParallelLSDynaInstantiator_h
#define __vtkIOParallelLSDynaInstantiator_h

#include "vtkInstantiator.h"

#include "vtkIOParallelLSDynaModule.h"


class VTKIOPARALLELLSDYNA_EXPORT vtkIOParallelLSDynaInstantiator
{
  public:
  vtkIOParallelLSDynaInstantiator();
  ~vtkIOParallelLSDynaInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkIOParallelLSDynaInstantiator vtkIOParallelLSDynaInstantiatorInitializer;

#endif
