#ifndef __vtkIOParallelExodusInstantiator_h
#define __vtkIOParallelExodusInstantiator_h

#include "vtkInstantiator.h"

#include "vtkIOParallelExodusModule.h"


class VTKIOPARALLELEXODUS_EXPORT vtkIOParallelExodusInstantiator
{
  public:
  vtkIOParallelExodusInstantiator();
  ~vtkIOParallelExodusInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkIOParallelExodusInstantiator vtkIOParallelExodusInstantiatorInitializer;

#endif
