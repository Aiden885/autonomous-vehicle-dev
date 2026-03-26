#ifndef __vtkFiltersPythonInstantiator_h
#define __vtkFiltersPythonInstantiator_h

#include "vtkInstantiator.h"

#include "vtkFiltersPythonModule.h"


class VTKFILTERSPYTHON_EXPORT vtkFiltersPythonInstantiator
{
  public:
  vtkFiltersPythonInstantiator();
  ~vtkFiltersPythonInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkFiltersPythonInstantiator vtkFiltersPythonInstantiatorInitializer;

#endif
