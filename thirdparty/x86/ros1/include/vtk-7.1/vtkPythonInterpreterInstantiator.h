#ifndef __vtkPythonInterpreterInstantiator_h
#define __vtkPythonInterpreterInstantiator_h

#include "vtkInstantiator.h"

#include "vtkPythonInterpreterModule.h"


class VTKPYTHONINTERPRETER_EXPORT vtkPythonInterpreterInstantiator
{
  public:
  vtkPythonInterpreterInstantiator();
  ~vtkPythonInterpreterInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkPythonInterpreterInstantiator vtkPythonInterpreterInstantiatorInitializer;

#endif
