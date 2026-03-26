#ifndef __vtkIOMySQLInstantiator_h
#define __vtkIOMySQLInstantiator_h

#include "vtkInstantiator.h"

#include "vtkIOMySQLModule.h"


class VTKIOMYSQL_EXPORT vtkIOMySQLInstantiator
{
  public:
  vtkIOMySQLInstantiator();
  ~vtkIOMySQLInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkIOMySQLInstantiator vtkIOMySQLInstantiatorInitializer;

#endif
