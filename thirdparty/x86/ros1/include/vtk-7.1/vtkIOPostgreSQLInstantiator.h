#ifndef __vtkIOPostgreSQLInstantiator_h
#define __vtkIOPostgreSQLInstantiator_h

#include "vtkInstantiator.h"

#include "vtkIOPostgreSQLModule.h"


class VTKIOPOSTGRESQL_EXPORT vtkIOPostgreSQLInstantiator
{
  public:
  vtkIOPostgreSQLInstantiator();
  ~vtkIOPostgreSQLInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkIOPostgreSQLInstantiator vtkIOPostgreSQLInstantiatorInitializer;

#endif
