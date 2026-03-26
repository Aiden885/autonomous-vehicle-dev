#ifndef __vtkIOODBCInstantiator_h
#define __vtkIOODBCInstantiator_h

#include "vtkInstantiator.h"

#include "vtkIOODBCModule.h"


class VTKIOODBC_EXPORT vtkIOODBCInstantiator
{
  public:
  vtkIOODBCInstantiator();
  ~vtkIOODBCInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkIOODBCInstantiator vtkIOODBCInstantiatorInitializer;

#endif
