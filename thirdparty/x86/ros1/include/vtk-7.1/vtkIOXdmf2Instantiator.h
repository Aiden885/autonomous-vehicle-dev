#ifndef __vtkIOXdmf2Instantiator_h
#define __vtkIOXdmf2Instantiator_h

#include "vtkInstantiator.h"

#include "vtkIOXdmf2Module.h"


class VTKIOXDMF2_EXPORT vtkIOXdmf2Instantiator
{
  public:
  vtkIOXdmf2Instantiator();
  ~vtkIOXdmf2Instantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkIOXdmf2Instantiator vtkIOXdmf2InstantiatorInitializer;

#endif
