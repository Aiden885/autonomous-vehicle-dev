#ifndef __vtkIOVPICInstantiator_h
#define __vtkIOVPICInstantiator_h

#include "vtkInstantiator.h"

#include "vtkIOVPICModule.h"


class VTKIOVPIC_EXPORT vtkIOVPICInstantiator
{
  public:
  vtkIOVPICInstantiator();
  ~vtkIOVPICInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkIOVPICInstantiator vtkIOVPICInstantiatorInitializer;

#endif
