#ifndef __vtkIOTecplotTableInstantiator_h
#define __vtkIOTecplotTableInstantiator_h

#include "vtkInstantiator.h"

#include "vtkIOTecplotTableModule.h"


class VTKIOTECPLOTTABLE_EXPORT vtkIOTecplotTableInstantiator
{
  public:
  vtkIOTecplotTableInstantiator();
  ~vtkIOTecplotTableInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkIOTecplotTableInstantiator vtkIOTecplotTableInstantiatorInitializer;

#endif
