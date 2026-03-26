#ifndef __vtkIOGDALInstantiator_h
#define __vtkIOGDALInstantiator_h

#include "vtkInstantiator.h"

#include "vtkIOGDALModule.h"


class VTKIOGDAL_EXPORT vtkIOGDALInstantiator
{
  public:
  vtkIOGDALInstantiator();
  ~vtkIOGDALInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkIOGDALInstantiator vtkIOGDALInstantiatorInitializer;

#endif
