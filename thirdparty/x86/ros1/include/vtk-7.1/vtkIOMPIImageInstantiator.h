#ifndef __vtkIOMPIImageInstantiator_h
#define __vtkIOMPIImageInstantiator_h

#include "vtkInstantiator.h"

#include "vtkIOMPIImageModule.h"


class VTKIOMPIIMAGE_EXPORT vtkIOMPIImageInstantiator
{
  public:
  vtkIOMPIImageInstantiator();
  ~vtkIOMPIImageInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkIOMPIImageInstantiator vtkIOMPIImageInstantiatorInitializer;

#endif
