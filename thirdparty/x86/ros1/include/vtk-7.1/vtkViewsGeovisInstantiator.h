#ifndef __vtkViewsGeovisInstantiator_h
#define __vtkViewsGeovisInstantiator_h

#include "vtkInstantiator.h"

#include "vtkViewsGeovisModule.h"


class VTKVIEWSGEOVIS_EXPORT vtkViewsGeovisInstantiator
{
  public:
  vtkViewsGeovisInstantiator();
  ~vtkViewsGeovisInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkViewsGeovisInstantiator vtkViewsGeovisInstantiatorInitializer;

#endif
