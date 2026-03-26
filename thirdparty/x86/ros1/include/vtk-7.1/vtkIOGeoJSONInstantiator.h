#ifndef __vtkIOGeoJSONInstantiator_h
#define __vtkIOGeoJSONInstantiator_h

#include "vtkInstantiator.h"

#include "vtkIOGeoJSONModule.h"


class VTKIOGEOJSON_EXPORT vtkIOGeoJSONInstantiator
{
  public:
  vtkIOGeoJSONInstantiator();
  ~vtkIOGeoJSONInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkIOGeoJSONInstantiator vtkIOGeoJSONInstantiatorInitializer;

#endif
