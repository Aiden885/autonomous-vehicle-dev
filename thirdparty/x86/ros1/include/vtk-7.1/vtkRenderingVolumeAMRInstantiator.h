#ifndef __vtkRenderingVolumeAMRInstantiator_h
#define __vtkRenderingVolumeAMRInstantiator_h

#include "vtkInstantiator.h"

#include "vtkRenderingVolumeAMRModule.h"


class VTKRENDERINGVOLUMEAMR_EXPORT vtkRenderingVolumeAMRInstantiator
{
  public:
  vtkRenderingVolumeAMRInstantiator();
  ~vtkRenderingVolumeAMRInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkRenderingVolumeAMRInstantiator vtkRenderingVolumeAMRInstantiatorInitializer;

#endif
