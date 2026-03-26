#ifndef __vtkIOFFMPEGInstantiator_h
#define __vtkIOFFMPEGInstantiator_h

#include "vtkInstantiator.h"

#include "vtkIOFFMPEGModule.h"


class VTKIOFFMPEG_EXPORT vtkIOFFMPEGInstantiator
{
  public:
  vtkIOFFMPEGInstantiator();
  ~vtkIOFFMPEGInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkIOFFMPEGInstantiator vtkIOFFMPEGInstantiatorInitializer;

#endif
