#ifndef __vtkDomainsChemistryOpenGL2Instantiator_h
#define __vtkDomainsChemistryOpenGL2Instantiator_h

#include "vtkInstantiator.h"

#include "vtkDomainsChemistryOpenGL2Module.h"


class VTKDOMAINSCHEMISTRYOPENGL2_EXPORT vtkDomainsChemistryOpenGL2Instantiator
{
  public:
  vtkDomainsChemistryOpenGL2Instantiator();
  ~vtkDomainsChemistryOpenGL2Instantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkDomainsChemistryOpenGL2Instantiator vtkDomainsChemistryOpenGL2InstantiatorInitializer;

#endif
