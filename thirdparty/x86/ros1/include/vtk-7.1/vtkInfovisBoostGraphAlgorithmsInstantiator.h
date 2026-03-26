#ifndef __vtkInfovisBoostGraphAlgorithmsInstantiator_h
#define __vtkInfovisBoostGraphAlgorithmsInstantiator_h

#include "vtkInstantiator.h"

#include "vtkInfovisBoostGraphAlgorithmsModule.h"


class VTKINFOVISBOOSTGRAPHALGORITHMS_EXPORT vtkInfovisBoostGraphAlgorithmsInstantiator
{
  public:
  vtkInfovisBoostGraphAlgorithmsInstantiator();
  ~vtkInfovisBoostGraphAlgorithmsInstantiator();
  private:
  static void ClassInitialize();
  static void ClassFinalize();
  static unsigned int Count;
};

static vtkInfovisBoostGraphAlgorithmsInstantiator vtkInfovisBoostGraphAlgorithmsInstantiatorInitializer;

#endif
