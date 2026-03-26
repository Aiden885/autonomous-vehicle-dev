
#ifndef VTKFILTERSPARALLELFLOWPATHS_EXPORT_H
#define VTKFILTERSPARALLELFLOWPATHS_EXPORT_H

#ifdef VTKFILTERSPARALLELFLOWPATHS_STATIC_DEFINE
#  define VTKFILTERSPARALLELFLOWPATHS_EXPORT
#  define VTKFILTERSPARALLELFLOWPATHS_NO_EXPORT
#else
#  ifndef VTKFILTERSPARALLELFLOWPATHS_EXPORT
#    ifdef vtkFiltersParallelFlowPaths_EXPORTS
        /* We are building this library */
#      define VTKFILTERSPARALLELFLOWPATHS_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VTKFILTERSPARALLELFLOWPATHS_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VTKFILTERSPARALLELFLOWPATHS_NO_EXPORT
#    define VTKFILTERSPARALLELFLOWPATHS_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VTKFILTERSPARALLELFLOWPATHS_DEPRECATED
#  define VTKFILTERSPARALLELFLOWPATHS_DEPRECATED __attribute__ ((__deprecated__))
#  define VTKFILTERSPARALLELFLOWPATHS_DEPRECATED_EXPORT VTKFILTERSPARALLELFLOWPATHS_EXPORT __attribute__ ((__deprecated__))
#  define VTKFILTERSPARALLELFLOWPATHS_DEPRECATED_NO_EXPORT VTKFILTERSPARALLELFLOWPATHS_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKFILTERSPARALLELFLOWPATHS_NO_DEPRECATED
#endif

/* AutoInit dependencies.  */
#include "vtkFiltersFlowPathsModule.h"
#include "vtkFiltersFlowPathsModule.h"

/* AutoInit implementations.  */
#if defined(vtkFiltersParallelFlowPaths_INCLUDE)
# include vtkFiltersParallelFlowPaths_INCLUDE
#endif
#if defined(vtkFiltersParallelFlowPaths_AUTOINIT)
# include "vtkAutoInit.h"
VTK_AUTOINIT(vtkFiltersParallelFlowPaths)
#endif

#endif
