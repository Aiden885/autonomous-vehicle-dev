
#ifndef VTKFILTERSPARALLELDIY2_EXPORT_H
#define VTKFILTERSPARALLELDIY2_EXPORT_H

#ifdef VTKFILTERSPARALLELDIY2_STATIC_DEFINE
#  define VTKFILTERSPARALLELDIY2_EXPORT
#  define VTKFILTERSPARALLELDIY2_NO_EXPORT
#else
#  ifndef VTKFILTERSPARALLELDIY2_EXPORT
#    ifdef vtkFiltersParallelDIY2_EXPORTS
        /* We are building this library */
#      define VTKFILTERSPARALLELDIY2_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VTKFILTERSPARALLELDIY2_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VTKFILTERSPARALLELDIY2_NO_EXPORT
#    define VTKFILTERSPARALLELDIY2_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VTKFILTERSPARALLELDIY2_DEPRECATED
#  define VTKFILTERSPARALLELDIY2_DEPRECATED __attribute__ ((__deprecated__))
#  define VTKFILTERSPARALLELDIY2_DEPRECATED_EXPORT VTKFILTERSPARALLELDIY2_EXPORT __attribute__ ((__deprecated__))
#  define VTKFILTERSPARALLELDIY2_DEPRECATED_NO_EXPORT VTKFILTERSPARALLELDIY2_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKFILTERSPARALLELDIY2_NO_DEPRECATED
#endif

/* AutoInit dependencies.  */
#include "vtkFiltersCoreModule.h"
#include "vtkFiltersCoreModule.h"

/* AutoInit implementations.  */
#if defined(vtkFiltersParallelDIY2_INCLUDE)
# include vtkFiltersParallelDIY2_INCLUDE
#endif
#if defined(vtkFiltersParallelDIY2_AUTOINIT)
# include "vtkAutoInit.h"
VTK_AUTOINIT(vtkFiltersParallelDIY2)
#endif

#endif
