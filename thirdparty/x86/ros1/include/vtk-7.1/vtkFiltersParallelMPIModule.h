
#ifndef VTKFILTERSPARALLELMPI_EXPORT_H
#define VTKFILTERSPARALLELMPI_EXPORT_H

#ifdef VTKFILTERSPARALLELMPI_STATIC_DEFINE
#  define VTKFILTERSPARALLELMPI_EXPORT
#  define VTKFILTERSPARALLELMPI_NO_EXPORT
#else
#  ifndef VTKFILTERSPARALLELMPI_EXPORT
#    ifdef vtkFiltersParallelMPI_EXPORTS
        /* We are building this library */
#      define VTKFILTERSPARALLELMPI_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VTKFILTERSPARALLELMPI_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VTKFILTERSPARALLELMPI_NO_EXPORT
#    define VTKFILTERSPARALLELMPI_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VTKFILTERSPARALLELMPI_DEPRECATED
#  define VTKFILTERSPARALLELMPI_DEPRECATED __attribute__ ((__deprecated__))
#  define VTKFILTERSPARALLELMPI_DEPRECATED_EXPORT VTKFILTERSPARALLELMPI_EXPORT __attribute__ ((__deprecated__))
#  define VTKFILTERSPARALLELMPI_DEPRECATED_NO_EXPORT VTKFILTERSPARALLELMPI_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKFILTERSPARALLELMPI_NO_DEPRECATED
#endif

/* AutoInit dependencies.  */
#include "vtkFiltersExtractionModule.h"

#endif
