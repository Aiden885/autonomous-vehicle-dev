
#ifndef VTKIOMPIPARALLEL_EXPORT_H
#define VTKIOMPIPARALLEL_EXPORT_H

#ifdef VTKIOMPIPARALLEL_STATIC_DEFINE
#  define VTKIOMPIPARALLEL_EXPORT
#  define VTKIOMPIPARALLEL_NO_EXPORT
#else
#  ifndef VTKIOMPIPARALLEL_EXPORT
#    ifdef vtkIOMPIParallel_EXPORTS
        /* We are building this library */
#      define VTKIOMPIPARALLEL_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VTKIOMPIPARALLEL_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VTKIOMPIPARALLEL_NO_EXPORT
#    define VTKIOMPIPARALLEL_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VTKIOMPIPARALLEL_DEPRECATED
#  define VTKIOMPIPARALLEL_DEPRECATED __attribute__ ((__deprecated__))
#  define VTKIOMPIPARALLEL_DEPRECATED_EXPORT VTKIOMPIPARALLEL_EXPORT __attribute__ ((__deprecated__))
#  define VTKIOMPIPARALLEL_DEPRECATED_NO_EXPORT VTKIOMPIPARALLEL_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKIOMPIPARALLEL_NO_DEPRECATED
#endif

/* AutoInit dependencies.  */
#include "vtkIOGeometryModule.h"
#include "vtkIOGeometryModule.h"
#include "vtkIOParallelModule.h"
#include "vtkIOParallelModule.h"

/* AutoInit implementations.  */
#if defined(vtkIOMPIParallel_INCLUDE)
# include vtkIOMPIParallel_INCLUDE
#endif
#if defined(vtkIOMPIParallel_AUTOINIT)
# include "vtkAutoInit.h"
VTK_AUTOINIT(vtkIOMPIParallel)
#endif

#endif
