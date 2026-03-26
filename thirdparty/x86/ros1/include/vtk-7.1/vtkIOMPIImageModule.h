
#ifndef VTKIOMPIIMAGE_EXPORT_H
#define VTKIOMPIIMAGE_EXPORT_H

#ifdef VTKIOMPIIMAGE_STATIC_DEFINE
#  define VTKIOMPIIMAGE_EXPORT
#  define VTKIOMPIIMAGE_NO_EXPORT
#else
#  ifndef VTKIOMPIIMAGE_EXPORT
#    ifdef vtkIOMPIImage_EXPORTS
        /* We are building this library */
#      define VTKIOMPIIMAGE_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VTKIOMPIIMAGE_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VTKIOMPIIMAGE_NO_EXPORT
#    define VTKIOMPIIMAGE_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VTKIOMPIIMAGE_DEPRECATED
#  define VTKIOMPIIMAGE_DEPRECATED __attribute__ ((__deprecated__))
#  define VTKIOMPIIMAGE_DEPRECATED_EXPORT VTKIOMPIIMAGE_EXPORT __attribute__ ((__deprecated__))
#  define VTKIOMPIIMAGE_DEPRECATED_NO_EXPORT VTKIOMPIIMAGE_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKIOMPIIMAGE_NO_DEPRECATED
#endif

/* AutoInit dependencies.  */
#include "vtkIOImageModule.h"
#include "vtkIOImageModule.h"

/* AutoInit implementations.  */
#if defined(vtkIOMPIImage_INCLUDE)
# include vtkIOMPIImage_INCLUDE
#endif
#if defined(vtkIOMPIImage_AUTOINIT)
# include "vtkAutoInit.h"
VTK_AUTOINIT(vtkIOMPIImage)
#endif

#endif
