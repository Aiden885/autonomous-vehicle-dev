
#ifndef VTKRENDERINGEXTERNAL_EXPORT_H
#define VTKRENDERINGEXTERNAL_EXPORT_H

#ifdef VTKRENDERINGEXTERNAL_STATIC_DEFINE
#  define VTKRENDERINGEXTERNAL_EXPORT
#  define VTKRENDERINGEXTERNAL_NO_EXPORT
#else
#  ifndef VTKRENDERINGEXTERNAL_EXPORT
#    ifdef vtkRenderingExternal_EXPORTS
        /* We are building this library */
#      define VTKRENDERINGEXTERNAL_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VTKRENDERINGEXTERNAL_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VTKRENDERINGEXTERNAL_NO_EXPORT
#    define VTKRENDERINGEXTERNAL_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VTKRENDERINGEXTERNAL_DEPRECATED
#  define VTKRENDERINGEXTERNAL_DEPRECATED __attribute__ ((__deprecated__))
#  define VTKRENDERINGEXTERNAL_DEPRECATED_EXPORT VTKRENDERINGEXTERNAL_EXPORT __attribute__ ((__deprecated__))
#  define VTKRENDERINGEXTERNAL_DEPRECATED_NO_EXPORT VTKRENDERINGEXTERNAL_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKRENDERINGEXTERNAL_NO_DEPRECATED
#endif

/* AutoInit dependencies.  */
#include "vtkRenderingCoreModule.h"
#include "vtkRenderingOpenGL2Module.h"

#endif
