
#ifndef VTKRENDERINGSCENEGRAPH_EXPORT_H
#define VTKRENDERINGSCENEGRAPH_EXPORT_H

#ifdef VTKRENDERINGSCENEGRAPH_STATIC_DEFINE
#  define VTKRENDERINGSCENEGRAPH_EXPORT
#  define VTKRENDERINGSCENEGRAPH_NO_EXPORT
#else
#  ifndef VTKRENDERINGSCENEGRAPH_EXPORT
#    ifdef vtkRenderingSceneGraph_EXPORTS
        /* We are building this library */
#      define VTKRENDERINGSCENEGRAPH_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VTKRENDERINGSCENEGRAPH_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VTKRENDERINGSCENEGRAPH_NO_EXPORT
#    define VTKRENDERINGSCENEGRAPH_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VTKRENDERINGSCENEGRAPH_DEPRECATED
#  define VTKRENDERINGSCENEGRAPH_DEPRECATED __attribute__ ((__deprecated__))
#  define VTKRENDERINGSCENEGRAPH_DEPRECATED_EXPORT VTKRENDERINGSCENEGRAPH_EXPORT __attribute__ ((__deprecated__))
#  define VTKRENDERINGSCENEGRAPH_DEPRECATED_NO_EXPORT VTKRENDERINGSCENEGRAPH_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKRENDERINGSCENEGRAPH_NO_DEPRECATED
#endif



#endif
