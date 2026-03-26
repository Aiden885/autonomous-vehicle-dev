
#ifndef VTKFILTERSREEBGRAPH_EXPORT_H
#define VTKFILTERSREEBGRAPH_EXPORT_H

#ifdef VTKFILTERSREEBGRAPH_STATIC_DEFINE
#  define VTKFILTERSREEBGRAPH_EXPORT
#  define VTKFILTERSREEBGRAPH_NO_EXPORT
#else
#  ifndef VTKFILTERSREEBGRAPH_EXPORT
#    ifdef vtkFiltersReebGraph_EXPORTS
        /* We are building this library */
#      define VTKFILTERSREEBGRAPH_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VTKFILTERSREEBGRAPH_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VTKFILTERSREEBGRAPH_NO_EXPORT
#    define VTKFILTERSREEBGRAPH_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VTKFILTERSREEBGRAPH_DEPRECATED
#  define VTKFILTERSREEBGRAPH_DEPRECATED __attribute__ ((__deprecated__))
#  define VTKFILTERSREEBGRAPH_DEPRECATED_EXPORT VTKFILTERSREEBGRAPH_EXPORT __attribute__ ((__deprecated__))
#  define VTKFILTERSREEBGRAPH_DEPRECATED_NO_EXPORT VTKFILTERSREEBGRAPH_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKFILTERSREEBGRAPH_NO_DEPRECATED
#endif



#endif
