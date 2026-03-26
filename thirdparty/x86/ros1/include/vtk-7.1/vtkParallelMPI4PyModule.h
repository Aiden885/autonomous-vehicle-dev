
#ifndef VTKPARALLELMPI4PY_EXPORT_H
#define VTKPARALLELMPI4PY_EXPORT_H

#ifdef VTKPARALLELMPI4PY_STATIC_DEFINE
#  define VTKPARALLELMPI4PY_EXPORT
#  define VTKPARALLELMPI4PY_NO_EXPORT
#else
#  ifndef VTKPARALLELMPI4PY_EXPORT
#    ifdef vtkParallelMPI4Py_EXPORTS
        /* We are building this library */
#      define VTKPARALLELMPI4PY_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VTKPARALLELMPI4PY_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VTKPARALLELMPI4PY_NO_EXPORT
#    define VTKPARALLELMPI4PY_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VTKPARALLELMPI4PY_DEPRECATED
#  define VTKPARALLELMPI4PY_DEPRECATED __attribute__ ((__deprecated__))
#  define VTKPARALLELMPI4PY_DEPRECATED_EXPORT VTKPARALLELMPI4PY_EXPORT __attribute__ ((__deprecated__))
#  define VTKPARALLELMPI4PY_DEPRECATED_NO_EXPORT VTKPARALLELMPI4PY_NO_EXPORT __attribute__ ((__deprecated__))
#endif

#define DEFINE_NO_DEPRECATED 0
#if DEFINE_NO_DEPRECATED
# define VTKPARALLELMPI4PY_NO_DEPRECATED
#endif

#define VTK_USE_MPI

#endif
