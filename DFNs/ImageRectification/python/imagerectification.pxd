from libcpp cimport bool
from cython.operator cimport dereference as deref
cimport _imagerectification


cdef class ImageRectification:
    cdef _imagerectification.ImageRectification* thisptr
    cdef bool delete_thisptr


cdef class ImageRectificationEdres:
    cdef _imagerectification.ImageRectificationEdres* thisptr
    cdef bool delete_thisptr
