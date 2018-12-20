from libcpp cimport bool
from cython.operator cimport dereference as deref
cimport _imagedegradation


cdef class ImageDegradation:
    cdef _imagedegradation.ImageDegradation* thisptr
    cdef bool delete_thisptr


cdef class ImageDegradationEdres:
    cdef _imagedegradation.ImageDegradationEdres* thisptr
    cdef bool delete_thisptr
