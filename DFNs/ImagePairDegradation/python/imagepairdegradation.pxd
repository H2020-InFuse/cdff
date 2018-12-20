from libcpp cimport bool
from cython.operator cimport dereference as deref
cimport _imagepairdegradation


cdef class ImagePairDegradation:
    cdef _imagepairdegradation.ImagePairDegradation* thisptr
    cdef bool delete_thisptr


cdef class ImagePairDegradationEdres:
    cdef _imagepairdegradation.ImagePairDegradationEdres* thisptr
    cdef bool delete_thisptr
