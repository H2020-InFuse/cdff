from libcpp cimport bool
from cython.operator cimport dereference as deref
cimport _stereomotionestimation


cdef class StereoMotionEstimationEdres:
    cdef _stereomotionestimation.StereoMotionEstimationEdres* thisptr
    cdef bool delete_thisptr
