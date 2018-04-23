from libcpp cimport bool
from cython.operator cimport dereference as deref
cimport _dfn_ci_edgedetection
cdef class CannyDetector:
    cdef _dfn_ci_edgedetection.CannyDetector* thisptr
    cdef bool delete_thisptr
cdef class SobelDerivative:
    cdef _dfn_ci_edgedetection.SobelDerivative* thisptr
    cdef bool delete_thisptr