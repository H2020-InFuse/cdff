from libcpp cimport bool
from cython.operator cimport dereference as deref
cimport _dfn_ci_sobelderivative
cdef class SobelScharr:
    cdef _dfn_ci_sobelderivative.SobelScharr* thisptr
    cdef bool delete_thisptr