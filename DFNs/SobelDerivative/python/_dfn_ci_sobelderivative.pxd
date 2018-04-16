from libcpp cimport bool
from libcpp.string cimport string
cimport _cdff_types

cdef extern from "SobelDerivativeInterface.hpp" namespace "dfn_ci":
    cdef cppclass SobelDerivativeInterface:
        SobelDerivativeInterface()
        void process()
        void setConfigurationFile(string)
        void configure()
        void imageInput(_cdff_types.Frame data)
        _cdff_types.Frame sobelGradxOutput()
        _cdff_types.Frame sobelGradyOutput()


cdef extern from "SobelScharr.hpp" namespace "dfn_ci":
    cdef cppclass SobelScharr(SobelDerivativeInterface):
        pass
