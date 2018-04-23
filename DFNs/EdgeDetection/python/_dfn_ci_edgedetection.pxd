from libcpp cimport bool
from libcpp.string cimport string
cimport _cdff_types

cdef extern from "EdgeDetectionInterface.hpp" namespace "dfn_ci":
    cdef cppclass EdgeDetectionInterface:
        EdgeDetectionInterface()
        void process()
        void setConfigurationFile(string)
        void configure()
        void imageInput(_cdff_types.Frame data)
        _cdff_types.Frame edgeMapOutput()
        _cdff_types.Frame sobelGradientXOutput()
        _cdff_types.Frame sobelGradientYOutput()


cdef extern from "CannyDetector.hpp" namespace "dfn_ci":
    cdef cppclass CannyDetector(EdgeDetectionInterface):
        pass
cdef extern from "SobelDerivative.hpp" namespace "dfn_ci":
    cdef cppclass SobelDerivative(EdgeDetectionInterface):
        pass
