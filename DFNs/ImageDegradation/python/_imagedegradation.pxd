from libcpp cimport bool
from libcpp.string cimport string
cimport _cdff_types


cdef extern from "ImageDegradationInterface.hpp" namespace "CDFF::DFN":
    cdef cppclass ImageDegradationInterface:
        ImageDegradationInterface()
        void process() except +
        void setConfigurationFile(string) except +
        void configure() except +

        void originalImageInput(_cdff_types.asn1SccFrame& data) except +

        _cdff_types.asn1SccFrame& degradedImageOutput() except +


cdef extern from "ImageDegradation.hpp" namespace "CDFF::DFN::ImageDegradation":
    cdef cppclass ImageDegradation(ImageDegradationInterface):
        pass


cdef extern from "ImageDegradationEdres.hpp" namespace "CDFF::DFN::ImageDegradation":
    cdef cppclass ImageDegradationEdres(ImageDegradationInterface):
        pass
