from libcpp cimport bool
from libcpp.string cimport string
cimport _cdff_types


cdef extern from "ImageRectificationInterface.hpp" namespace "CDFF::DFN":
    cdef cppclass ImageRectificationInterface:
        ImageRectificationInterface()
        void process() except +
        void setConfigurationFile(string) except +
        void configure() except +

        void originalImageInput(_cdff_types.asn1SccFrame& data) except +

        _cdff_types.asn1SccFrame& rectifiedImageOutput() except +


cdef extern from "ImageRectification.hpp" namespace "CDFF::DFN::ImageRectification":
    cdef cppclass ImageRectification(ImageRectificationInterface):
        pass


cdef extern from "ImageRectificationEdres.hpp" namespace "CDFF::DFN::ImageRectification":
    cdef cppclass ImageRectificationEdres(ImageRectificationInterface):
        pass
