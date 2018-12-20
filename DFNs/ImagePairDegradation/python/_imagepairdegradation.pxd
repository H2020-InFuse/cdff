from libcpp cimport bool
from libcpp.string cimport string
cimport _cdff_types


cdef extern from "ImagePairDegradationInterface.hpp" namespace "CDFF::DFN":
    cdef cppclass ImagePairDegradationInterface:
        ImagePairDegradationInterface()
        void process() except +
        void setConfigurationFile(string) except +
        void configure() except +

        void originalImagePairInput(_cdff_types.asn1SccFramePair& data) except +

        _cdff_types.asn1SccFramePair& degradedImagePairOutput() except +


cdef extern from "ImagePairDegradation.hpp" namespace "CDFF::DFN::ImagePairDegradation":
    cdef cppclass ImagePairDegradation(ImagePairDegradationInterface):
        pass


cdef extern from "ImagePairDegradationEdres.hpp" namespace "CDFF::DFN::ImagePairDegradation":
    cdef cppclass ImagePairDegradationEdres(ImagePairDegradationInterface):
        pass
