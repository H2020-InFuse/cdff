add_library(
    xml_helper
    XmlHelper.cpp
)
target_link_libraries(xml_helper tinyxml2)
install(
    TARGETS xml_helper
    DESTINATION lib
)
