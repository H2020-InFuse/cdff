The Envire ASN1 library is used to register the types that can be stored in the Central Data Products Manager. This registration has to steps: 
0. Registration for serialization. Implemented in the `Register<type>.cpp`sources.
0. Registration as plugin of the correspondent items for the Envire plugin mechanishm, that uses the plugin_manager library. Implemented in `plugin_manager/cdff_support_envireasn1.xml` file
