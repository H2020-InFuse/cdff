# Types

We use ASN.1 types to exchange data via the DFNCI. The ASN.1 definitions
are in the subfolder ASN.1/. From these definitions we generate C types
automatically. These are located in the subfolder C/.

## Generating C Code for New Types

We use [asn1scc](https://github.com/ttsiodras/asn1scc) to generate C code from
.asn files. Build instructions for the compiler can be found at the projects
readme. The executable Asn1f2.exe is required to use the compiler. It is
located at asn1scc/Asn1f2/bin/Debug/Asn1f2.exe. You can generate C types
for all .asn files in the folder ASN.1/ with

    mono Asn1f2.exe ASN.1/*.asn -c -o C/

