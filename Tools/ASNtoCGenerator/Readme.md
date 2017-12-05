# Types

We use ASN.1 types to exchange data via the DFNCI. The ASN.1 definitions
are in the subfolder /Common/Types/ASN.1/. From these definitions we generate C types
automatically. These are located in /Common/Types/C/.

## Generating C Code for New Types
We use [asn1scc](https://github.com/ttsiodras/asn1scc) to generate C code from asn files.
It can be downloaded from https://download.tuxfamily.org/taste/ASN1SCC/

Usage (see GeneratorScript.sh): 
mono asn1.exe ../Common/Types/ASN.1/*.asn -c -o ../Common/Types/C
