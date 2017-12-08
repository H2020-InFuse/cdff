# Types
We use ASN.1 types to exchange data via the DFNCI. The ASN.1 definitions
are in the subfolder /Common/Types/ASN.1/. From these definitions we generate C types
automatically. These are located in /Common/Types/C/.

## Generating C Code from ASN files
Run GeneratorScript.sh, it will compile all .asn files withing the /Common/Types/ASN.1 folder into /Common/Types/C.
* note [pre-requisite] mono needs to be installed. 

The script uses the ASN.1 [asn1scc](https://github.com/ttsiodras/asn1scc) compiler to generate C code from asn files.
A Binary can be downloaded from https://download.tuxfamily.org/taste/ASN1SCC/

## Fetching latest C code
Run FetcherScript.sh to get latest C generated build files from master. 
*note : This will override your /Common/Types/C folder. 

