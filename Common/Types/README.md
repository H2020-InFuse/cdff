# Types
We use ASN.1 types to exchange data via the DFNCI. The ASN.1 definitions
are in the subfolder ASN.1/. From these definitions we generate C types
automatically. These are located in the subfolder C/.

## Adding new ASN types
Run /Tools/ASN.1/GeneratorScript.sh to generate new C files.

## On first run
Cmake will run /Tools/ASN.1/FetcherScript.sh on first build to get the latest C files from the gitlab server.
