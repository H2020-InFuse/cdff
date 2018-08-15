#!/usr/bin/env bash

# This script uses bashisms and sgrep (structured grep): sudo apt install sgrep:
#
# https://www.cs.helsinki.fi/u/jjaakkol/sgrep.html
# https://www.cs.helsinki.fi/u/jjaakkol/sgrepman.html
#
# Usage:
#
# 1. Move this script OUT OF the CDFF repository so it doesn't grep itself
# 2. Change directory to the root of the CDFF repository
# 3. Run the script ONCE AND ONCE ONLY
# 4. Explore results with git status, git diff, cat fix-namespacing.dfpc.todo

# 2018-08-14, romain michalec

# Enable extended pathname expansion if it isn't on already (it is usually on by default)
shopt -s extglob

# Output file for text filters
workfile="$(mktemp --tmpdir tmp.XXXXXXXXXX)"

# In the code for the DFPC interfaces: dfpc_ci --> CDFF::DFPC
for file in DFPCs/*/*Interface.{hpp,cpp}; do
    sed -e "s/^namespace dfpc_ci[[:space:]]*{[[:space:]]*$/namespace dfpc_ci\n{/" "${file}" |
        sgrep -a -N -O Tools/Sgrep/output-format.dfpc.interface -e 'outer("{" .. "}" in ("namespace dfpc_ci" .. end))' |
        sed -e "/^namespace dfpc_ci$/d" > "${workfile}"
    cp "${workfile}" "${file}"
done

# In the code for the DFPC interfaces: DFPCCommonInterface --> CDFF::DFPC::DFPCCommonInterface
for file in DFPCs/*/*Interface.hpp; do
    sed -i -e "s/public DFPCCommonInterface/public CDFF::DFPC::DFPCCommonInterface/" "${file}"
done

# In the code for the DFPC implementations: dfpc_ci --> CDFF::DFPC::DFPCName
format="$(mktemp --tmpdir dfpc.XXXXXXXXXX)"
for file in DFPCs/*/!(*Interface).{hpp,cpp}; do
    dfpc="$(basename $(dirname "${file}"))"
    sed -e "s/dfpc/${dfpc}/" Tools/Sgrep/output-format.dfpc.implementation > "${format}"
    sed -e "s/^namespace dfpc_ci[[:space:]]*{[[:space:]]*$/namespace dfpc_ci\n{/" "${file}" |
        sgrep -a -N -O "${format}" -e 'outer("{" .. "}" in ("namespace dfpc_ci" .. end))' |
        sed -e "/^namespace dfpc_ci$/d" > "${workfile}"
    cp "${workfile}" "${file}"
done

# In the client code (code that uses one or several DFPCs): using namespace dfpc_ci --> using namespace CDFF::DFPC[::DFPCName]
dfpcs="$(basename -a DFPCs/!(*.*))"
for file in $(grep -l -r -e "using namespace dfpc_ci" Common DFNs DFPCs Tests Tools); do
    parentdir="$(basename $(dirname "${file}"))"
    if echo "${dfpcs}" | grep -q -w -e "${parentdir}"; then
        dfpc=${parentdir}
        sed -i -e "s/using namespace dfpc_ci/using namespace CDFF::DFPC::${dfpc}/" "${file}"
    else
        sed -i -e "s/using namespace dfpc_ci/using namespace CDFF::DFPC::WHICH-DFPC(S)-IF-ANY?/" "${file}"
    fi
done

# In the client code (code that uses one or several DFPCs): dfpc_ci:: --> CDFF::DFPC::
for file in $(grep -l -r -e "dfpc_ci::" Common DFNs DFPCs Tests Tools); do
    sed -i -e "s/dfpc_ci::/CDFF::DFPC::/g" "${file}"
done

# Code that cannot be fixed automatically
echo "Files to fix manually have been written to fix-namespacing.dfpc.todo"
grep -r -e "dfpc_ci" Common DFNs DFPCs Tests Tools > fix-namespacing.dfpc.todo
grep -r -e "WHICH-DFPC(S)-IF-ANY?" Common DFNs DFPCs Tests Tools >> fix-namespacing.dfpc.todo
