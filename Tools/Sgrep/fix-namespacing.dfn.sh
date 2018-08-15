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
# 4. Explore results with git status, git diff, cat fix-namespacing.dfn.todo

# 2018-08-14, romain michalec

# Enable extended pathname expansion if it isn't on already (it is usually on by default)
shopt -s extglob

# Output file for text filters
workfile="$(mktemp --tmpdir tmp.XXXXXXXXXX)"

# In the code for the DFN interfaces: dfn_ci --> CDFF::DFN
for file in DFNs/*/*Interface.{hpp,cpp}; do
    sed -e "s/^namespace dfn_ci[[:space:]]*{[[:space:]]*$/namespace dfn_ci\n{/" "${file}" |
        sgrep -a -N -O Tools/Sgrep/output-format.dfn.interface -e 'outer("{" .. "}" in ("namespace dfn_ci" .. end))' |
        sed -e "/^namespace dfn_ci$/d" > "${workfile}"
    cp "${workfile}" "${file}"
done

# In the code for the DFN interfaces: DFNCommonInterface --> CDFF::DFN::DFNCommonInterface
for file in DFNs/*/*Interface.hpp; do
    sed -i -e "s/public DFNCommonInterface/public CDFF::DFN::DFNCommonInterface/" "${file}"
done

# In the code for the DFN implementations: dfn_ci --> CDFF::DFN::DFNName
format="$(mktemp --tmpdir dfn.XXXXXXXXXX)"
for file in DFNs/*/!(*Interface).{hpp,cpp}; do
    dfn="$(basename $(dirname "${file}"))"
    sed -e "s/dfn/${dfn}/" Tools/Sgrep/output-format.dfn.implementation > "${format}"
    sed -e "s/^namespace dfn_ci[[:space:]]*{[[:space:]]*$/namespace dfn_ci\n{/" "${file}" |
        sgrep -a -N -O "${format}" -e 'outer("{" .. "}" in ("namespace dfn_ci" .. end))' |
        sed -e "/^namespace dfn_ci$/d" > "${workfile}"
    cp "${workfile}" "${file}"
done

# In the client code (code that uses one or several DFNs): using namespace dfn_ci --> using namespace CDFF::DFN[::DFNName]
dfns="$(basename -a DFNs/!(*.*))"
for file in $(grep -l -r -e "using namespace dfn_ci" Common DFNs DFPCs Tests Tools); do
    parentdir="$(basename $(dirname "${file}"))"
    if echo "${dfns}" | grep -q -w -e "${parentdir}"; then
        dfn=${parentdir}
        sed -i -e "s/using namespace dfn_ci/using namespace CDFF::DFN::${dfn}/" "${file}"
    else
        sed -i -e "s/using namespace dfn_ci/using namespace CDFF::DFN::WHICH-DFN(S)-IF-ANY?/" "${file}"
    fi
done

# In the client code (code that uses one or several DFNs): dfn_ci:: --> CDFF::DFN::
for file in $(grep -l -r -e "dfn_ci::" Common DFNs DFPCs Tests Tools); do
    sed -i -e "s/dfn_ci::/CDFF::DFN::/g" "${file}"
done

# Code that cannot be fixed automatically
echo "Files to fix manually have been written to fix-namespacing.dfn.todo"
grep -r -e "dfn_ci" Common DFNs DFPCs Tests Tools > fix-namespacing.dfn.todo
grep -r -e "WHICH-DFN(S)-IF-ANY?" Common DFNs DFPCs Tests Tools >> fix-namespacing.dfn.todo
