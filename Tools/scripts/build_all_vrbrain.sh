#!/bin/bash
# build all targets for PX4
# This helps when doing large merges
# Andrew Tridgell, February 2013

. config.mk

set -e
set -x

for d in ArduPlane ArduCopter APMrover2 ArduSub; do
    pushd $d
    make vrbrain-clean
    popd
done

echo "Testing ArduPlane build"
pushd ArduPlane
make -j8 vrbrain
popd

echo "Testing ArduCopter build"
pushd ArduCopter
make -j8 vrbrain
popd

echo "Testing APMrover2 build"
pushd APMrover2
make -j8 vrbrain
popd

echo "Testing ArduSub build"
pushd ArduSub
make -j8 vrbrain
popd

exit 0
