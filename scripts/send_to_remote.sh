#!/bin/bash
set -e
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"


cd ${DIR}/../build/
rm -rf Control-Software
mkdir Control-Software
mkdir Control-Software/build
#cp common/test-common Control-Software/build
#cp $1 Control-Software/build
find . -name \*.so* -exec cp {} ./Control-Software/build \;
#cp ../scripts/run_mc* ./Control-Software/build
#cp ../scripts/setup_network_mc.py ./Control-Software/build
#cp ../scripts/run_mc_debug.sh ./Control-Software/build
#cp ../scripts/config_network_lcm.sh ./Control-Software
#cp -r ../robot Control-Software
cp -r ../config Control-Software

scp -r Control-Software root@192.168.3.4:/home/leeqee

#after that, ssh into romote host, add "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/leeqee/Control-Software/build" into ~/.bashrc 



