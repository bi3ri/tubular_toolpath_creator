#!/bin/bash
SCRIPT=`realpath $0`
SCRIPTPATH=`dirname $SCRIPT`

eval "$(command conda 'shell.bash' 'hook' 2> /dev/null)"
conda activate vmtk
python3 $SCRIPTPATH/toolpath_centerline.py $1 $2