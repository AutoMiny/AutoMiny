#!/usr/bin/env sh
# generated from within catkin_tools/verbs/catkin_build/common.py

if [ $# -eq 0 ] ; then
  /bin/echo "Usage: build_env.sh COMMANDS"
  /bin/echo "Calling build_env.sh without arguments is not supported anymore."
  /bin/echo "Instead spawn a subshell and source a setup file manually."
  exit 1
fi

# save original args for later
_ARGS=
_ARGI=0
for arg in "$@"; do
  # Define placeholder variable
  eval "_A$_ARGI=\$arg"
  # Add placeholder variable to arg list
  _ARGS="$_ARGS \"\$_A$_ARGI\""
  # Increment arg index
  _ARGI=`expr $_ARGI + 1`

  #######################
  ## Uncomment for debug:
  #_escaped="$(echo "$arg" | sed -e 's@ @ @g')"
  #echo "$_escaped"
  #eval "echo '$_ARGI \$_A$_ARGI'"
  #######################
done

#######################
## Uncomment for debug:
#echo "exec args:"
#echo "$_ARGS"
#for arg in $_ARGS; do eval echo $arg; done
#echo "-----------"
#####################

# remove all passed in args, resetting $@, $*, $#, $n
shift $#
# set the args for the sourced scripts
set -- $@ "--extend"
# source setup.sh with implicit --extend argument for each direct build depend in the workspace
. "/root/catkin_ws/odroid-devel/setup.sh"

# execute given args
eval exec $_ARGS
