# There are 3 different types of shells in bash: the login shell, normal shell
# and interactive shell. Login shells read ~/.profile and interactive shells
# read ~/.bashrc; in our setup, .profile sources .bashrc - thus all settings
# made here will also take effect in a login shell.
#

if [ -f /etc/bash_completion ] && ! shopt -oq posix; then
	. /etc/bash_completion
fi

# configure bash history
HISTSIZE=100             # save only up to 100 commands
HISTCONTROL=ignoreboth   # ignore duplicate commands
shopt -s histappend      # append to history file instead of overwriting it
export PROMPT_COMMAND="history -a; $PROMPT_COMMAND"

# Some applications read the EDITOR variable to determine your favourite text
# editor. So uncomment the line below and enter the editor of your choice :-)
export EDITOR=/usr/bin/joe

test -s ~/.alias && . ~/.alias

#
# Set shell prompt
NOCOLOR="\[\033[0;0m\]"
MAGENTA="\[\033[0;35m\]"
GREEN="\[\033[0;32m\]"
RED="\[\033[0;33m\]"

if test "$UID" = 0 ; then
	LOGINCOLOR=$RED
else
	LOGINCOLOR=$GREEN
fi

PS1="$RED# $NOCOLOR\$(date +%H:%M:%S) $LOGINCOLOR\h:\w>$NOCOLOR "
export PS1

# Set path
export PATH=$PATH:/root:/sbin:/usr/sbin:/usr/local/sbin

# Look for libraries in local folder as well
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}.
export ROS_HOSTNAME=192.168.43.102
export ROS_MASTER_URI=http://192.168.43.102:11311
source /opt/ros/indigo/setup.bash
source /root/catkin_ws/devel/setup.bash
