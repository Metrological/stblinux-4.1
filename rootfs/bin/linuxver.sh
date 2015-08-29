#!/bin/sh

error_usage() {
	echo "Usage: $0 <linux_kernel_directory>" 1>&2
	exit 1
}

[ $# -eq 0 -o "$1" = "-h" ] && error_usage

LINUX_MAKEFILE="$1/Makefile"

# Extract the first 10 lines of the Makefile, just in case there are additional
# variables in there
VAR="$(cat $LINUX_MAKEFILE | head -10 | sed 's/ //g')"
while read line
do
	[ -z $version ] && version=$(echo $line | grep VERSION | cut -d= -f2)
	[ -z $patch_lvl ] && patch_lvl=$(echo $line | grep PATCHLEVEL | cut -d= -f2)
	# remove the pre here, we do not want that
	[ -z $extra_ver ] && extra_ver=$(echo $line | grep EXTRAVERSION | cut -d= -f2 | sed s/pre//)
done <<< "$VAR"

fullversion="$version$patch_lvl$extra_ver"

if [ -z "$fullversion" ]; then
	echo "Error: version not found in $LINUX_MAKEFILE" 1>&2
	exit 1
fi

echo "$fullversion"
