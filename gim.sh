# !/bin/sh

# Copyright (c) 2014-2017 Advanced Micro Devices, Inc. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE

ORIGIN_USER=$(whoami)
echo "Current User is $ORIGIN_USER"

SRC_ROOT=$(cd `dirname $0`; pwd)

if [ $UID -ne 0 ]; then
	echo "Superuser privileges are required to run this script."
	cd $SRC_ROOT
	su -c './gim.sh'
else
	Path=$(readlink /lib/modules/$(uname -r)/build)
	echo "check: $Path"

	#A work around for relative Path link for CentOS kernel 4.9
	if [ $Path == "../../../usr/src/kernels/4.9.13-22.el7.x86_64" ]; then
		cd /lib/modules/$(uname -r)/
		rm  ./build
		ln -s /usr/src/kernels/4.9.13-22.el7.x86_64 build
		ls -l
	fi

	#su $(ORIGIN_USER)

	cd $SRC_ROOT/drv

	make
	make install

fi

exit
