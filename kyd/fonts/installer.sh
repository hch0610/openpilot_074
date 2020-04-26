#!/usr/bin/bash

###############################################################################
# The MIT License
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
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
# Noto is a trademark of Google Inc. Noto fonts are open source.
# All Noto fonts are published under the SIL Open Font License,
# Version 1.1. Language data and some sample texts are from the Unicode CLDR project.
#
###############################################################################


# Android system locale
lang=ko-KR

update_font=0

# check regular font
if [ ! -f "/system/fonts/NotoSansCJKtc-Regular.otf" ]; then
    update_font=1
fi

if [ $update_font -eq "1" ]; then
    # sleep 3 secs in case, make sure the /system is re-mountable
    sleep 3
    mount -o remount,rw /system
    if [ $update_font -eq "1" ]; then
        # install font
        cp -rf /data/openpilot/kyd/fonts/NotoSansCJKtc-* /system/fonts/
        # install font mapping
        cp -rf /data/openpilot/kyd/fonts/fonts.xml /system/etc/fonts.xml
        # change permissions
        chmod 644 /system/etc/fonts.xml
        chmod 644 /system/fonts/NotoSansCJKtc-*
    fi
    mount -o remount,r /system
fi

# change system locale
setprop persist.sys.locale $lang
setprop persist.sys.local $lang
