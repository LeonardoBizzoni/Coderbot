#!/usr/bin/env bash
cd "$(dirname "$0")"
set -eu

make -C libcoderbot clean all

for arg in "$@"; do
    if [[ $arg == *=* ]]; then
        var="${arg%%=*}"
        val="${arg#*=}"
        declare "$var"="$val"
    else
        declare $arg='1'
    fi
done

if [ -v reset ]; then
    file="src/reset.c libcoderbot/libcoderbot.a"
    outputf="reset.o"
else
    file="src/main.c libcoderbot/libcoderbot.a"
    outputf="main.o"
fi

links="-lpigpio -lpthread -lm -lrt -I./src/base -I."
common_flags="-pedantic -Wall -Werror -DPLATFORM_CODERBOT"
no_annoying_warnings="-Wno-unused-function -Wno-initializer-overrides"
no_annoying_cpp_warnings=""
asan="-fsanitize=address,undefined"

opt_flags="-O3 -s"
dbg_flags="-ggdb -O0 -DENABLE_ASSERT=1 -DDEBUG=1"

cpp_mode="-std=c++23 -fno-exceptions"

if [ ! -v release ]; then debug=1; fi
if [ ! -v clang ];   then gcc=1; fi

if [ -v gcc ];   then compiler=$([ -v cpp ] && echo "g++" || echo "gcc");         fi
if [ -v clang ]; then compiler=$([ -v cpp ] && echo "clang++" || echo "clang");   fi

flags="$links $common_flags $no_annoying_warnings "
if [ -v cpp ]; then flags+="$no_annoying_cpp_warnings $cpp_mode "; fi

if [ -v debug ]; then
    flags+="$dbg_flags "
else
    flags+="$opt_flags "
fi

if   [ -v gcc ];   then printf "+ [ GNU "
elif [ -v clang ]; then printf "+ [ Clang "; fi

if [ -v cpp ];   then printf "C++ "; else printf "C "; fi; printf "compilation ]\n"
if [ -v debug ]; then echo "+ [ debug mode ]"; else echo "+ [ release mode ]"; fi
if [ -v encoder ]; then flags+="-DENABLE_ENCODER_PRINT "; fi
if [ -v odometry ]; then flags+="-DENABLE_ODOMETRY_PRINT "; fi
if [ -v planner ]; then flags+="-DENABLE_PLANNER_PRINT "; fi
(set -x; $compiler $file $flags $asan -o $outputf)
