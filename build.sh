#!/usr/bin/env bash
cd "$(dirname "$0")"
set -eu

make -C libcoderbot

file="src/main.c libcoderbot/libcoderbot.a"

for arg in "$@"; do
    if [[ $arg == *=* ]]; then
        var="${arg%%=*}"
        val="${arg#*=}"
        declare "$var"="$val"
    else
        declare $arg='1'
    fi
done

links="-lpthread -lm -lrt -lpigpio -I./src/base -I."
common_flags="-pedantic -Wall -Werror -DPLATFORM_CODERBOT"
no_annoying_warnings="-Wno-unused-function -Wno-initializer-overrides"
no_annoying_cpp_warnings=""
asan="-fsanitize=address,undefined"

opt_flags="-O3 -s"
dbg_flags="-O0 -g3 -ggdb ${asan} -DENABLE_ASSERT=1 -DDEBUG=1"

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
(set -x; $compiler $file $flags -o main)
