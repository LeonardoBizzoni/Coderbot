#!/usr/bin/env bash
cd "$(dirname "$0")"
set -eu

links="-lpthread -lm -lrt -I./src/base -I."
common_flags="-pedantic -Wall -Werror"
no_annoying_warnings="-Wno-unused-function -Wno-strict-aliasing"
asan="-fsanitize=address,undefined"
dbg_flags="-ggdb -O0 -DENABLE_ASSERT=1 -DDEBUG=1 $asan"

flags="$links $common_flags $no_annoying_warnings "

expected_trajectory=$(gcc $flags $dbg_flags src/gen_arcs.c -o gen_args.o && ./gen_args.o)
# TODO(lb): does this work?
# followed_trajectory=$(ssh coderbot@icecube3 << EOF
#                                             cd LB/Progetto;
#                                             ./build.sh odometry release >> /dev/null;
#                                             sudo ./main.o;
#                           EOF)
# Rscript grafico_coderbot.R "$expected_trajectory" "$followed_trajectory"
Rscript grafico_coderbot.R "$expected_trajectory" "(2000, 2000)"
