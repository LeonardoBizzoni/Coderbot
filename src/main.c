#include <stdio.h>

#include <base/base_inc.h>
#include <OS/os_inc.h>

#include <base/base_inc.c>
#include <OS/os_inc.c>

fn void start(CmdLine *cli) {
#if 1
  printf("Compiler GCC:   %d\n", COMPILER_GCC);
  printf("Compiler CL:    %d\n", COMPILER_CL);
  printf("Compiler CLANG: %d\n", COMPILER_CLANG);

  printf("OS GNU/Linux:   %d\n", OS_LINUX);
  printf("OS BSD:         %d\n", OS_BSD);
  printf("OS MAC:         %d\n", OS_MAC);
  printf("OS Windows:     %d\n", OS_WINDOWS);

  printf("Architecture x86 32bit: %d\n", ARCH_X86);
  printf("Architecture x64 64bit: %d\n", ARCH_X64);
  printf("Architecture ARM 32bit: %d\n", ARCH_ARM32);
  printf("Architecture ARM 64bit: %d\n", ARCH_ARM64);
#endif

}
