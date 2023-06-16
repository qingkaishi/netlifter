### Popeye

Popeye is a prototype implementation of the technique known as NetLifter, which is published in CCS 2023
and aims to lift the source code of a protocol implementation to BNF-style protocol formats.

```
Lifting Network Protocol Implementation to Precise Format Specification with Security Applications
Qingkai Shi, Junyang Shao, Yapeng Ye, Mingwei Zheng, Xiangyu Zhang
The ACM Conference on Computer and Communications Security (CCS '23)
```

### Disclaimer 
There are a lot of rough edges and bugs in this early prototype. Pull requests are welcome if you like to contribute to this project.


### Build

Dependency:
* llvm-12.0.1, you can install it by `sudo apt install llvm-12`
* z3-4.8.12, better to use [my copy](https://github.com/qingkaishi/z3/tree/4.8.12-popeye), need to compile and install using [cmake](https://github.com/qingkaishi/z3/blob/4.8.12-popeye/README-CMake.md).

```bash
$ mkdir build
$ cd build
$ cmake ..
$ make
```

Run the following command for regression testing. You can refer to `benchmarks/popeye/regession.sh` for how we run each benchmark protocol

```bash
$ cd build
$ make regression
```

### Run

Details of Popeye's I/O format can be found [here](https://docs.google.com/document/d/1u80FbynWhiit1cgC0s5sGQcInXIbXNDOs3kEJbGR9VA/edit?usp=sharing).
Please refer to benchmarks/readme.txt for more examples.

Note that when compiling source code to LLVM bitcode,
please add `-fno-vectorize -fno-slp-vectorize` to `CFLAGS` and `CXXFLAGS`
to avoid generating vectorized instructions, which currently we do not support.

Also, it's better to remove all `-Ox` options and add the `-g` option
to `CFLAGS` and `CXXFLAGS`, which will let the bitcode include debug information.
The debug information will help infer the name of fields in a network message.

What follows is the basic steps of running our tool. You may use `wllvm` to compile 
all source files in a C/C++ project into a single bitcode file.

```bash
$ # since we do not support vectorized instructions, please use
$ # -fno-vectorize -fno-slp-vectorize when producing the bitcode
$ clang -emit-llvm -g -fno-vectorize -fno-slp-vectorize test.c -o test.bc
$ ./popeye -popeye-output:bnf test.bc 
=========================
L0 := B[2] B[3];
 assert(B[2]B[3] ≠ 1)
 assert(B[2]B[3] ≠ 2)
 assert(B[2]B[3] ≠ 5)
L1 := B[2] B[3];
 assert(B[2]B[3] = 1)
L2 := B[2] B[3];
 assert(B[2]B[3] = 2)
L3 := B[2] B[3];
 assert(B[2]B[3] = 5)
S := B[0] B[1] L0 | L1 | L2 | L3;

=========================
Popeye completes in 6ms
```

