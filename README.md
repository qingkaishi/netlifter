### Popeye

Popeye is a prototype implementation of the technique known as NetLifter, which is published in CCS 2023
and aims to lift the source code of a protocol implementation to BNF-style protocol formats.
- [Paper](https://qingkaishi.github.io/public_pdfs/CCS23.pdf)
- [Artifact](https://github.com/qingkaishi/netlifter)

```
Lifting Network Protocol Implementation to Precise Format Specification with Security Applications
Qingkai Shi, Junyang Shao, Yapeng Ye, Mingwei Zheng, Xiangyu Zhang
The ACM Conference on Computer and Communications Security (CCS '23)
```

News! A follow-up work on identifying silent protocol bugs has been accepted by OOPSLA 2024! 
- [Paper](https://qingkaishi.github.io/public_pdfs/OOPSLA2024.pdf)
- [Artifact](https://github.com/zmw12306/ParDiff)

```
ParDiff: Practical Static Differential Analysis of Network Protocol Parsers
Mingwei Zheng, Qingkai Shi, Xuwei Liu, Xiangzhe Xu, Le Yu, Congyu Liu, Guannan Wei, Xiangyu Zhang
Proceedings of the ACM on Programming Languages (OOPSLA '24)
```

### Disclaimer 
There are a lot of rough edges and bugs in this early prototype. Pull requests are welcome if you like to contribute to this project.


### Build

Dependency:
* llvm-12.0.1, you can install it by `sudo apt install llvm-12`.
* z3-4.8.12, better to use [my copy](https://github.com/qingkaishi/z3/tree/4.8.12-popeye), need to compile and install using [cmake](https://github.com/qingkaishi/z3/blob/4.8.12-popeye/README-CMake.md).

```bash
$ mkdir build
$ cd build
$ cmake ..
$ make
```

Run the following command for regression testing. You can refer to `benchmarks/popeye/regession.sh` for how we run each benchmark protocol.

```bash
$ cd build
$ make regression
```

### Run


Details of how we run Popeye can be found [here](https://docs.google.com/document/d/1u80FbynWhiit1cgC0s5sGQcInXIbXNDOs3kEJbGR9VA/edit?usp=sharing).
Please refer to benchmarks/readme.txt for more examples.
Basically, the input is LLVM bitcode with some simple annotations that annotate the byte buffer containing the network message.
The output is the message format in BNF.

Note that when compiling source code to LLVM bitcode,
please add `-fno-vectorize -fno-slp-vectorize` to `CFLAGS` and `CXXFLAGS`
to avoid generating vectorized instructions, which currently we do not support.

Also, it's better to remove all `-Ox` options and add the `-g` option
to `CFLAGS` and `CXXFLAGS`, which will let the bitcode include debug information.
The debug information will help infer the name of fields in a network message.
