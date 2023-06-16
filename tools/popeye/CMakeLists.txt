set(LLVM_LINK_COMPONENTS
        LLVMAggressiveInstCombine
        LLVMAnalysis
        LLVMAsmParser
        LLVMAsmPrinter
        LLVMBinaryFormat
        LLVMBitReader
        LLVMBitWriter
        LLVMBitstreamReader
        LLVMCodeGen
        LLVMCore
        LLVMCoroutines
        LLVMDemangle
        LLVMFrontendOpenMP
        LLVMIRReader
        LLVMInstCombine
        LLVMInstrumentation
        LLVMLTO
        LLVMLinker
        LLVMMC
        LLVMMCParser
        LLVMMIRParser
        LLVMObject
        LLVMObjectYAML
        LLVMOption
        LLVMPasses
        LLVMProfileData
        LLVMRemarks
        LLVMScalarOpts
        LLVMSupport
        LLVMTarget
        LLVMTransformUtils
        LLVMVectorize
        LLVMipo
        )

find_library(GRAPHVIZ gvc)
if(GRAPHVIZ-NOTFOUND)
    set(GRAPHVIZ_DEP)
    message(STATUS "GRAPHVIZ not found")
else()
    set(GRAPHVIZ_DEP gvc cgraph cdt)
    message(STATUS "GRAPHVIZ found")
endif()

add_executable(popeye popeye.cpp LiftingPass.cpp)
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_link_libraries(popeye PRIVATE
            PPYCore PPYMemory PPYSupport PPYTransform PPYBNF
            -Wl,--start-group
            ${LLVM_LINK_COMPONENTS}
            -Wl,--end-group
            z3 z ncurses pthread dl
            ${GRAPHVIZ_DEP}
            )
else()
    target_link_libraries(popeye PRIVATE
            PPYCore PPYMemory PPYSupport PPYTransform PPYBNF
            ${LLVM_LINK_COMPONENTS}
            z3 z ncurses pthread dl
            ${GRAPHVIZ_DEP}
            )
endif()