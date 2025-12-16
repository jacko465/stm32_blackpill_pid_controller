Import("env")

# Force hard-float + FPU flags for compilation AND linking
fpu_flags = ["-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]

env.Append(CCFLAGS=fpu_flags)
env.Append(CXXFLAGS=fpu_flags)
env.Append(ASFLAGS=fpu_flags)
env.Append(LINKFLAGS=fpu_flags)
