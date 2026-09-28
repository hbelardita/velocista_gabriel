Import("env")
import os

toolchain_path = os.path.expanduser("~/.platformio/packages/toolchain-gccmingw32/bin")
if os.path.exists(toolchain_path):
    env.PrependENVPath("PATH", toolchain_path)
    os.environ["PATH"] = toolchain_path + os.pathsep + os.environ.get("PATH", "")

env.Append(LINKFLAGS=["-static-libgcc", "-static-libstdc++"])
