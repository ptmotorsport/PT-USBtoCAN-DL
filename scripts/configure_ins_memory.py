from pathlib import Path

Import("env")

source = Path(env.subst("$LDSCRIPT_PATH"))
text = source.read_text(encoding="utf-8")
include = "INCLUDE memory_regions.ld"
if text.count(include) != 1:
    raise RuntimeError("INS stack configuration: unsupported Renesas linker script")

destination = Path(env.subst("$BUILD_DIR")) / "ins_memory.ld"
destination.parent.mkdir(parents=True, exist_ok=True)
destination.write_text(
    text.replace(include, include + "\nBSP_CFG_STACK_MAIN_BYTES = 0x1000;"),
    encoding="utf-8",
)
flags = list(env["LINKFLAGS"])
script_options = [i for i, flag in enumerate(flags) if flag == "-T"]
if len(script_options) != 1 or script_options[0] + 1 >= len(flags):
    raise RuntimeError("INS stack configuration: unsupported linker options")
flags[script_options[0] + 1] = str(destination)
env.Replace(LDSCRIPT_PATH=str(destination), LINKFLAGS=flags)
