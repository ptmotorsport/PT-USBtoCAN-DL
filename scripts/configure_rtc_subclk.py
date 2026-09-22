from pathlib import Path

Import("env")

framework_dir = Path(
    env.PioPlatform().get_package_dir("framework-arduinorenesas-uno")
)
rtc_cpp = framework_dir / "libraries" / "RTC" / "src" / "RTC.cpp"

default_setting = "#define RTC_CLOCK_SOURCE RTC_CLOCK_SOURCE_LOCO"
subclk_setting = "#define RTC_CLOCK_SOURCE RTC_CLOCK_SOURCE_SUBCLK"

contents = rtc_cpp.read_text(encoding="utf-8")

if default_setting in contents:
    rtc_cpp.write_text(contents.replace(default_setting, subclk_setting, 1), encoding="utf-8")
    print("Configured Arduino RTC library to use the external SUBCLK source.")
elif subclk_setting not in contents:
    raise RuntimeError(
        f"Could not find the expected RTC clock-source setting in {rtc_cpp}."
    )