import json
import os

# Find the esp32-camera library.json
lib_dir = os.path.join(".", ".pio", "libdeps", "esp32cam", "esp32-camera")
lib_json_path = os.path.join(lib_dir, "library.json")

REQUIRED_FLAGS = [
    "-Idriver/include",
    "-Iconversions/include",
    "-Idriver/private_include",
    "-Iconversions/private_include",
    "-Isensors/private_include",
    "-Itarget/private_include",
    "-Itarget/jpeg_include",
]

REQUIRED_SRC_FILTER = [
    "-<*>",
    "+<driver>",
    "-<driver/sccb.c>",
    "+<conversions>",
    "+<sensors>",
    "+<target>",
    "-<target/esp32s2>",
    "-<target/esp32s3>",
    "-<target/tjpgd.c>",
]

if not os.path.exists(lib_json_path):
    if os.path.isdir(lib_dir):
        data = {
            "name": "esp32-camera",
            "frameworks": "espidf",
            "platforms": "*",
            "build": {
                "flags": REQUIRED_FLAGS,
                "includeDir": ".",
                "srcDir": ".",
                "srcFilter": REQUIRED_SRC_FILTER
            }
        }
        with open(lib_json_path, "w") as f:
            json.dump(data, f, indent=2)
        print("Created esp32-camera library.json with target sources")
else:
    with open(lib_json_path, "r") as f:
        data = json.load(f)

    needs_update = False

    # Ensure src filter includes target
    src_filter = data.get("build", {}).get("srcFilter", [])
    for entry in REQUIRED_SRC_FILTER:
        if entry not in src_filter:
            src_filter.append(entry)
            needs_update = True
    data.setdefault("build", {})["srcFilter"] = src_filter

    # Ensure include flags
    flags = data["build"].get("flags", [])
    for flag in REQUIRED_FLAGS:
        if flag not in flags:
            flags.append(flag)
            needs_update = True
    data["build"]["flags"] = flags

    if needs_update:
        with open(lib_json_path, "w") as f:
            json.dump(data, f, indent=2)
        print("Patched esp32-camera library.json")
