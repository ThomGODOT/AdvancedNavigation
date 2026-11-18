#!/usr/bin/env python

# Build Godot C++ bindings for the selected platform.
env = SConscript(
    "godot-cpp/SConstruct",
    {
        "api_version": "4.7",
    },
)

# Include our extension source directory.
env.Append(CPPPATH=["src/"])

# Collect extension source files.
sources = Glob("src/*.cpp")

# Build the shared library.
library = env.SharedLibrary(
    "project/addons/advanced_ai_nav/bin/libadvanced_ai_nav{}{}".format(
        env["suffix"], env["SHLIBSUFFIX"]
    ),
    source=sources,
)

Default(library)
