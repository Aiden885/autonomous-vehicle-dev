#!/usr/bin/env python3
"""Check that generated ACC/LKS ZMQ boundary nodes are wired to Pangu IO."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]


CHECKS = {
    "ACC input header": (
        ROOT
        / "project/newaccpro4/icvos/src/modules/module_empty"
        / "module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43/src"
        / "AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43.h",
        ("AccZmqBridge", "DECLARE_PUBLISHER(acc_input"),
    ),
    "ACC input source": (
        ROOT
        / "project/newaccpro4/icvos/src/modules/module_empty"
        / "module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43/src"
        / "AccZmqInputModule_b6d54e1b_2577_4492_bcea_dfe707509e43.cc",
        ("CREATE_OUTPUT_RETURN_IF_ERROR(acc_input", "PUB_MSG(acc_input"),
    ),
    "ACC output header": (
        ROOT
        / "project/newaccpro4/icvos/src/modules/module_empty"
        / "module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43/src"
        / "AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43.h",
        (
            "AccZmqBridge",
            "DECLARE_CALLBACK(acc_output",
            "DECLARE_SUBSCRIBER(acc_output)",
        ),
    ),
    "ACC output source": (
        ROOT
        / "project/newaccpro4/icvos/src/modules/module_empty"
        / "module_empty_b6d54e1b_2577_4492_bcea_dfe707509e43/src"
        / "AccZmqOutputModule_b6d54e1b_2577_4492_bcea_dfe707509e43.cc",
        (
            "REGISTER_CALLBACK(",
            "CREATE_INPUT_RETURN_IF_ERROR(",
            "DEFINE_CALLBACK_HEAD(",
            "PublishControl(",
        ),
    ),
    "LKS input header": (
        ROOT
        / "project/lks3/icvos/src/modules/module_empty"
        / "module_empty_a807407f_21e6_442d_a6e4_13a788ec666d/src"
        / "LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d.h",
        ("LksZmqBridge", "DECLARE_PUBLISHER(lks_input"),
    ),
    "LKS input source": (
        ROOT
        / "project/lks3/icvos/src/modules/module_empty"
        / "module_empty_a807407f_21e6_442d_a6e4_13a788ec666d/src"
        / "LksZmqInputModule_a807407f_21e6_442d_a6e4_13a788ec666d.cc",
        ("CREATE_OUTPUT_RETURN_IF_ERROR(lks_input", "PUB_MSG(lks_input"),
    ),
    "LKS output header": (
        ROOT
        / "project/lks3/icvos/src/modules/module_empty"
        / "module_empty_a807407f_21e6_442d_a6e4_13a788ec666d/src"
        / "LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d.h",
        (
            "LksZmqBridge",
            "DECLARE_CALLBACK(lks_output",
            "DECLARE_SUBSCRIBER(lks_output)",
        ),
    ),
    "LKS output source": (
        ROOT
        / "project/lks3/icvos/src/modules/module_empty"
        / "module_empty_a807407f_21e6_442d_a6e4_13a788ec666d/src"
        / "LksZmqOutputModule_a807407f_21e6_442d_a6e4_13a788ec666d.cc",
        (
            "REGISTER_CALLBACK(",
            "CREATE_INPUT_RETURN_IF_ERROR(",
            "DEFINE_CALLBACK_HEAD(",
            "PublishControl(",
        ),
    ),
}


def main() -> int:
    failures = []
    for label, (path, fragments) in CHECKS.items():
        if not path.is_file():
            failures.append(f"{label}: missing {path}")
            continue
        text = path.read_text(encoding="utf-8")
        missing = [fragment for fragment in fragments if fragment not in text]
        if missing:
            failures.append(f"{label}: missing {', '.join(missing)}")

    if failures:
        print("Generated boundary check: FAILED")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print("Generated boundary check: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
