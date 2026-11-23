"""Select one stereo geometry processor, with an optional display."""

import json
from pathlib import Path
import re
import tempfile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SOCKET = re.compile(r"@dvrk:[A-Za-z0-9_.-]+:[A-Za-z0-9_.-]+")
UNIXFD = re.compile(r"""(\bsocket-path=)(?:'[^']*'|"[^"]*"|[^\s!]+)""")


def _output_socket(config, default):
    output = config.get("gst_output", default)
    if not isinstance(output, str) or not SOCKET.fullmatch(output):
        raise RuntimeError("Expected a configured @dvrk:role:name output socket")
    return output


def _connect_display(config, socket):
    pipeline = config.get("gst_input", "")
    if isinstance(pipeline, str) and SOCKET.fullmatch(pipeline):
        config["gst_input"] = socket
        return
    if isinstance(pipeline, str) and pipeline.lstrip().startswith("unixfdsrc "):
        # Keep filters such as the HRSV videoscale chain after the socket.
        source, separator, downstream = pipeline.partition("!")
        source, count = UNIXFD.subn(lambda match: match[1] + socket[1:], source, count=1)
        if count:
            config["gst_input"] = source + separator + downstream
            return
    raise RuntimeError("Display gst_input must begin with a dVRK socket or unixfdsrc")


def _launch(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    directory = Path(value("config_dir")).expanduser().resolve()
    source = directory / Path(value("source_config")).expanduser()
    if not source.is_file():
        raise RuntimeError(f"Missing source configuration: {source}")

    geometries = {
        mode: directory / Path(value(f"{mode}_config")).expanduser()
        for mode in ("alignment", "rectify")
    }
    mode = value("processor")
    if mode == "auto":
        available = [key for key, path in geometries.items() if path.is_file()]
        if len(available) != 1:
            raise RuntimeError(
                "Choose processor:=alignment or processor:=rectify when both configs exist."
                if available else "No stereo_alignment or stereo_rectify configuration found."
            )
        mode = available[0]
    geometry = geometries[mode]
    if not geometry.is_file():
        raise RuntimeError(f"Missing {mode} configuration: {geometry}")

    source_args = ["-c", str(source)]
    processor_args = ["-c", str(geometry)]
    if mode == "rectify":
        source_socket = _output_socket(json.loads(source.read_text()), "@dvrk:stereo_source:stereo")
        source_args.append("--stereo-output")
        processor_args.extend(["-i", source_socket])

    source_node = Node(package="dvrk_data", executable="stereo_source",
                       output="screen", arguments=source_args)
    processor = Node(package="dvrk_data", executable=f"stereo_{mode}",
                     output="screen", arguments=processor_args)
    actions = [
        RegisterEventHandler(OnProcessExit(target_action=processor,
                                           on_exit=[Shutdown(reason="Stereo processor exited")])),
        source_node,
        processor,
    ]

    display = value("display_config")
    if display:
        output = _output_socket(json.loads(geometry.read_text()), f"@dvrk:stereo_{mode}:stereo")
        display_path = directory / Path(display).expanduser()
        config = json.loads(display_path.read_text())
        _connect_display(config, output)
        temporary = tempfile.TemporaryDirectory(prefix="dvrk-stereo-display-")
        configured_path = Path(temporary.name) / "stereo_display.json"
        configured_path.write_text(json.dumps(config))
        actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda context: temporary.cleanup())
        ])))
        actions.append(Node(package="dvrk_console", executable="stereo_display",
                            output="screen", arguments=["-c", str(configured_path)]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_dir", default_value="."),
        DeclareLaunchArgument("source_config", default_value="stereo_source.json"),
        DeclareLaunchArgument("processor", default_value="auto",
                              choices=["auto", "alignment", "rectify"]),
        DeclareLaunchArgument("alignment_config", default_value="stereo_alignment.json"),
        DeclareLaunchArgument("rectify_config", default_value="stereo_rectify.json"),
        DeclareLaunchArgument("display_config", default_value="",
                              description="Optional dvrk_console display JSON."),
        OpaqueFunction(function=_launch),
    ])
