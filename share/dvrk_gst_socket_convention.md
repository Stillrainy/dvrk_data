# dVRK GStreamer endpoint convention

All dVRK-managed GStreamer Unix sockets use the canonical abstract name:

```text
@dvrk:<role>:<name>
```

The `@dvrk:` prefix is mandatory. It identifies a socket carrying dVRK frame
metadata and using the dVRK reconnection/data path. Bare `@` socket names are
not dVRK endpoint references.

The fixed roles are:

| Role | Inputs | Outputs |
| --- | --- | --- |
| `stereo_source` | camera pipelines | `left`, `right`, optional combined `stereo` |
| `stereo_alignment` | source `left`, `right` | `stereo` |
| `stereo_rectify` | combined source `stereo` | rectified `stereo` |
| `stereo_display` | processed `stereo` | optional `stereo`, `overlay` |

## JSON fields

Use `gst_input` for an input. It may contain either ordinary GStreamer
pipeline text or a canonical dVRK socket reference. Use `gst_output` for an
output socket; it must always be a canonical `@dvrk:` reference.

For example, a source configuration can be written as:

```json
{
  "camera": {
    "left": {
      "gst_input": "v4l2src device=/dev/video0",
      "gst_output": "@dvrk:stereo_source:left"
    },
    "right": {
      "gst_input": "v4l2src device=/dev/video2",
      "gst_output": "@dvrk:stereo_source:right"
    }
  }
}
```

Alignment consumes those endpoints and publishes one stereo endpoint:

```json
{
  "camera": {
    "left": {"gst_input": "@dvrk:stereo_source:left"},
    "right": {"gst_input": "@dvrk:stereo_source:right"}
  },
  "gst_output": "@dvrk:stereo_alignment:stereo"
}
```

Display accepts either a pipeline or socket input:

```json
{
  "gst_input": "@dvrk:stereo_alignment:stereo",
  "eye_size": {"width": 1920, "height": 1080},
  "gst_output": "@dvrk:stereo_display:stereo"
}
```

The source and alignment executables calculate their standard output names
when `gst_output` is omitted. Display output is optional. `pip_gst_inputs`
replaces the old extra-stream setting and uses nested `gst_input` objects for
monoscopic and stereo picture-in-picture streams.

## Stereo rectification

Use [`stereo_rectify.json`](config/stereo_rectify.json) and the consolidated
`stereo_calibration.yaml` or JSON exported by the `devel` ChArUco calibrator.
`calibration_file` is relative to the rectification config. The export stores
`image_width`, `image_height`, one set of `K`, `D`, `R`, and `P` per eye,
and the joint stereo extrinsics. The saved projection matrices determine the
output geometry. The separate ROS CameraInfo exports remain useful for
downstream vision consumers.

For a matched pair of ROS CameraInfo exports, use
`"camera_info": {"left": "left.yaml", "right": "right.yaml"}` in place of
`calibration_file`. Paths may be absolute or relative to the rectification
JSON. The rectifier follows symlinks each time it starts.

```json
{
  "type": "dvrk_data:stereo_rectify@1.0.0",
  "calibration_file": "stereo_calibration.yaml",
  "backend": "gpu",
  "gst_input": "@dvrk:stereo_source:stereo",
  "gst_output": "@dvrk:stereo_rectify:stereo",
  "camera": {"size": "auto"}
}
```

Use `"backend": "gpu"` or omit it for OpenGL. In a `devel` config that
explicitly selects `"cpu"`, change that value to `"gpu"`. The rectifier
takes a progressive, side-by-side left/right frame at the calibrated size.
It handles five-coefficient `plumb_bob` and eight- or fourteen-coefficient
`rational_polynomial` exports. A single GStreamer shader warps both halves
on GPU textures and the rectifier publishes side-by-side I420 through the
usual FD transport after readback.

```bash
ros2 launch dvrk_data stereo_processing.launch.py \
  config_dir:=/path/to/configs processor:=rectify
```

With `processor:=rectify`, the launcher enables the combined
`stereo_source` socket and runs only `stereo_rectify` as the geometry stage.
With `processor:=alignment`, it runs the existing manual-alignment path.
`processor:=auto` selects the sole available geometry config. If both are
present, choose the processor explicitly. Supply `display_config:=...` to
start the display and direct its socket input to the chosen processor.

For example, the HRSV setup can reuse its existing source and display JSON:

```bash
ros2 launch dvrk_data stereo_processing.launch.py \
  config_dir:=/path/to/cwru-erie-dVRK/sdi \
  source_config:=stereo_source_hd.json \
  rectify_config:=/path/to/stereo_rectify.json \
  display_config:=stereo_display_hrsv_hd.json processor:=rectify
```

When starting components directly, set the source's top-level `gst_output`
or pass `--stereo-output` to enable its combined socket. The rectifier
accepts an input override with `-i`. Frame pairing still depends on capture
synchronization and timestamps.

## GStreamer and discovery

The `@` is removed when constructing GStreamer properties:

```text
unixfdsink socket-path=dvrk:stereo_source:left socket-type=abstract
unixfdsrc  socket-path=dvrk:stereo_source:left socket-type=abstract
```

The shared C++ and Python helpers provide `make`, `resolve`, `build_input`,
`build_sink`, `build_src`, and socket discovery. `resolve` accepts only the
canonical `@dvrk:role:name` form; plain input pipelines are passed through
unchanged.
