# Terrain surfaces (TINs)

CADCraft can load terrain surfaces, draw contours, profiles and cross sections from them, and measure the volume
between two. The geometry is `cadcraft_geom::tin`; the commands are in `crates/engine/src/cmd/surface/`. Surfaces
live in the session (they are not written to the DWG/DXF): load the LandXML again after a restart.

There are no menu entries yet. Run the commands from the command line with a JSON parameter object
(`surfacecontours {"surface":"EG","interval":1}`), from a script, the CLI (`--cmd`), the control channel or MCP.

| Command | Parameters | Result |
|---|---|---|
| `landxmlin` | `path`, `names?` (name substrings), `maxEdge?` (drop triangles with a longer plan edge) | surfaces loaded |
| `surface.create` | `name`, `points: [[x,y,z],...]`, `maxEdge?` | Delaunay surface |
| `surface.list`, `surface.delete` | `name` or `all` | |
| `surfacecontours` | `surface`, `interval`, `majorEvery?`, `window?`, `from?`, `to?`, `labels?` | polylines on `CONTOUR-MINOR` / `CONTOUR-MAJOR` at their elevation |
| `surfacetin` | `surface`, `window?`, `layer?` | triangle edges |
| `surfacesection` | `path`, `surfaces`, `at`, `hscale?`, `vscale?` | a profile graph of each surface along the path |
| `surfacesections` | `alignment`, `surfaces`, `startStation?`, `interval?`, `width?`, `at`, `columns?`, `divisor?` | cross sections at stations; fill and cut area between the first two surfaces and the average-end-area volume |
| `surfacevolume` | `top`, `base`, `region?` | `fill`, `cut`, `net`, `area` |
| `surfacestrips` | `top`, `base`, `alignment`, `startStation?`, `from?`, `to?`, `step?`, `width?`, `side?` | volume per station strip |

Volumes are exact for planar triangles: each overlap of a top triangle and a base triangle is cut where the two
surfaces cross, so fill and cut are separate. `surfacestrips` puts strip edges on multiples of `step` in station.

## Limits

- Surfaces made from points have no breaklines, and triangles at the outer hull can be lost; surfaces read from
  LandXML keep the file's own triangles.
- An alignment is a polyline of points (flatten arcs first); there are no alignment objects.
- At most 4,000,000 points and 8,000,000 triangles per surface; a drawing command adds at most 400,000 objects.
